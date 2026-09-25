// Host-side simulator for the e33 line-following mission.
// Models the field, differential-drive kinematics, the 8-channel reflectance
// bar mounted 9.5 cm ahead of the pivot, and the gripper -- then runs the real
// firmware (setup()/loop()) against it.
//
// Build:  g++ -O2 -I sim -o sim/sim sim/sim.cpp
// Run:    ./sim/sim [--quiet] [--trim=0.95] [--vmax=40] [--cell=25] [--grip=12]
#include <cstdarg>
#define _USE_MATH_DEFINES
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "Arduino.h"
#include <vector>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

SerialClass Serial;
#include "EEPROM.h"
EEPROMClass EEPROM;

// ----------------------------- tunables ------------------------------------
struct Cfg {
  double cell        = 25.0;  // cm between grid lines
  double lineHalfW   = 0.9;   // cm, half the tape width
  double sensorAhead = 9.5;   // cm, bar -> pivot centre (the measured value)
  double sensorPitch = 1.2;   // cm between adjacent sensors
  double gripReach   = 12.0;  // cm, pivot -> gripper jaws
  double track       = 11.0;  // cm between wheel contact patches
  double vMax        = 40.0;  // cm/s at duty 255
  double deadband    = 25.0;  // duty below which the motor does not turn
  double trimL       = 1.00;  // left motor gain (1.0 = perfectly matched)
  double trimR       = 1.00;  // right motor gain
  double gripRadius  = 3.5;   // cm, how close the jaws must be to take an object
  double objBeyond   = 5.0;   // cm, how far past the outer line objects sit
} CFG;

// ----------------------------- field ---------------------------------------
struct Seg { double x1, y1, x2, y2; };
static std::vector<Seg> w_lines;

struct Obj {
  std::string name;
  double x, y;
  bool held = false;
  bool placed = false;
};
static std::vector<Obj> w_objs;

struct Target { std::string name; double x, y; };
static std::vector<Target> w_targets;

static void buildField() {
  const double c = CFG.cell;
  const double C1 = 0, C2 = c, C3 = 2 * c, C4 = 3 * c;
  const double TOP = c, MID = 0, BOT = -c;
  double ys[3] = {TOP, MID, BOT};
  for (int i = 0; i < 3; i++) w_lines.push_back({C1 - 12, ys[i], C4 + 12, ys[i]});
  double xs[4] = {C1, C2, C3, C4};
  for (int i = 0; i < 4; i++) w_lines.push_back({xs[i], BOT, xs[i], TOP});

  const double d = CFG.objBeyond;
  w_objs.push_back({"o1", C1, TOP + d, false, false});
  w_objs.push_back({"o2", C1, BOT - d, false, false});
  w_objs.push_back({"o3", C4, TOP + d, false, false});
  w_targets.push_back({"x1", C4, BOT - d});
  w_targets.push_back({"x2", C3, BOT - d});
  w_targets.push_back({"x3", C2, BOT - d});
}

static double distToSeg(double px, double py, const Seg& s) {
  double vx = s.x2 - s.x1, vy = s.y2 - s.y1;
  double wx = px - s.x1, wy = py - s.y1;
  double L2 = vx * vx + vy * vy;
  double t = L2 > 0 ? (wx * vx + wy * vy) / L2 : 0;
  if (t < 0) t = 0; else if (t > 1) t = 1;
  double cx = s.x1 + t * vx, cy = s.y1 + t * vy;
  return std::hypot(px - cx, py - cy);
}

static bool onLine(double px, double py) {
  for (size_t i = 0; i < w_lines.size(); i++)
    if (distToSeg(px, py, w_lines[i]) <= CFG.lineHalfW) return true;
  return false;
}

// ----------------------------- robot ---------------------------------------
struct RobotState {
  double x = 0, y = 0, th = 0;   // th: radians, CCW from +x
  double odo = 0;                // total path length, cm
} w_rob;

static int    w_digital[24];
static int    w_pwm[24];
static int    w_servoAngle[24];
static double w_t_us = 0;
static bool   w_gripClosed = false;
static int    w_heldIdx = -1;
static bool   w_verbose = true;

static void gripPoint(double& gx, double& gy) {
  gx = w_rob.x + CFG.gripReach * std::cos(w_rob.th);
  gy = w_rob.y + CFG.gripReach * std::sin(w_rob.th);
}

static void logEvent(const char* fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  printf("[%7.2fs] %s\n", w_t_us / 1e6, buf);
}

// direction pins + duty -> wheel velocity in cm/s
static double wheelV(int fPin, int bPin, int pwmPin, double trim) {
  if (w_digital[10] == 0) return 0;          // STBY low: driver disabled
  int f = w_digital[fPin], b = w_digital[bPin];
  if (f == b) return 0;                      // both high = brake, both low = coast
  double duty = w_pwm[pwmPin];
  if (duty <= CFG.deadband) return 0;
  double v = (duty - CFG.deadband) / (255.0 - CFG.deadband) * CFG.vMax * trim;
  return (f == 1 && b == 0) ? v : -v;
}

static void stepPhysics(double dt_s) {
  double vL = wheelV(12, 11, 6, CFG.trimL);
  double vR = wheelV(7, 8, 3, CFG.trimR);
  double v = (vL + vR) / 2.0;
  double omega = (vR - vL) / CFG.track;
  w_rob.x += v * std::cos(w_rob.th) * dt_s;
  w_rob.y += v * std::sin(w_rob.th) * dt_s;
  w_rob.th += omega * dt_s;
  w_rob.odo += std::fabs(v) * dt_s;
  if (w_heldIdx >= 0) gripPoint(w_objs[w_heldIdx].x, w_objs[w_heldIdx].y);
}

static void advance(double us) {
  const double MAXSTEP = 1000.0;             // integrate in <=1 ms slices
  while (us > 0) {
    double chunk = us > MAXSTEP ? MAXSTEP : us;
    stepPhysics(chunk / 1e6);
    w_t_us += chunk;
    us -= chunk;
  }
}

// ----------------------------- Arduino API ---------------------------------
// On AVR these are not free: micros() is ~4 us of work, millis() ~2 us. The
// sim must charge for them, otherwise a busy-wait like
//   while (millis() - t0 < ms) { odoTick(); }
// consumes no simulated time and hangs forever.
unsigned long millis() { advance(2); return (unsigned long)(w_t_us / 1000.0); }
unsigned long micros() { advance(4); return (unsigned long)w_t_us; }
void delay(unsigned long ms) { advance((double)ms * 1000.0); }
void delayMicroseconds(unsigned long us) { advance((double)us); }
void pinMode(int pin, int mode) {
  if (pin >= 0 && pin < 24 && mode == INPUT_PULLUP) w_digital[pin] = 1;
}
void digitalWrite(int pin, int v) { if (pin >= 0 && pin < 24) w_digital[pin] = v; advance(1); }
int  digitalRead(int pin) { advance(1); return (pin >= 0 && pin < 24) ? w_digital[pin] : 0; }
void analogWrite(int pin, int v) { if (pin >= 0 && pin < 24) w_pwm[pin] = v; advance(1); }

int analogRead(int pin) {
  advance(104);                              // a real analogRead costs ~104 us
  int idx = pin - A0;
  if (idx < 0 || idx > 7) return 0;
  // channel 0 (A0) is the LEFT-most sensor
  double off = (3.5 - idx) * CFG.sensorPitch;
  double bx = w_rob.x + CFG.sensorAhead * std::cos(w_rob.th) - off * std::sin(w_rob.th);
  double by = w_rob.y + CFG.sensorAhead * std::sin(w_rob.th) + off * std::cos(w_rob.th);
  return onLine(bx, by) ? 900 : 100;
}

void sim_servo_write(int pin, int angle) {
  if (pin >= 0 && pin < 24) w_servoAngle[pin] = angle;
  if (pin != 5) return;                      // only the gripper servo matters here
  /* Assumes SERVO_GRIP_CLOSED < 80 < SERVO_GRIP_OPEN. config.h is not visible
   * this early in the file, so this stays a literal -- check it if you retune
   * the gripper angles. */
  bool closing = angle <= 80;
  if (closing && !w_gripClosed) {
    double gx, gy;
    gripPoint(gx, gy);
    int found = -1;
    for (size_t i = 0; i < w_objs.size(); i++) {
      if (w_objs[i].held || w_objs[i].placed) continue;
      double d = std::hypot(w_objs[i].x - gx, w_objs[i].y - gy);
      if (d <= CFG.gripRadius) { found = (int)i; break; }
    }
    if (found >= 0) {
      w_heldIdx = found;
      w_objs[found].held = true;
      logEvent("GRIP    %s at (%.1f, %.1f)", w_objs[found].name.c_str(), gx, gy);
    } else {
      logEvent("GRIP    MISSED -- nothing within %.1f cm of (%.1f, %.1f)",
               CFG.gripRadius, gx, gy);
    }
    w_gripClosed = true;
  } else if (!closing && w_gripClosed) {
    if (w_heldIdx >= 0) {
      double gx, gy;
      gripPoint(gx, gy);
      w_objs[w_heldIdx].x = gx;
      w_objs[w_heldIdx].y = gy;
      w_objs[w_heldIdx].held = false;
      w_objs[w_heldIdx].placed = true;
      logEvent("RELEASE %s at (%.1f, %.1f)", w_objs[w_heldIdx].name.c_str(), gx, gy);
      w_heldIdx = -1;
    }
    w_gripClosed = false;
  }
}

// ----------------------------- firmware under test --------------------------
/* Which sketch is under test. Override to compare them:
 *   g++ -O2 -std=c++14 -I sim -DFIRMWARE_INO='"../robot_basic/robot_basic.ino"'  *       -o sim/sim_basic.exe sim/sim.cpp */
#ifndef FIRMWARE_INO
#define FIRMWARE_INO "../robot_countGride/robot_countGride.ino"
#endif
#include FIRMWARE_INO

// ----------------------------- runner ---------------------------------------
int main(int argc, char** argv) {
  double maxSimSec = 240.0;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--quiet")) w_verbose = false;
    else if (!strcmp(argv[i], "--serial")) Serial.echo = true;  /* show what the firmware prints */
    else if (!strncmp(argv[i], "--trim=", 7)) CFG.trimR = atof(argv[i] + 7);
    else if (!strncmp(argv[i], "--vmax=", 7)) CFG.vMax = atof(argv[i] + 7);
    else if (!strncmp(argv[i], "--cell=", 7)) CFG.cell = atof(argv[i] + 7);
    else if (!strncmp(argv[i], "--grip=", 7)) CFG.gripReach = atof(argv[i] + 7);
    else if (!strncmp(argv[i], "--secs=", 7)) maxSimSec = atof(argv[i] + 7);
  }
  for (int i = 0; i < 24; i++) { w_digital[i] = 0; w_pwm[i] = 0; w_servoAngle[i] = 90; }
  buildField();

  // start pose: on the MID line at C1, facing east, bar just past the C1 crossing
  w_rob.x = -5.0; w_rob.y = 0.0; w_rob.th = 0.0;
  logEvent("START   pose=(%.1f, %.1f) heading=%.0fdeg  sensorAhead=%.1fcm cell=%.0fcm",
           w_rob.x, w_rob.y, w_rob.th * 180 / M_PI, CFG.sensorAhead, CFG.cell);

  setup();

  int lastCount = -1;
  double nextSample = 0;
  double stillSince = -1;
  double lastOdo = -1;
  while (w_t_us / 1e6 < maxSimSec) {
    loop();
    if ((int)numGride != lastCount) {
      lastCount = (int)numGride;
      if (w_verbose)
        logEvent("COUNT   numGride=%-3d pose=(%6.1f,%6.1f) heading=%4.0fdeg",
                 lastCount, w_rob.x, w_rob.y, w_rob.th * 180 / M_PI);
    }
    if (w_verbose && w_t_us / 1e6 > nextSample) {
      nextSample = w_t_us / 1e6 + 10.0;
      printf("           ...  pose=(%6.1f,%6.1f) heading=%4.0fdeg odo=%.0fcm n=%d\n",
             w_rob.x, w_rob.y, w_rob.th * 180 / M_PI, w_rob.odo, (int)numGride);
    }
    if (std::fabs(w_rob.odo - lastOdo) < 1e-9) {
      if (stillSince < 0) stillSince = w_t_us;
      else if (w_t_us - stillSince > 3e6) { logEvent("HALT    robot stationary"); break; }
    } else {
      stillSince = -1;
      lastOdo = w_rob.odo;
    }
  }

  printf("\n================ RESULT ================\n");
  printf("sim time %.1f s   path %.0f cm   final numGride %d\n",
         w_t_us / 1e6, w_rob.odo, (int)numGride);
  printf("final pose (%.1f, %.1f) heading %.0f deg\n",
         w_rob.x, w_rob.y, w_rob.th * 180 / M_PI);

  const char* want[3][2] = {{"o1", "x1"}, {"o2", "x2"}, {"o3", "x3"}};
  int ok = 0;
  for (int i = 0; i < 3; i++) {
    const Obj* o = 0;
    const Target* t = 0;
    for (size_t a = 0; a < w_objs.size(); a++)    if (w_objs[a].name == want[i][0]) o = &w_objs[a];
    for (size_t b = 0; b < w_targets.size(); b++) if (w_targets[b].name == want[i][1]) t = &w_targets[b];
    double d = std::hypot(o->x - t->x, o->y - t->y);
    bool good = d <= 6.0;
    if (good) ok++;
    printf("  %s -> %s : %s (object %.1f,%.1f ; target %.1f,%.1f ; off by %.1f cm)\n",
           want[i][0], want[i][1], good ? "PLACED" : "FAILED",
           o->x, o->y, t->x, t->y, d);
  }
  printf("score %d/3\n", ok);
  return ok == 3 ? 0 : 1;
}
