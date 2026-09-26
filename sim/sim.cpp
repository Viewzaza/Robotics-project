// Host-side simulator for the e33 line-following mission.
// Models the field, differential-drive kinematics, the 8-channel reflectance
// bar mounted 9.5 cm ahead of the pivot, and the gripper -- then runs the real
// firmware (setup()/loop()) against it.
//
// Build:  g++ -O2 -std=c++14 -I sim -o sim/sim.exe sim/sim.cpp
// Run:    ./sim/sim.exe [--quiet] [--trim=0.95] [--vmax=40] [--cell=25] [--grip=12]
//         ./sim/sim.exe --help        (every flag, including the realism ones)
//
// Real-robot effects (all OFF by default, so default runs are unchanged):
//   sensors : --white=N --black=N --chan=K:W:B --weakch=K --weakgain=G
//             --edge=CM --noise=N --seed=S
//   motors  : --trimL=G (left gain; --trim is the right gain) --dbL=N --dbR=N --db=N
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
  double deadband    = 25.0;  // duty below which the motor does not turn (both, --db)
  double dbL         = -1;    // per-motor dead band; <0 means "use deadband" (--dbL)
  double dbR         = -1;    //                                              (--dbR)
  double trimL       = 1.00;  // left motor gain (1.0 = perfectly matched) (--trimL)
  double trimR       = 1.00;  // right motor gain                          (--trim)
  double gripRadius  = 3.5;   // cm, how close the jaws must be to take an object
  double objBeyond   = 5.0;   // cm, how far past the outer line objects sit
  double overhang    = 12.0;  // cm, how far TOP/MID/BOT stick out past C1 and C4 (--overhang)
  double trimLrev    = -1;    // left gain driving backward; <0 = same as forward (--trimLr)
  double trimRrev    = -1;    // right gain driving backward                     (--trimRr)

  // ---- analog sensor realism. The defaults reproduce the original ideal
  // ---- bar exactly: 100 over white, 900 over black, a hard step at the tape
  // ---- edge, no noise.
  double white       = 100.0; // count over white, all channels   (--white)
  double black       = 900.0; // count over black, all channels   (--black)
  unsigned weakMask  = 0;     // bit K set = channel K is weak    (--weakch=K, repeatable)
  double weakGain    = 0.30;  // weak channel keeps this fraction of its contrast (--weakgain)
  double edge        = 0.0;   // cm, width of the soft white->black ramp at the tape edge (--edge)
  double noise       = 0.0;   // counts, standard deviation of additive noise (--noise)
  unsigned long seed = 1;     // noise generator seed; same seed = same run (--seed)
} CFG;

// Per-channel white/black levels, filled from CFG after the flags are parsed.
// Channel 0 (A0) is the LEFT-most sensor.
static double w_chWhite[8], w_chBlack[8];
static bool   w_chSet[8];             // set by --chan=K:W:B, beats --white/--black/--weakch
static double w_chSetW[8], w_chSetB[8];

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
  const double oh = CFG.overhang;
  for (int i = 0; i < 3; i++) w_lines.push_back({C1 - oh, ys[i], C4 + oh, ys[i]});
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

static double distToNearestLine(double px, double py) {
  double best = 1e9;
  for (size_t i = 0; i < w_lines.size(); i++) {
    double d = distToSeg(px, py, w_lines[i]);
    if (d < best) best = d;
  }
  return best;
}

// How much of the sensor's view is tape, 0 (all white) .. 1 (all black).
// edge == 0: the original hard step at the tape edge.
// edge  > 0: a smooth ramp of that width centred on the tape edge, so the
//            reading is exactly halfway at the edge and saturates edge/2
//            inside / outside it. A tape narrower than the ramp never reaches
//            full black -- which is what a real, slightly defocused sensor does.
static double tapeCoverage(double px, double py) {
  if (CFG.edge <= 0) return onLine(px, py) ? 1.0 : 0.0;
  double s = distToNearestLine(px, py) - CFG.lineHalfW;   // <0 inside the tape
  double t = 0.5 - s / CFG.edge;
  if (t <= 0) return 0.0;
  if (t >= 1) return 1.0;
  return t * t * (3.0 - 2.0 * t);                         // smoothstep
}

// Deterministic noise: xorshift64*, seeded from --seed. Never from time, so a
// run can be repeated exactly.
static unsigned long long w_rng = 1;
static double rngUniform() {                              // [0,1)
  w_rng ^= w_rng >> 12; w_rng ^= w_rng << 25; w_rng ^= w_rng >> 27;
  return (double)((w_rng * 2685821657736338717ULL) >> 11) * (1.0 / 9007199254740992.0);
}
static double rngGauss() {                                // mean 0, sd 1 (Irwin-Hall, 12 terms)
  double a = 0;
  for (int i = 0; i < 12; i++) a += rngUniform();
  return a - 6.0;
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
// Below its own dead band a motor does not turn at all; above it, speed rises
// linearly to vMax*trim at duty 255.
static double wheelV(int fPin, int bPin, int pwmPin, double trim, double db, double trimRev) {
  if (w_digital[10] == 0) return 0;          // STBY low: driver disabled
  int f = w_digital[fPin], b = w_digital[bPin];
  if (f == b) return 0;                      // both high = brake, both low = coast
  double duty = w_pwm[pwmPin];
  if (duty <= db) return 0;
  bool fwd = (f == 1 && b == 0);
  double g = (!fwd && trimRev >= 0) ? trimRev : trim;
  double v = (duty - db) / (255.0 - db) * CFG.vMax * g;
  return fwd ? v : -v;
}

static void stepPhysics(double dt_s) {
  double vL = wheelV(12, 11, 6, CFG.trimL, CFG.dbL, CFG.trimLrev);
  double vR = wheelV(7, 8, 3, CFG.trimR, CFG.dbR, CFG.trimRrev);
  double v = (vL + vR) / 2.0;
  double omega = (vR - vL) / CFG.track;
  w_rob.x += v * std::cos(w_rob.th) * dt_s;
  w_rob.y += v * std::sin(w_rob.th) * dt_s;
  w_rob.th += omega * dt_s;
  w_rob.odo += std::fabs(v) * dt_s;
  if (w_heldIdx >= 0) gripPoint(w_objs[w_heldIdx].x, w_objs[w_heldIdx].y);
}

// Safety net for firmware that never returns from setup() or loop() (a
// calibration run, a fault halt, a "mission done" idle loop): end the run
// when simulated time runs out or the robot has been still for w_stillSec.
static double w_maxSimSec = 240.0;
static double w_stillSec  = 10.0;
static double w_netStill  = 0, w_netOdo = -1;
static void finishRun(const char* why);

static void advance(double us) {
  const double MAXSTEP = 1000.0;             // integrate in <=1 ms slices
  while (us > 0) {
    double chunk = us > MAXSTEP ? MAXSTEP : us;
    stepPhysics(chunk / 1e6);
    w_t_us += chunk;
    us -= chunk;
  }
  if (std::fabs(w_rob.odo - w_netOdo) > 1e-9) { w_netOdo = w_rob.odo; w_netStill = w_t_us; }
  else if (w_t_us - w_netStill > w_stillSec * 1e6) finishRun("robot still (firmware idle or halted)");
  if (w_t_us / 1e6 > w_maxSimSec + 1.0) finishRun("time cap");
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
  // With every realism flag at its default this is exactly the original
  // "onLine ? 900 : 100": coverage is 0 or 1, 100 + 800*c, and no noise draw.
  double c = tapeCoverage(bx, by);
  double v = w_chWhite[idx] + (w_chBlack[idx] - w_chWhite[idx]) * c;
  if (CFG.noise > 0) v += CFG.noise * rngGauss();
  long r = std::lround(v);
  if (r < 0) r = 0; else if (r > 1023) r = 1023;          // the 10-bit ADC range
  return (int)r;
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
static void usage() {
  printf(
    "usage: sim [flags]\n"
    "  run / output\n"
    "    --quiet            only the RESULT block\n"
    "    --serial           echo what the firmware prints on Serial\n"
    "    --secs=S           simulated-time cap (default 240)\n"
    "  field / geometry\n"
    "    --cell=CM          grid pitch (default 25)\n"
    "    --grip=CM          pivot -> gripper jaws (default 12)\n"
    "  motors (dead band and gain are per motor; defaults = perfectly matched)\n"
    "    --vmax=CMS         wheel speed at duty 255 with gain 1 (default 40)\n"
    "    --trim=G           RIGHT motor gain (default 1.00)\n"
    "    --trimL=G          LEFT motor gain (default 1.00; the real robot is ~0.75)\n"
    "    --db=N             dead band of both motors, duty 0-255 (default 25)\n"
    "    --dbL=N --dbR=N    dead band of one motor (default: --db)\n"
    "  sensors (defaults = the ideal bar: 100 white, 900 black, hard edge, no noise)\n"
    "    --white=N          reading over white, every channel (default 100)\n"
    "    --black=N          reading over black, every channel (default 900)\n"
    "    --chan=K:W:B       channel K (0 = A0 = left-most) reads W on white, B on black\n"
    "    --weakch=K         channel K keeps only --weakgain of its contrast (repeatable)\n"
    "    --weakgain=G       contrast fraction a weak channel keeps (default 0.30)\n"
    "    --edge=CM          width of the soft white->black ramp at the tape edge (default 0 = hard)\n"
    "    --noise=N          additive noise, standard deviation in counts (default 0)\n"
    "    --seed=S           noise seed; same seed, same run (default 1)\n"
    "  field, start pose, memory\n"
    "    --overhang=CM      how far the lines stick out past C1 and C4 (default 12)\n"
    "    --trimLr=G --trimRr=G  motor gain driving BACKWARD (default: same as forward)\n"
    "    --startx=CM --starty=CM --starth=DEG   axle position and heading (default -5 0 0;\n"
    "                       robot_e33 is placed with the axle over C1: --startx=0)\n"
    "    --eeprom=FILE      load EEPROM from FILE at the start, save it back at the end\n"
    "    --still=S          end a run whose firmware has idled this long (default 10)\n");
}

static bool flagVal(const char* arg, const char* name, const char** val) {
  size_t n = strlen(name);
  if (strncmp(arg, name, n)) return false;
  *val = arg + n;
  return true;
}

static const char* w_eepromFile = 0;
static double w_startX = -5.0, w_startY = 0.0, w_startH = 0.0;

int main(int argc, char** argv) {
  double& maxSimSec = w_maxSimSec;
  for (int i = 1; i < argc; i++) {
    const char* v = 0;
    if (!strcmp(argv[i], "--quiet")) w_verbose = false;
    else if (!strcmp(argv[i], "--serial")) Serial.echo = true;  /* show what the firmware prints */
    else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) { usage(); return 0; }
    else if (!strncmp(argv[i], "--trim=", 7)) CFG.trimR = atof(argv[i] + 7);
    else if (!strncmp(argv[i], "--vmax=", 7)) CFG.vMax = atof(argv[i] + 7);
    else if (!strncmp(argv[i], "--cell=", 7)) CFG.cell = atof(argv[i] + 7);
    else if (!strncmp(argv[i], "--grip=", 7)) CFG.gripReach = atof(argv[i] + 7);
    else if (!strncmp(argv[i], "--secs=", 7)) maxSimSec = atof(argv[i] + 7);
    /* ---- real-robot effects, all default OFF ---- */
    else if (flagVal(argv[i], "--trimL=", &v))    CFG.trimL = atof(v);
    else if (flagVal(argv[i], "--db=", &v))       CFG.deadband = atof(v);
    else if (flagVal(argv[i], "--dbL=", &v))      CFG.dbL = atof(v);
    else if (flagVal(argv[i], "--dbR=", &v))      CFG.dbR = atof(v);
    else if (flagVal(argv[i], "--white=", &v))    CFG.white = atof(v);
    else if (flagVal(argv[i], "--black=", &v))    CFG.black = atof(v);
    else if (flagVal(argv[i], "--weakgain=", &v)) CFG.weakGain = atof(v);
    else if (flagVal(argv[i], "--edge=", &v))     CFG.edge = atof(v);
    else if (flagVal(argv[i], "--noise=", &v))    CFG.noise = atof(v);
    else if (flagVal(argv[i], "--seed=", &v))     CFG.seed = strtoul(v, 0, 10);
    else if (flagVal(argv[i], "--overhang=", &v)) CFG.overhang = atof(v);
    else if (flagVal(argv[i], "--trimLr=", &v))   CFG.trimLrev = atof(v);
    else if (flagVal(argv[i], "--trimRr=", &v))   CFG.trimRrev = atof(v);
    else if (flagVal(argv[i], "--startx=", &v))   w_startX = atof(v);
    else if (flagVal(argv[i], "--starty=", &v))   w_startY = atof(v);
    else if (flagVal(argv[i], "--starth=", &v))   w_startH = atof(v);
    else if (flagVal(argv[i], "--eeprom=", &v))   w_eepromFile = v;
    else if (flagVal(argv[i], "--still=", &v))    w_stillSec = atof(v);
    else if (flagVal(argv[i], "--weakch=", &v)) {
      int k = atoi(v);
      if (k < 0 || k > 7) { fprintf(stderr, "sim: --weakch wants 0..7, got %s\n", v); return 2; }
      CFG.weakMask |= 1u << k;
    }
    else if (flagVal(argv[i], "--chan=", &v)) {
      int k; double cw, cb;
      if (sscanf(v, "%d:%lf:%lf", &k, &cw, &cb) != 3 || k < 0 || k > 7) {
        fprintf(stderr, "sim: --chan wants K:W:B with K 0..7, got %s\n", v); return 2;
      }
      w_chSet[k] = true; w_chSetW[k] = cw; w_chSetB[k] = cb;
    }
    else fprintf(stderr, "sim: ignoring unknown flag %s (see --help)\n", argv[i]);
  }

  // Resolve per-motor dead bands and per-channel sensor levels.
  if (CFG.dbL < 0) CFG.dbL = CFG.deadband;
  if (CFG.dbR < 0) CFG.dbR = CFG.deadband;
  for (int k = 0; k < 8; k++) {
    w_chWhite[k] = CFG.white;
    w_chBlack[k] = CFG.black;
    if (CFG.weakMask & (1u << k))
      w_chBlack[k] = w_chWhite[k] + (w_chBlack[k] - w_chWhite[k]) * CFG.weakGain;
    if (w_chSet[k]) { w_chWhite[k] = w_chSetW[k]; w_chBlack[k] = w_chSetB[k]; }
  }
  w_rng = CFG.seed ? (unsigned long long)CFG.seed : 0x9E3779B97F4A7C15ULL;  /* xorshift must not start at 0 */

  for (int i = 0; i < 24; i++) { w_digital[i] = 0; w_pwm[i] = 0; w_servoAngle[i] = 90; }
  buildField();

  // start pose: on the MID line at C1, facing east, bar just past the C1 crossing
  w_rob.x = w_startX; w_rob.y = w_startY; w_rob.th = w_startH * M_PI / 180.0;
  if (w_eepromFile) EEPROM.load(w_eepromFile);
  logEvent("START   pose=(%.1f, %.1f) heading=%.0fdeg  sensorAhead=%.1fcm cell=%.0fcm",
           w_rob.x, w_rob.y, w_rob.th * 180 / M_PI, CFG.sensorAhead, CFG.cell);

  // Only when a realism effect is on, say so -- default output stays byte-identical.
  bool motorFx = CFG.trimL != 1.0 || CFG.dbL != 25.0 || CFG.dbR != 25.0 ||
                 CFG.trimLrev >= 0 || CFG.trimRrev >= 0;
  bool sensFx = false;
  for (int k = 0; k < 8; k++) if (w_chWhite[k] != 100.0 || w_chBlack[k] != 900.0) sensFx = true;
  if (CFG.edge > 0 || CFG.noise > 0) sensFx = true;
  if (motorFx)
    logEvent("MOTORS  left gain %.2f dead band %.0f | right gain %.2f dead band %.0f | vmax %.0f cm/s",
             CFG.trimL, CFG.dbL, CFG.trimR, CFG.dbR, CFG.vMax);
  if (sensFx) {
    char buf[200]; int n = 0;
    for (int k = 0; k < 8; k++)
      n += snprintf(buf + n, sizeof buf - n, " %d/%d", (int)std::lround(w_chWhite[k]), (int)std::lround(w_chBlack[k]));
    logEvent("SENSORS white/black A0..A7:%s  edge %.2f cm  noise sd %.0f  seed %lu",
             buf, CFG.edge, CFG.noise, CFG.seed);
  }

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

  finishRun(0);
  return 0;
}

// Prints the RESULT block and ends the process. Called at the end of main(),
// or by the safety net in advance() when the firmware never returns.
static void finishRun(const char* why) {
  static bool done = false;
  if (done) return;
  done = true;
  if (why) logEvent("HALT    %s", why);
  if (w_eepromFile) EEPROM.save(w_eepromFile);
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
  fflush(stdout);
  exit(ok == 3 ? 0 : 1);
}
