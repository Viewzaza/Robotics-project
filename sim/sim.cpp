// Host-side simulator for the e33 line-following mission.
// Models the field, differential-drive kinematics, the 8-channel reflectance
// bar mounted 9.5 cm ahead of the pivot, and the gripper, then runs the real
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
//   supply  : --vbat=V --rint=OHM ... (only the firmware's VCC reading and the
//             brown-out check; the defaults are a fresh 9 V PP3, see --help)
//   presets : --real (lag, scrub, sag), --student / --student-slow / --student-fast
//             (the student's robot: see applyStudent() and --help)
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
  double dbLrev      = -1;    // left dead band driving backward; <0 = same as forward (--dbLr)
  double dbRrev      = -1;    // right dead band driving backward                     (--dbRr)

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

  // ---- physical realism (all OFF by default: the ideal model is unchanged).
  // ---- --real turns on the set of plausible values listed in usage().
  double tau         = 0.0;   // ms, motor+robot speed lag while driven (--tau)
  double taub        = -1;    // ms, lag while short-braking; <0 = 0.75*tau (--taub)
  double dbRun       = 1.0;   // running dead band as a fraction of the start one (--dbrun)
  double scrub       = 0.0;   // fraction of turn rate lost in a pure spin (--scrub)
  double spinJit     = 0.0;   // relative unevenness of the spin rate, sd (--spinjit)
  double pivot       = 0.0;   // cm, turning centre ahead (+) of the axle in spins (--pivot)
  double pivotJit    = 0.0;   // cm, sd of the wandering of that centre (--pivotjit)
  double sag         = 0.0;   // speed fraction lost while a servo moves (--sag)
  double sagHold     = 0.0;   // ... while the gripper holds an object (--saghold)
  double sensLag     = 0.0;   // ms, first-order lag of every sensor channel (--slag)

  // ---- supply model (the firmware measures its own VCC against the chip's
  // ---- 1.1 V bandgap). It only gives that reading and the brown-out check;
  // ---- it does not change the motors (that is --sag), so every run that
  // ---- does not brown out drives exactly as before. The defaults are a
  // ---- fresh 9 V PP3: VCC stays at 5.00 V all the time.
  double vbat        = 9.0;   // V, battery with no load (--vbat)
  double rint        = 1.5;   // ohm, battery internal resistance (--rint)
  double ldo         = 1.1;   // V, the 5 V regulator's dropout (--ldo)
  double iBase       = 0.12;  // A, Nano + sensor bar LEDs + idle servos (--ibase)
  double iMotor      = 0.30;  // A per motor at duty 255 while driving (--imotor)
  double iServo      = 0.60;  // A while a servo moves (--iservo)
  double iHold       = 0.15;  // A extra while the gripper holds an object (--ihold)
  double bandgap     = 1.100; // V, this chip's real bandgap; the firmware assumes 1.100 (--bandgap)
  double bod         = 2.7;   // V, VCC below this = brown-out: the Nano restarts (--bod)
  int    vccAdc      = -1;    // >=0: the bandgap reading is always this (--vccadc, e.g. 0 or 1023)
} CFG;
static bool w_supplySet = false;   // a supply flag was given: say so at the start
static bool w_phys = false;   // any of the realism effects above is on

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

// Worn tape (all empty by default, so the field is the clean one).
//   --erase=X:Y:R[:T0:T1]  a white disc of radius R at (X,Y): a gap in the tape,
//                          optionally only between sim times T0 and T1 (seconds)
//   --patch=X:Y:HW:HH      a black rectangle of half-width HW, half-height HH: a mark
//   --wipe=X:Y:HW:HH       a white rectangle: a clean cut across the tape
// A patch beats an erase or a wipe, which beat the tape. Their edges are always hard.
static double w_rowPitch = -1;   // --row=CM: TOP/BOT distance from MID (default: --cell)
struct Erase { double x, y, r, t0, t1; };
static std::vector<Erase> w_erase;
struct Patch { double x, y, hw, hh; };
static std::vector<Patch> w_patch, w_wipe;
static double simNowSec();
static bool inRect(const std::vector<Patch>& v, double px, double py) {
  for (size_t i = 0; i < v.size(); i++) {
    const Patch& q = v[i];
    if (std::fabs(px - q.x) <= q.hw && std::fabs(py - q.y) <= q.hh) return true;
  }
  return false;
}
static bool patched(double px, double py) { return inRect(w_patch, px, py); }
static bool erased(double px, double py) {
  for (size_t i = 0; i < w_erase.size(); i++) {
    const Erase& e = w_erase[i];
    if (std::hypot(px - e.x, py - e.y) <= e.r && simNowSec() >= e.t0 && simNowSec() <= e.t1) return true;
  }
  return inRect(w_wipe, px, py);
}

// Field set-up variations (all empty / off by default, so the field is the clean one).
//   --objoff=NAME:DX:DY    move object o1..o3 or target x1..x3 by (DX,DY) cm from its place
//   --gripr=CM             how close the jaws must be to take an object (default 3.5)
//   --wzone=X:Y:HW:HH:DW[:MASK]  inside this rectangle the white of the channels in MASK
//                          (hex, bit K = channel K, default ff) reads DW higher (the black
//                          stays: a darker or dirtier patch of the field)
//   --wshift=DW:T0:T1[:MASK]  the white of those channels moves by DW, linearly from sim
//                          time T0 to T1 (s), then stays: the light changing during the run
//   --bshift=DW:T0:T1[:MASK]  the same for the black level
struct ObjOff { std::string name; double dx, dy; };
static std::vector<ObjOff> w_objOff;
struct WZone { double x, y, hw, hh, dw; unsigned mask; };
static std::vector<WZone> w_wzone;
struct LShift { double dw, t0, t1; unsigned mask; };
static std::vector<LShift> w_wshift, w_bshift;
static double shiftNow(const std::vector<LShift>& v, int ch) {
  double d = 0, t = simNowSec();
  for (size_t i = 0; i < v.size(); i++) {
    const LShift& q = v[i];
    if (!(q.mask & (1u << ch)) || t < q.t0) continue;
    double f = (q.t1 > q.t0) ? (t - q.t0) / (q.t1 - q.t0) : 1.0;
    d += q.dw * (f > 1.0 ? 1.0 : f);
  }
  return d;
}
static double zoneNow(double px, double py, int ch) {
  double d = 0;
  for (size_t i = 0; i < w_wzone.size(); i++) {
    const WZone& z = w_wzone[i];
    if ((z.mask & (1u << ch)) && std::fabs(px - z.x) <= z.hw && std::fabs(py - z.y) <= z.hh) d += z.dw;
  }
  return d;
}

static void buildField() {
  const double c = CFG.cell;
  const double C1 = 0, C2 = c, C3 = 2 * c, C4 = 3 * c;
  const double r = (w_rowPitch > 0) ? w_rowPitch : c;
  const double TOP = r, MID = 0, BOT = -r;
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
  for (size_t k = 0; k < w_objOff.size(); k++) {
    const ObjOff& q = w_objOff[k];
    bool found = false;
    for (size_t i = 0; i < w_objs.size(); i++)
      if (w_objs[i].name == q.name) { w_objs[i].x += q.dx; w_objs[i].y += q.dy; found = true; }
    for (size_t i = 0; i < w_targets.size(); i++)
      if (w_targets[i].name == q.name) { w_targets[i].x += q.dx; w_targets[i].y += q.dy; found = true; }
    if (!found) fprintf(stderr, "sim: --objoff: no object or target %s\n", q.name.c_str());
  }
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
  if (patched(px, py)) return true;
  if (erased(px, py)) return false;
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
//            full black, which is what a real, slightly defocused sensor does.
static double tapeCoverage(double px, double py) {
  if (CFG.edge <= 0) return onLine(px, py) ? 1.0 : 0.0;
  if (patched(px, py)) return 1.0;
  if (erased(px, py)) return 0.0;
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
static double simNowSec() { return w_t_us / 1e6; }
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
static double wheelV(int fPin, int bPin, int pwmPin, double trim, double dbFwd, double trimRev,
                     double dbRev) {
  if (w_digital[10] == 0) return 0;          // STBY low: driver disabled
  int f = w_digital[fPin], b = w_digital[bPin];
  if (f == b) return 0;                      // both high = brake, both low = coast
  double duty = w_pwm[pwmPin];
  bool fwd = (f == 1 && b == 0);
  double db = (!fwd && dbRev >= 0) ? dbRev : dbFwd;
  if (duty <= db) return 0;
  double g = (!fwd && trimRev >= 0) ? trimRev : trim;
  double v = (duty - db) / (255.0 - db) * CFG.vMax * g;
  return fwd ? v : -v;
}

// ---------------- realistic physics (only when w_phys) ---------------------
// Each wheel is a first-order system: its speed moves toward the speed the
// duty would hold with time constant tau (taub while short-braking). The dead
// band is friction: a wheel at rest starts only above its start dead band,
// but once turning it keeps turning down to dbRun x that. A battery sag scales
// the effective duty (the friction does not sag, so slow wheels lose most).
struct WheelSt { double v = 0; bool stuck = true; };
static WheelSt w_wL, w_wR;
static double w_servoMoveUntil = -1;          // us: a servo is moving until then
static unsigned long long w_prng = 0x2545F4914F6CDD1DULL;   // own generator: sensor noise unchanged
static double prngU() {
  w_prng ^= w_prng >> 12; w_prng ^= w_prng << 25; w_prng ^= w_prng >> 27;
  return (double)((w_prng * 2685821657736338717ULL) >> 11) * (1.0 / 9007199254740992.0);
}
static double prngG() { double a = 0; for (int i = 0; i < 12; i++) a += prngU(); return a - 6.0; }
static double w_ouSpin = 0, w_ouPiv = 0;       // Ornstein-Uhlenbeck states, sd 1, 150 ms
static double w_sensC[8];                      // lagged tape coverage per channel
static bool   w_sensInit = false;
static double w_sagNow = 0;

static double sagNow() {
  if (w_t_us < w_servoMoveUntil) return CFG.sag;
  if (w_heldIdx >= 0) return CFG.sagHold;
  return 0;
}

// ---------------- supply: battery -> regulator -> VCC ----------------------
// Battery current = the Nano and sensors + each motor (in proportion to its
// duty, only while driving) + a moving servo (the ~25 ms after each step,
// the same window --sag uses) + the gripper holding an object.
// VIN = vbat - rint x current; VCC = min(5.0, VIN - ldo).
static double supplyAmps() {
  double a = CFG.iBase;
  if (w_digital[10]) {                                    // STBY: the driver is on
    if (w_digital[12] != w_digital[11]) a += CFG.iMotor * w_pwm[6] / 255.0;
    if (w_digital[7]  != w_digital[8])  a += CFG.iMotor * w_pwm[3] / 255.0;
  }
  if (w_t_us < w_servoMoveUntil) a += CFG.iServo;
  if (w_heldIdx >= 0) a += CFG.iHold;
  return a;
}
static double supplyVin() { return CFG.vbat - CFG.rint * supplyAmps(); }
static double supplyVcc() { double v = supplyVin() - CFG.ldo; return v > 5.0 ? 5.0 : (v < 0 ? 0 : v); }
static void brownOut(double vcc);

static void wheelStep(WheelSt& w, int fPin, int bPin, int pwmPin, double trim, double dbFwd,
                      double trimRev, double dbRev, double dt_s) {
  int mode, dir = 0;                           // mode 0 coast, 1 short brake, 2 drive
  if (w_digital[10] == 0) mode = 0;
  else {
    int f = w_digital[fPin], b = w_digital[bPin];
    if (f == b) mode = f ? 1 : 0;
    else { mode = 2; dir = (f == 1) ? 1 : -1; }
  }
  double duty = (mode == 2) ? w_pwm[pwmPin] * (1.0 - w_sagNow) : 0;
  if (mode == 2 && duty <= 0) mode = 1;        // PWM low all the time = short brake
  if (w.stuck) {                               // static friction
    double db0 = (dir < 0 && dbRev >= 0) ? dbRev : dbFwd;
    if (mode == 2 && duty > db0) w.stuck = false;
    else { w.v = 0; return; }
  }
  double sgnv = (w.v > 0) ? 1 : (w.v < 0 ? -1 : dir);
  double db = (sgnv < 0 && dbRev >= 0) ? dbRev : dbFwd;
  double dbR = db * CFG.dbRun;                 // running friction, in duty
  double g = (sgnv < 0 && trimRev >= 0) ? trimRev : trim;
  double k = CFG.vMax * g / (255.0 - dbR);     // cm/s per duty above friction
  double taub = CFG.taub >= 0 ? CFG.taub : 0.75 * CFG.tau;
  if (mode == 2) {
    double tgt = (dir * duty - sgnv * dbR) * k;
    if (CFG.tau <= 0) w.v = tgt;
    else w.v += (tgt - w.v) * (1.0 - std::exp(-dt_s * 1000.0 / CFG.tau));
  } else if (mode == 1) {
    double tgt = -sgnv * dbR * k;
    if (taub <= 0) w.v = 0;
    else w.v += (tgt - w.v) * (1.0 - std::exp(-dt_s * 1000.0 / taub));
  } else {                                     // coast: friction only
    if (CFG.tau <= 0) w.v = 0;
    else w.v -= sgnv * dbR * k * dt_s * 1000.0 / CFG.tau;
  }
  if (w.v * sgnv <= 0 || (mode != 2 && std::fabs(w.v) < 0.01)) { w.v = 0; w.stuck = true; }   // stopped: friction holds it
}

static void sensorPos(int idx, double& bx, double& by);

static void stepPhysicsReal(double dt_s) {
  w_sagNow = sagNow();
  wheelStep(w_wL, 12, 11, 6, CFG.trimL, CFG.dbL, CFG.trimLrev, CFG.dbLrev, dt_s);
  wheelStep(w_wR, 7, 8, 3, CFG.trimR, CFG.dbR, CFG.trimRrev, CFG.dbRrev, dt_s);
  double vL = w_wL.v, vR = w_wR.v;
  double v = (vL + vR) / 2.0;
  double omega = (vR - vL) / CFG.track;
  double vlat = 0;
  double a = std::fabs(vL) + std::fabs(vR);
  if (a > 1e-6) {
    // share of the motion that is spinning: 1 for a spin or a pivot, 0 straight
    double s = std::fabs(vR - vL) / a;
    if (s > 1) s = 1;
    if (CFG.spinJit > 0 || CFG.pivotJit > 0) {
      double tc = 0.150, e = std::sqrt(2.0 * dt_s / tc);
      w_ouSpin += -w_ouSpin * dt_s / tc + e * prngG();
      w_ouPiv  += -w_ouPiv  * dt_s / tc + e * prngG();
    }
    double loss = CFG.scrub * s * (1.0 + CFG.spinJit * w_ouSpin);
    if (loss < 0) loss = 0;
    if (loss > 0.9) loss = 0.9;
    omega *= (1.0 - loss);
    // turning about a point ahead of the axle: the axle slides sideways
    double piv = (CFG.pivot + CFG.pivotJit * w_ouPiv) * s;
    vlat = -piv * omega;
  }
  w_rob.x += (v * std::cos(w_rob.th) - vlat * std::sin(w_rob.th)) * dt_s;
  w_rob.y += (v * std::sin(w_rob.th) + vlat * std::cos(w_rob.th)) * dt_s;
  w_rob.th += omega * dt_s;
  w_rob.odo += (std::fabs(v) + std::fabs(vlat) + std::fabs(omega) * 0.01) * dt_s;
  if (w_heldIdx >= 0) gripPoint(w_objs[w_heldIdx].x, w_objs[w_heldIdx].y);
  static double lx = 1e9, ly = 1e9, lth = 1e9;
  static bool settled = false;
  if (CFG.sensLag > 0 && !(settled && w_rob.x == lx && w_rob.y == ly && w_rob.th == lth)) {
    double f = 1.0 - std::exp(-dt_s * 1000.0 / CFG.sensLag);
    settled = (w_rob.x == lx && w_rob.y == ly && w_rob.th == lth);
    for (int i = 0; i < 8; i++) {
      double bx, by; sensorPos(i, bx, by);
      double c = tapeCoverage(bx, by);
      if (!w_sensInit) w_sensC[i] = c; else w_sensC[i] += (c - w_sensC[i]) * f;
      if (std::fabs(c - w_sensC[i]) > 1e-5) settled = false;
    }
    lx = w_rob.x; ly = w_rob.y; lth = w_rob.th;
    w_sensInit = true;
  }
}

static void stepPhysics(double dt_s) {
  if (w_phys) { stepPhysicsReal(dt_s); return; }
  double vL = wheelV(12, 11, 6, CFG.trimL, CFG.dbL, CFG.trimLrev, CFG.dbLrev);
  double vR = wheelV(7, 8, 3, CFG.trimR, CFG.dbR, CFG.trimRrev, CFG.dbRrev);
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

// Moving the robot "by hand" (for the standing-still check modes):
// --at=T:X:Y:H puts it at (X,Y) heading H at time T; --sweep=T0:T1:X:Y0:Y1:H
// slides it from Y0 to Y1 between T0 and T1.
struct HandMove { double t0, t1, x, y0, y1, h; bool done; };
static HandMove w_hand[24];
static int w_nHand = 0;
static void handMoves() {
  double t = w_t_us / 1e6;
  for (int i = 0; i < w_nHand; i++) {
    HandMove& m = w_hand[i];
    if (m.done || t < m.t0) continue;
    double f = (m.t1 > m.t0) ? (t - m.t0) / (m.t1 - m.t0) : 1.0;
    if (f >= 1.0) { f = 1.0; m.done = true; }
    w_rob.x = m.x; w_rob.y = m.y0 + (m.y1 - m.y0) * f; w_rob.th = m.h * M_PI / 180.0;
  }
}

// --path (for sim/path_check.py): a PATH line at the moment the firmware's
// count changes. The COUNT line is printed only after loop() returns, so it
// never shows a count that a case in the switch moves on at once (a turn, a
// pick, a place); this is looked at on every step of the clock instead. The
// firmware's count is read through a function set up after the #include.
// Nothing is printed and nothing changes without the flag.
static int  (*w_pathCount)() = 0;              // reads numGride (0 = --path off)
static long (*w_pathExtra)() = 0;              // e33: retries x 256 + credited crossings
static int  w_pathLast = -1000;
static long w_pathLastX = -1;
static void pathWatch() {
  int n = w_pathCount();
  long x = w_pathExtra ? w_pathExtra() : 0;
  if (n == w_pathLast && x == w_pathLastX) return;
  w_pathLast = n; w_pathLastX = x;
  printf("[%7.3fs] PATH    numGride=%-3d axle=(%7.2f,%7.2f) heading=%7.2fdeg retry=%ld credit=%ld\n",
         w_t_us / 1e6, n, w_rob.x, w_rob.y, w_rob.th * 180 / M_PI, x >> 8, x & 255);
}

static double w_trT0 = -1, w_trT1 = -1, w_trNext = 0;   // --trace=T0:T1 (s)
static void advance(double us) {
  const double MAXSTEP = 1000.0;             // integrate in <=1 ms slices
  while (us > 0) {
    double chunk = us > MAXSTEP ? MAXSTEP : us;
    stepPhysics(chunk / 1e6);
    w_t_us += chunk;
    us -= chunk;
    { double vcc = supplyVcc(); if (vcc < CFG.bod) brownOut(vcc); }   // (never with the defaults)
    if (w_trT0 >= 0 && w_t_us >= w_trNext && w_t_us / 1e6 >= w_trT0 && w_t_us / 1e6 <= w_trT1) {
      w_trNext = w_t_us + 20000.0;
      printf("[%7.3fs] TRACE pose=(%6.2f,%6.2f) hdg=%7.1f vL=%6.2f vR=%6.2f pwmL=%3d%c pwmR=%3d%c\n", w_t_us / 1e6,
             w_rob.x, w_rob.y, w_rob.th * 180 / M_PI, w_wL.v, w_wR.v,
             w_pwm[6], w_digital[12] == w_digital[11] ? 'b' : (w_digital[12] ? 'f' : 'r'),
             w_pwm[3], w_digital[7] == w_digital[8] ? 'b' : (w_digital[7] ? 'f' : 'r'));
    }
  }
  if (w_nHand) handMoves();
  if (w_pathCount) pathWatch();
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

// channel 0 (A0) is the LEFT-most sensor
static void sensorPos(int idx, double& bx, double& by) {
  double off = (3.5 - idx) * CFG.sensorPitch;
  bx = w_rob.x + CFG.sensorAhead * std::cos(w_rob.th) - off * std::sin(w_rob.th);
  by = w_rob.y + CFG.sensorAhead * std::sin(w_rob.th) + off * std::cos(w_rob.th);
}

// --rawfile=FILE: real Serial Monitor printouts played back into A0..A7 while
// the robot stands still (sim/student_frames.txt, sim/replay_test.sh). Each
// frame is held for --rawhold ms, from the firmware's first analogRead; after
// the last one the run ends. Without the flag the list is empty and nothing
// changes.
struct RawFrame { int v[8]; int line; };
static std::vector<RawFrame> w_raw;
static double w_rawHoldMs = 1000.0;          // --rawhold=MS
static long   w_rawShown = -1;               // the frame last announced (verbose)
static double w_rawT0 = -1;                  // us: the firmware's first look at the bar
static int rawFrameValue(int idx) {
  if (w_rawT0 < 0) w_rawT0 = w_t_us;
  size_t k = (size_t)((w_t_us - w_rawT0) / (w_rawHoldMs * 1000.0));
  if (k >= w_raw.size()) { finishRun("rawfile: every frame played"); return 0; }
  if ((long)k != w_rawShown) {
    w_rawShown = (long)k;
    if (w_verbose) {
      const int* f = w_raw[k].v;
      logEvent("RAW     frame %d of %d (file line %d): %d %d %d %d %d %d %d %d", (int)k + 1, (int)w_raw.size(),
               w_raw[k].line, f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7]);
    }
  }
  return w_raw[k].v[idx];
}
// Reads the frames: a line with "raw" and 8 numbers after it (a waiting-screen
// or meter line as the Serial Monitor shows it), or a line of just 8 numbers.
// "[name]" starts a part; part (--rawpart) keeps only the parts named so (the
// first word of the name is enough). Anything else is skipped.
static bool loadRawFile(const char* path, const char* part) {
  FILE* f = fopen(path, "r");
  if (!f) { fprintf(stderr, "sim: cannot open --rawfile=%s\n", path); return false; }
  char buf[512], sect[128] = "";
  int ln = 0;
  while (fgets(buf, sizeof buf, f)) {
    ln++;
    char* s = buf;
    while (*s == ' ' || *s == '\t') s++;
    if (*s == '[') {
      char* e = strchr(s, ']');
      size_t n = e ? (size_t)(e - s - 1) : 0;
      if (n >= sizeof sect) n = sizeof sect - 1;
      memcpy(sect, s + 1, n); sect[n] = 0;
      continue;
    }
    if (*s == '#') continue;
    if (part && *part) {
      size_t n = strlen(part);
      if (strncmp(sect, part, n) || (sect[n] != 0 && sect[n] != ' ' && sect[n] != ':')) continue;
    }
    char* p = strstr(s, "raw ");
    p = p ? p + 4 : s;
    RawFrame fr; fr.line = ln;
    int got = 0;
    for (; got < 8; got++) {
      char* q;
      long x = strtol(p, &q, 10);
      if (q == p || x < 0 || x > 1023) break;
      fr.v[got] = (int)x; p = q;
    }
    if (got == 8 && (*p == 0 || *p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) w_raw.push_back(fr);
    else if (strstr(s, "raw ")) fprintf(stderr, "sim: --rawfile line %d skipped (not 8 readings)\n", ln);
  }
  fclose(f);
  if (w_raw.empty()) { fprintf(stderr, "sim: no frames in --rawfile=%s%s%s\n", path, part ? " part " : "", part ? part : ""); return false; }
  return true;
}

int analogRead(int pin) {
  advance(104);                              // a real analogRead costs ~104 us
  int idx = pin - A0;
  if (idx < 0 || idx > 7) return 0;
  if (!w_raw.empty()) return rawFrameValue(idx);
  double bx, by;
  sensorPos(idx, bx, by);
  // With every realism flag at its default this is exactly the original
  // "onLine ? 900 : 100": coverage is 0 or 1, 100 + 800*c, and no noise draw.
  double c = (CFG.sensLag > 0 && w_sensInit) ? w_sensC[idx] : tapeCoverage(bx, by);
  double wl = w_chWhite[idx], bl = w_chBlack[idx];
  if (!w_wzone.empty()) wl += zoneNow(bx, by, idx);
  if (!w_wshift.empty()) wl += shiftNow(w_wshift, idx);
  if (!w_bshift.empty()) bl += shiftNow(w_bshift, idx);
  double v = wl + (bl - wl) * c;
  if (CFG.noise > 0) v += CFG.noise * rngGauss();
  long r = std::lround(v);
  if (r < 0) r = 0; else if (r > 1023) r = 1023;          // the 10-bit ADC range
  return (int)r;
}

/* The ADC reading of the 1.1 V bandgap against VCC, as the firmware's
 * bandgapAdc() gets it on the Nano: round(bandgap x 1023 / VCC). It costs no
 * simulated time, so the firmware's timing stays exactly as before. */
uint16_t sim_bandgap_adc() {
  if (CFG.vccAdc >= 0) return (uint16_t)CFG.vccAdc;
  double vcc = supplyVcc();
  long a = vcc > 0.01 ? std::lround(CFG.bandgap * 1023.0 / vcc) : 1023;
  if (a > 1023) a = 1023;
  return (uint16_t)a;
}

/* VCC fell below the brown-out level: the real Nano resets here. The run
 * ends (the EEPROM is saved, as the chip keeps it); build with
 * -DSIM_RUNMARK=0xE33A55C3UL and boot that EEPROM to see what the restarted
 * firmware does. */
static void brownOut(double vcc) {
  logEvent("BROWN-OUT VCC %.2f V (battery %.2f V at %.2f A): the Nano restarts here",
           vcc, supplyVin(), supplyAmps());
  finishRun("brown-out");
}

/* debugging hook for instrumented firmware copies (unused by the real firmware) */
void simDbg(const char* w, long a, long b, long c) {
  printf("[%7.3fs] DBG %s %ld %ld %ld pose=(%.2f,%.2f) hdg=%.1f\n", w_t_us / 1e6, w, a, b, c,
         w_rob.x, w_rob.y, w_rob.th * 180 / M_PI);
}

void sim_servo_write(int pin, int angle) {
  /* a servo that changes angle draws current for about one 20 ms frame */
  if (pin >= 0 && pin < 24 && w_servoAngle[pin] != angle) w_servoMoveUntil = w_t_us + 25000.0;
  if (pin >= 0 && pin < 24) w_servoAngle[pin] = angle;
  if (pin != 5) return;                      // only the gripper servo matters here
  /* Assumes SERVO_GRIP_CLOSED < 80 < SERVO_GRIP_OPEN. config.h is not visible
   * this early in the file, so this stays a literal: check it if you retune
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

// --path: how the sim reads the firmware's count (see pathWatch)
static int simPathCount() { return (int)numGride; }
#ifdef N_DONE                                  /* e33_mission: its retry and credit counters */
static long simPathExtra() { return (long)nRetried * 256 + nCredited; }
#else
static long simPathExtra() { return 0; }
#endif
#ifdef ROUTE_ORDER
static const int w_pathRoute = ROUTE_ORDER;
#else
static const int w_pathRoute = 0;
#endif

// ----------------------------- runner ---------------------------------------
static void usage() {
  printf(
    "usage: sim [flags]\n"
    "  run / output\n"
    "    --quiet            only the RESULT block\n"
    "    --serial           echo what the firmware prints on Serial\n"
    "    --path             a PATH line at every change of the count, when it happens (sim/path_check.py)\n"
    "    --secs=S           simulated-time cap (default 240)\n"
    "  field / geometry\n"
    "    --cell=CM          grid pitch (default 25)\n"
    "    --row=CM           row pitch, MID to TOP and to BOT (default: --cell)\n"
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
    "  worn tape (repeatable; X, Y in cm, C1 on MID = (0,0), x east, y north)\n"
    "    --erase=X:Y:R[:T0:T1]  white disc of radius R: a gap in the tape\n"
    "                       (only between sim times T0 and T1 s when given)\n"
    "    --patch=X:Y:HW:HH  black rectangle, half-width HW, half-height HH: a mark\n"
    "    --wipe=X:Y:HW:HH   white rectangle: a clean cut across the tape\n"
    "    --trimLr=G --trimRr=G  motor gain driving BACKWARD (default: same as forward)\n"
    "    --dbLr=N --dbRr=N  motor dead band driving BACKWARD (default: same as forward)\n"
    "    --track=CM         distance between the wheels (default 11)\n"
    "    --startx=CM --starty=CM --starth=DEG   axle position and heading (default -5 0 0;\n"
    "                       the e33 sketches: the axle over C1, --startx=0)\n"
    "    --eeprom=FILE      load EEPROM from FILE at the start, save it back at the end\n"
    "    --still=S          end a run whose firmware has idled this long (default 10)\n"
    "    --at=T:X:Y:H       move the robot by hand at time T (for the check modes)\n"
    "    --sweep=T0:T1:X:Y0:Y1:H  slide it by hand from Y0 to Y1 between T0 and T1\n"
    "  real printouts (sim/student_frames.txt, sim/replay_test.sh)\n"
    "    --rawfile=FILE     play the \"raw\" readings of Serial Monitor lines into A0..A7, the robot\n"
    "                       standing still; the run ends after the last frame\n"
    "    --rawpart=NAME     only the frames under [NAME ...] in that file\n"
    "    --rawhold=MS       how long each frame lasts (default 1000, the printouts' 1 s)\n"
    "  field set-up (repeatable; all off by default; X, Y as for worn tape)\n"
    "    --objoff=NAME:DX:DY  move object o1..o3 or target x1..x3 by DX, DY cm from its place\n"
    "    --gripr=CM         the jaws take an object within this distance of them (default 3.5)\n"
    "    --wzone=X:Y:HW:HH:DW[:MASK]  the white reads DW higher inside this rectangle (a darker\n"
    "                       patch; the black level stays). MASK: the channels, hex, bit K =\n"
    "                       channel K (default ff = all eight, 0f = A0..A3)\n"
    "    --wshift=DW:T0:T1[:MASK]  the white moves by DW, linearly from sim time T0 to T1 s, then\n"
    "                       stays (T0 = T1: a step): the light changing during the run\n"
    "    --bshift=DW:T0:T1[:MASK]  the same for the black level\n"
    "  physical realism (all off by default; --real = the values in brackets, flags after it override)\n"
    "    --tau=MS           wheel speed lag, first order, while driven [100]\n"
    "    --taub=MS          the same while short-braking (default 0.75 x tau) [75]\n"
    "    --dbrun=F          a turning wheel keeps turning down to F x its start dead band [0.65]\n"
    "    --scrub=F          share of the turn rate lost in a spin or pivot (tyre scrub, caster) [0.20]\n"
    "    --spinjit=F        unevenness of that loss, sd relative, 150 ms correlation [0.15]\n"
    "    --pivot=CM         spins turn about a point this far ahead of the axle [0.8]\n"
    "    --pivotjit=CM      the turning centre wanders by this sd [0.3]\n"
    "    --sag=F            effective duty lost while a servo moves (battery sag) [0.10]\n"
    "    --saghold=F        ... while the gripper holds an object [0.04]\n"
    "    --slag=MS          sensor response lag, first order [1.5]\n"
    "    --pseed=S          seed of the physics jitter (default fixed)\n"
    "  supply (the firmware's VCC measurement; not the motors: that is --sag)\n"
    "    VIN = vbat - rint x (ibase + imotor x duty/255 per driving motor + iservo while a servo\n"
    "    moves + ihold while holding an object); VCC = min(5.0, VIN - ldo); VCC < bod = brown-out\n"
    "    (the run ends there, EEPROM saved). Defaults: a fresh PP3 (VCC stays 5.00 V).\n"
    "    --vbat=V           battery with no load (default 9.0) [8.2: a half-used PP3]\n"
    "    --rint=OHM         battery internal resistance (default 1.5) [2.5]\n"
    "    --ldo=V            regulator dropout (default 1.1)\n"
    "    --ibase=A --imotor=A --iservo=A --ihold=A   currents (default 0.12 0.30 0.60 0.15)\n"
    "    --bandgap=V        this chip's real bandgap, the firmware assumes 1.100 (default 1.100)\n"
    "    --bod=V            brown-out level (default 2.7)\n"
    "    --vccadc=N         the bandgap reading is always N (0 or 1023: an emulator without it)\n"
    "  the student's robot (TT gear motors, TB6612, 9 V PP3, their measured bar; flags after it override)\n"
    "    --student          best estimate: vmax 60, left gain 0.80, dead bands L 58/63 R 36/36 (fwd/back),\n"
    "                       left 0.95 x weaker backward, tau 140/90 ms, dbrun 0.65, scrub 0.22 jit 0.15,\n"
    "                       pivot 0.8 jit 0.3, sag 0.15 hold 0.05, slag 1.5, PP3 8.6 V 2.5 ohm, ibase 0.18,\n"
    "                       the student's 8 white/black levels (as --chan), edge 1.0, noise 10\n"
    "    --student-slow     the slow end: a tired PP3 (8.0 V 3.5 ohm), vmax 45, dead bands L 68/73 R 43,\n"
    "                       tau 190, sag 0.25 hold 0.08 (the rest as --student)\n"
    "    --student-fast     the fast end: a fresh PP3 (9.4 V 1.7 ohm), vmax 75, dead bands L 51/56 R 31,\n"
    "                       tau 110, sag 0.10 hold 0.03 (the rest as --student)\n"
    "                       After a preset: --trimL/--trim keep the backward ratio, --dbL/--dbR keep the\n"
    "                       backward offset, --db sets all four dead bands, --white/--black replace its\n"
    "                       channel levels, --weakch weakens one of them, --chan sets one channel.\n");
}

static bool flagVal(const char* arg, const char* name, const char** val) {
  size_t n = strlen(name);
  if (strncmp(arg, name, n)) return false;
  *val = arg + n;
  return true;
}

static const char* w_eepromFile = 0;
static const char* w_rawPath = 0, *w_rawPart = 0;   // --rawfile, --rawpart
static double w_startX = -5.0, w_startY = 0.0, w_startH = 0.0;

// ---------------- --student presets: the student's robot --------------------
// Values and the reasoning behind each are in studentsim/NOTES.md. In short:
// yellow TT gear motors 1:48 (about 200 rpm no load at 6 V) on 65 mm wheels,
// a TB6612 driven straight from a 9 V PP3 that sags under load (about 7.6 V
// at the motors while cruising), the LEFT motor weaker (the student saw
// 270 deg against 360 deg; "speed 50 did not move your left wheel"),
// SG90-class servos on the Nano's 5 V, and the student's measured bar.
// The three presets differ in the battery (and so in speed, dead bands, lag
// and sag); the mechanics and the sensors are the same in all three.
struct StudentPreset {
  const char* name;
  double vmax, dbL, dbR, tau, sag, sagHold, vbat, rint;
};
static const StudentPreset w_students[3] = {
  {"student",      60, 58, 36, 140, 0.15, 0.05, 8.6, 2.5},   // best estimate
  {"student-slow", 45, 68, 43, 190, 0.25, 0.08, 8.0, 3.5},   // tired PP3, slower motors
  {"student-fast", 75, 51, 31, 110, 0.10, 0.03, 9.4, 1.7},   // fresh PP3, faster motors
};
// the student's own levels (Serial printouts: black 964-976, white 72-203)
static const double w_studentW[8] = {203, 158, 74, 77, 72, 96, 99, 99};
static const double w_studentB[8] = {970, 971, 965, 966, 966, 975, 971, 971};
static int    w_preset = -1;                 // which preset was given last (-1 none)
static bool   w_chPreset[8];                 // channel levels set by the preset
static bool   w_psDb = false;                // backward dead bands follow the forward ones
static double w_psDbOffL = 0, w_psDbOffR = 0;  // ... by this much
static double w_psRevL = -1, w_psRevR = -1;    // backward gain = this x forward gain

static void applyStudent(int k) {
  const StudentPreset& p = w_students[k];
  w_preset = k;
  CFG.vMax = p.vmax;
  CFG.trimL = 0.80; CFG.trimR = 1.00;          // slope above the dead band (0.75 was seen at duty 120)
  CFG.trimLrev = -1; CFG.trimRrev = -1;        // resolved after the flags: 0.95 x / 1.00 x forward
  w_psRevL = 0.95; w_psRevR = 1.00;
  CFG.dbL = p.dbL; CFG.dbR = p.dbR;
  CFG.dbLrev = -1; CFG.dbRrev = -1;            // resolved after the flags: forward + 5 / + 0
  w_psDb = true; w_psDbOffL = 5; w_psDbOffR = 0;
  CFG.tau = p.tau; CFG.taub = 90; CFG.dbRun = 0.65;
  CFG.scrub = 0.22; CFG.spinJit = 0.15; CFG.pivot = 0.8; CFG.pivotJit = 0.3;
  CFG.sag = p.sag; CFG.sagHold = p.sagHold; CFG.sensLag = 1.5;
  CFG.vbat = p.vbat; CFG.rint = p.rint; CFG.iBase = 0.18;
  w_supplySet = true;
  for (int c = 0; c < 8; c++) {
    w_chSet[c] = true; w_chPreset[c] = true;
    w_chSetW[c] = w_studentW[c]; w_chSetB[c] = w_studentB[c];
  }
  CFG.edge = 1.0; CFG.noise = 10;
}

int main(int argc, char** argv) {
  double& maxSimSec = w_maxSimSec;
  for (int i = 1; i < argc; i++) {
    const char* v = 0;
    if (!strcmp(argv[i], "--quiet")) w_verbose = false;
    else if (!strcmp(argv[i], "--serial")) Serial.echo = true;  /* show what the firmware prints */
    else if (!strcmp(argv[i], "--path")) w_pathCount = simPathCount;   /* PATH lines (pathWatch) */
    else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) { usage(); return 0; }
    else if (!strncmp(argv[i], "--trim=", 7)) CFG.trimR = atof(argv[i] + 7);
    else if (!strncmp(argv[i], "--vmax=", 7)) CFG.vMax = atof(argv[i] + 7);
    else if (!strncmp(argv[i], "--cell=", 7)) CFG.cell = atof(argv[i] + 7);
    else if (!strncmp(argv[i], "--row=", 6)) w_rowPitch = atof(argv[i] + 6);
    else if (!strncmp(argv[i], "--grip=", 7)) CFG.gripReach = atof(argv[i] + 7);
    else if (!strncmp(argv[i], "--secs=", 7)) maxSimSec = atof(argv[i] + 7);
    /* ---- real-robot effects, all default OFF ---- */
    else if (flagVal(argv[i], "--trimL=", &v))    CFG.trimL = atof(v);
    else if (flagVal(argv[i], "--db=", &v)) {
      CFG.deadband = atof(v);
      if (w_psDb) { CFG.dbL = CFG.dbR = CFG.dbLrev = CFG.dbRrev = -1; w_psDb = false; }   /* after a preset: all four */
    }
    else if (flagVal(argv[i], "--dbLr=", &v))     CFG.dbLrev = atof(v);
    else if (flagVal(argv[i], "--dbRr=", &v))     CFG.dbRrev = atof(v);
    else if (flagVal(argv[i], "--track=", &v))    CFG.track = atof(v);
    else if (!strcmp(argv[i], "--student"))       applyStudent(0);
    else if (!strcmp(argv[i], "--student-slow"))  applyStudent(1);
    else if (!strcmp(argv[i], "--student-fast"))  applyStudent(2);
    else if (flagVal(argv[i], "--dbL=", &v))      CFG.dbL = atof(v);
    else if (flagVal(argv[i], "--dbR=", &v))      CFG.dbR = atof(v);
    else if (flagVal(argv[i], "--white=", &v) || flagVal(argv[i], "--black=", &v)) {
      (argv[i][2] == 'w' ? CFG.white : CFG.black) = atof(v);
      for (int k = 0; k < 8; k++)              /* after a preset: these replace its levels */
        if (w_chPreset[k]) { w_chPreset[k] = false; w_chSet[k] = false; }
    }
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
    else if (flagVal(argv[i], "--rawfile=", &v))  w_rawPath = v;
    else if (flagVal(argv[i], "--rawpart=", &v))  w_rawPart = v;
    else if (flagVal(argv[i], "--rawhold=", &v))  w_rawHoldMs = atof(v) > 0 ? atof(v) : 1000.0;
    else if (flagVal(argv[i], "--tau=", &v))      CFG.tau = atof(v);
    else if (flagVal(argv[i], "--trace=", &v))    sscanf(v, "%lf:%lf", &w_trT0, &w_trT1);
    else if (flagVal(argv[i], "--taub=", &v))     CFG.taub = atof(v);
    else if (flagVal(argv[i], "--dbrun=", &v))    CFG.dbRun = atof(v);
    else if (flagVal(argv[i], "--scrub=", &v))    CFG.scrub = atof(v);
    else if (flagVal(argv[i], "--spinjit=", &v))  CFG.spinJit = atof(v);
    else if (flagVal(argv[i], "--pivot=", &v))    CFG.pivot = atof(v);
    else if (flagVal(argv[i], "--pivotjit=", &v)) CFG.pivotJit = atof(v);
    else if (flagVal(argv[i], "--sag=", &v))      CFG.sag = atof(v);
    else if (flagVal(argv[i], "--saghold=", &v))  CFG.sagHold = atof(v);
    else if (flagVal(argv[i], "--slag=", &v))     CFG.sensLag = atof(v);
    else if (flagVal(argv[i], "--erase=", &v)) {
      Erase e; e.t0 = 0; e.t1 = 1e9;
      if (sscanf(v, "%lf:%lf:%lf:%lf:%lf", &e.x, &e.y, &e.r, &e.t0, &e.t1) >= 3) w_erase.push_back(e);
      else fprintf(stderr, "sim: bad --erase=%s (X:Y:R[:T0:T1])\n", v);
    }
    else if (flagVal(argv[i], "--patch=", &v) || flagVal(argv[i], "--wipe=", &v)) {
      Patch q;
      bool white = argv[i][2] == 'w';
      if (sscanf(v, "%lf:%lf:%lf:%lf", &q.x, &q.y, &q.hw, &q.hh) == 4) (white ? w_wipe : w_patch).push_back(q);
      else fprintf(stderr, "sim: bad %s (X:Y:HW:HH)\n", argv[i]);
    }
    else if (flagVal(argv[i], "--objoff=", &v)) {
      char nm[8]; ObjOff q;
      if (sscanf(v, "%7[^:]:%lf:%lf", nm, &q.dx, &q.dy) == 3) { q.name = nm; w_objOff.push_back(q); }
      else { fprintf(stderr, "sim: bad --objoff=%s (NAME:DX:DY)\n", v); return 2; }
    }
    else if (flagVal(argv[i], "--gripr=", &v))    CFG.gripRadius = atof(v);
    else if (flagVal(argv[i], "--wzone=", &v)) {
      WZone z; z.mask = 0xff;
      if (sscanf(v, "%lf:%lf:%lf:%lf:%lf:%x", &z.x, &z.y, &z.hw, &z.hh, &z.dw, &z.mask) >= 5) w_wzone.push_back(z);
      else { fprintf(stderr, "sim: bad --wzone=%s (X:Y:HW:HH:DW[:MASK])\n", v); return 2; }
    }
    else if (flagVal(argv[i], "--wshift=", &v) || flagVal(argv[i], "--bshift=", &v)) {
      LShift q; q.mask = 0xff;
      bool wh = argv[i][2] == 'w';
      if (sscanf(v, "%lf:%lf:%lf:%x", &q.dw, &q.t0, &q.t1, &q.mask) >= 3) (wh ? w_wshift : w_bshift).push_back(q);
      else { fprintf(stderr, "sim: bad %s (DW:T0:T1[:MASK])\n", argv[i]); return 2; }
    }
    else if (flagVal(argv[i], "--pseed=", &v))    w_prng = strtoull(v, 0, 10) * 0x9E3779B97F4A7C15ULL + 1;
    else if (flagVal(argv[i], "--vbat=", &v))     { CFG.vbat = atof(v); w_supplySet = true; }
    else if (flagVal(argv[i], "--rint=", &v))     { CFG.rint = atof(v); w_supplySet = true; }
    else if (flagVal(argv[i], "--ldo=", &v))      { CFG.ldo = atof(v); w_supplySet = true; }
    else if (flagVal(argv[i], "--ibase=", &v))    { CFG.iBase = atof(v); w_supplySet = true; }
    else if (flagVal(argv[i], "--imotor=", &v))   { CFG.iMotor = atof(v); w_supplySet = true; }
    else if (flagVal(argv[i], "--iservo=", &v))   { CFG.iServo = atof(v); w_supplySet = true; }
    else if (flagVal(argv[i], "--ihold=", &v))    { CFG.iHold = atof(v); w_supplySet = true; }
    else if (flagVal(argv[i], "--bandgap=", &v))  { CFG.bandgap = atof(v); w_supplySet = true; }
    else if (flagVal(argv[i], "--bod=", &v))      { CFG.bod = atof(v); w_supplySet = true; }
    else if (flagVal(argv[i], "--vccadc=", &v))   { CFG.vccAdc = atoi(v); w_supplySet = true; }
    else if (!strcmp(argv[i], "--real")) {
      /* plausible values for a small TT-gearmotor robot on a 9 V PP3 (see usage) */
      CFG.tau = 100; CFG.taub = 75; CFG.dbRun = 0.65; CFG.scrub = 0.20; CFG.spinJit = 0.15;
      CFG.pivot = 0.8; CFG.pivotJit = 0.3; CFG.sag = 0.10; CFG.sagHold = 0.04; CFG.sensLag = 1.5;
      CFG.vbat = 8.2; CFG.rint = 2.5;      /* a half-used PP3: VCC dips a little while a servo moves */
    }
    else if (flagVal(argv[i], "--at=", &v) && w_nHand < 24) {
      HandMove& m = w_hand[w_nHand++];
      if (sscanf(v, "%lf:%lf:%lf:%lf", &m.t0, &m.x, &m.y0, &m.h) != 4) { fprintf(stderr, "sim: --at wants T:X:Y:H\n"); return 2; }
      m.t1 = m.t0; m.y1 = m.y0; m.done = false;
    }
    else if (flagVal(argv[i], "--sweep=", &v) && w_nHand < 24) {
      HandMove& m = w_hand[w_nHand++];
      if (sscanf(v, "%lf:%lf:%lf:%lf:%lf:%lf", &m.t0, &m.t1, &m.x, &m.y0, &m.y1, &m.h) != 6) { fprintf(stderr, "sim: --sweep wants T0:T1:X:Y0:Y1:H\n"); return 2; }
      m.done = false;
    }
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
      w_chSet[k] = true; w_chSetW[k] = cw; w_chSetB[k] = cb; w_chPreset[k] = false;
    }
    else fprintf(stderr, "sim: ignoring unknown flag %s (see --help)\n", argv[i]);
  }

  // Resolve per-motor dead bands and per-channel sensor levels.
  if (CFG.dbL < 0) CFG.dbL = CFG.deadband;
  if (CFG.dbR < 0) CFG.dbR = CFG.deadband;
  if (w_psDb) {                                /* a preset: backward follows forward */
    if (CFG.dbLrev < 0) CFG.dbLrev = CFG.dbL + w_psDbOffL;
    if (CFG.dbRrev < 0) CFG.dbRrev = CFG.dbR + w_psDbOffR;
  }
  if (w_psRevL > 0 && CFG.trimLrev < 0) CFG.trimLrev = w_psRevL * CFG.trimL;
  if (w_psRevR > 0 && CFG.trimRrev < 0) CFG.trimRrev = w_psRevR * CFG.trimR;
  for (int k = 0; k < 8; k++) {
    w_chWhite[k] = CFG.white;
    w_chBlack[k] = CFG.black;
    if (CFG.weakMask & (1u << k))
      w_chBlack[k] = w_chWhite[k] + (w_chBlack[k] - w_chWhite[k]) * CFG.weakGain;
    if (w_chSet[k]) { w_chWhite[k] = w_chSetW[k]; w_chBlack[k] = w_chSetB[k]; }
    if (w_chPreset[k] && (CFG.weakMask & (1u << k)))   /* --weakch with a preset: weaken its level */
      w_chBlack[k] = w_chWhite[k] + (w_chBlack[k] - w_chWhite[k]) * CFG.weakGain;
  }
  w_rng = CFG.seed ? (unsigned long long)CFG.seed : 0x9E3779B97F4A7C15ULL;  /* xorshift must not start at 0 */
  if (w_rawPath && !loadRawFile(w_rawPath, w_rawPart)) return 2;

  for (int i = 0; i < 24; i++) { w_digital[i] = 0; w_pwm[i] = 0; w_servoAngle[i] = 90; }
  buildField();

  // start pose: on the MID line at C1, facing east, bar just past the C1 crossing
  w_rob.x = w_startX; w_rob.y = w_startY; w_rob.th = w_startH * M_PI / 180.0;
  if (w_eepromFile) EEPROM.load(w_eepromFile);
  logEvent("START   pose=(%.1f, %.1f) heading=%.0fdeg  sensorAhead=%.1fcm cell=%.0fcm",
           w_rob.x, w_rob.y, w_rob.th * 180 / M_PI, CFG.sensorAhead, CFG.cell);

  if (w_preset >= 0)
    logEvent("PRESET  --%s (the student's robot: TT motors, TB6612, 9 V PP3; flags after it override)",
             w_students[w_preset].name);
  // Only when a realism effect is on, say so: default output stays byte-identical.
  bool motorFx = CFG.trimL != 1.0 || CFG.dbL != 25.0 || CFG.dbR != 25.0 ||
                 CFG.trimLrev >= 0 || CFG.trimRrev >= 0 || CFG.dbLrev >= 0 || CFG.dbRrev >= 0;
  bool sensFx = false;
  for (int k = 0; k < 8; k++) if (w_chWhite[k] != 100.0 || w_chBlack[k] != 900.0) sensFx = true;
  if (CFG.edge > 0 || CFG.noise > 0) sensFx = true;
  if (motorFx)
    logEvent("MOTORS  left gain %.2f dead band %.0f | right gain %.2f dead band %.0f | vmax %.0f cm/s",
             CFG.trimL, CFG.dbL, CFG.trimR, CFG.dbR, CFG.vMax);
  if (CFG.dbLrev >= 0 || CFG.dbRrev >= 0 || w_preset >= 0)   /* (only with --dbLr/--dbRr or a preset) */
    logEvent("MOTORS  backward: left gain %.2f dead band %.0f | right gain %.2f dead band %.0f",
             CFG.trimLrev >= 0 ? CFG.trimLrev : CFG.trimL, CFG.dbLrev >= 0 ? CFG.dbLrev : CFG.dbL,
             CFG.trimRrev >= 0 ? CFG.trimRrev : CFG.trimR, CFG.dbRrev >= 0 ? CFG.dbRrev : CFG.dbR);
  if (CFG.track != 11.0)
    logEvent("GEOMETRY track %.1f cm", CFG.track);
  if (sensFx) {
    char buf[200]; int n = 0;
    for (int k = 0; k < 8; k++)
      n += snprintf(buf + n, sizeof buf - n, " %d/%d", (int)std::lround(w_chWhite[k]), (int)std::lround(w_chBlack[k]));
    logEvent("SENSORS white/black A0..A7:%s  edge %.2f cm  noise sd %.0f  seed %lu",
             buf, CFG.edge, CFG.noise, CFG.seed);
  }

  w_phys = CFG.tau > 0 || CFG.dbRun != 1.0 || CFG.scrub > 0 || CFG.pivot != 0 ||
           CFG.pivotJit > 0 || CFG.sag > 0 || CFG.sagHold > 0 || CFG.sensLag > 0;
  if (w_phys)
    logEvent("PHYSICS tau %.0f/%.0f ms  dbrun %.2f  scrub %.2f jit %.2f  pivot %.1f jit %.1f cm  sag %.2f hold %.2f  sensor lag %.1f ms",
             CFG.tau, CFG.taub >= 0 ? CFG.taub : 0.75 * CFG.tau, CFG.dbRun, CFG.scrub, CFG.spinJit,
             CFG.pivot, CFG.pivotJit, CFG.sag, CFG.sagHold, CFG.sensLag);

  // (only when a supply flag was given: --real alone keeps its output as before)
  if (w_supplySet)
    logEvent("SUPPLY  battery %.2f V %.2f ohm, dropout %.2f V; A: base %.2f motor %.2f servo %.2f hold %.2f; bandgap %.3f V, brown-out %.2f V%s",
             CFG.vbat, CFG.rint, CFG.ldo, CFG.iBase, CFG.iMotor, CFG.iServo, CFG.iHold, CFG.bandgap, CFG.bod,
             CFG.vccAdc >= 0 ? " (bandgap reading forced)" : "");

  if (!w_objOff.empty() || CFG.gripRadius != 3.5 || !w_wzone.empty() || !w_wshift.empty() || !w_bshift.empty()) {
    logEvent("FIELDVAR grip radius %.2f cm, %d moved, %d white zones, %d white shifts, %d black shifts",
             CFG.gripRadius, (int)w_objOff.size(), (int)w_wzone.size(), (int)w_wshift.size(), (int)w_bshift.size());
    for (size_t i = 0; i < w_objs.size(); i++)
      logEvent("FIELDVAR %s at (%.2f, %.2f)", w_objs[i].name.c_str(), w_objs[i].x, w_objs[i].y);
    for (size_t i = 0; i < w_targets.size(); i++)
      logEvent("FIELDVAR %s at (%.2f, %.2f)", w_targets[i].name.c_str(), w_targets[i].x, w_targets[i].y);
  }
  /* --path: the geometry sim/path_check.py needs to know where every count must happen */
  if (w_pathCount) {
    w_pathExtra = simPathExtra;
    logEvent("PATHCFG route=%d cell=%.2f row=%.2f sensorAhead=%.2f grip=%.2f objBeyond=%.2f overhang=%.2f gripRadius=%.2f",
             w_pathRoute, CFG.cell, (w_rowPitch > 0) ? w_rowPitch : CFG.cell, CFG.sensorAhead, CFG.gripReach,
             CFG.objBeyond, CFG.overhang, CFG.gripRadius);
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
  if (w_pathCount) pathWatch();               // a last change of the count, if any
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
