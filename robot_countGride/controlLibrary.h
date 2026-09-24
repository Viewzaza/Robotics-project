/* controlLibrary.h
 *
 * The API from the course slides (robot07 - robot11), re-implemented on top of
 * hal.h. Every name the slides use still exists and still means the same thing;
 * what changed is that each one is now calibrated, bounded in time, and aware
 * that the sensor bar sits 9.5 cm ahead of the point the robot pivots about.
 */
#ifndef CONTROLLIBRARY_H
#define CONTROLLIBRARY_H

#include "config.h"
#include "hal.h"

/* Kept because the slides talk about them. sp is now the ramped cruise duty. */
int sp = DUTY_START;
int maxSp = DUTY_CRUISE;
unsigned long tUpSp = 0;
String detectLine = "00000000";

/* Junction bookkeeping. g_odoAtJunction is the last true position fix the
 * robot has: crossing a line is the only moment it knows where it really is. */
static float   g_odoAtJunction = 0;
static uint8_t g_lastBranches = 0;
static uint8_t g_junctionMask = 0;
static bool    g_inJunction = false;

/* Half the width of a crossing line, expressed in ODOMETRY cm -- see
 * countGrid(). Seeded with the nominal tape half-width and then re-measured
 * every time the robot drives straight through a junction. */
static float   g_crossHalfCm = LINE_HALF_W_CM;

static float cmSinceJunction() { return g_odoCm - g_odoAtJunction; }

/* How far the SENSOR BAR is past the CENTRE of the crossing line.
 *
 * cmSinceJunction() is measured from the moment the junction was declared, and
 * that moment is not the centre of the tape: the bar declares a junction as
 * soon as its leading edge reaches the tape, which is half a tape-width BEFORE
 * the centre. Every distance the mission works out from a junction -- the 9.5 cm
 * advance before a pivot, and the advance that puts the jaws on the object --
 * was therefore biased short by that half-width. Measured on the simulator the
 * bias is a flat 0.9 cm on every single manoeuvre of the run.
 *
 * g_crossHalfCm is carried in odometry cm rather than real cm on purpose. If
 * the speed calibration is wrong by a factor k, odometry cm are wrong by the
 * same k, so a half-width measured in odometry cm cancels the error exactly and
 * the correction stays right on a flat battery. */
static float cmPastJunction() { return cmSinceJunction() - g_crossHalfCm; }

/* ================================================================== startup */

void beginFnc() {
  pinMode(sp_L, OUTPUT); pinMode(F_L, OUTPUT); pinMode(B_L, OUTPUT);
  pinMode(sp_R, OUTPUT); pinMode(F_R, OUTPUT); pinMode(B_R, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  fastIoInit();                       /* 8x faster ADC, matched PWM rates */
  digitalWrite(STBY, 0);              /* driver disabled until we are ready */
  brake();

  calLoad();                          /* falls back to defaults if unset */

  servo_x.attach(x_pin);
  servo_y.attach(y_pin);
  g_servoXNow = SERVO_GRIP_OPEN;  servo_x.write(g_servoXNow);
  g_servoYNow = SERVO_ARM_DOWN;   servo_y.write(g_servoYNow);
  delay(300);

  g_odoLastUs = micros();
  g_odoCm = 0;
  g_odoAtJunction = 0;
  readMask();

  digitalWrite(STBY, 1);
  clearPid();
  sp = DUTY_START;
  tUpSp = millis();
}

/* ============================================================ line following */

void stopRobot() {
  brake();
  sp = DUTY_START;
}

void upSpeed() {
  if ((unsigned long)(millis() - tUpSp) >= 10) {
    sp += RAMP_STEP;
    tUpSp = millis();
  }
  if (sp > maxSp) sp = maxSp;
}

void moveFor() { driveStraight(sp); }

/* Compatibility: the slides' 16-way pattern table. The follower below uses the
 * continuous centroid instead, but this is kept so the mapping is still legible
 * and so anything written against the slides still links. */
int getErrorInput(String L) {
  const char *p = L.c_str();
  int sum = 0, n = 0;
  for (int i = 0; i < 8; i++)
    if (p[i] == '1') { sum += (i - 3) * 2 + 1; n++; }
  if (n == 0) return 100;                 /* 100 = line lost, as in the slides */
  return sum / n;
}

/* One step of line following.
 *  - continuous centroid error rather than 15 discrete buckets
 *  - holds a straight course while crossing a junction, where the centroid is
 *    meaningless because every sensor is lit
 *  - decelerates as it approaches the junction it expects next, instead of
 *    arriving at full cruise
 *  - actively searches when the line is lost, instead of leaving the motors at
 *    their last command and driving away
 */
void followLine() {
  uint8_t m = readMask();

  if (!m) {                                   /* line lost */
    if (!searchLine()) { stopRobot(); return; }
    clearPid();
    m = g_mask;
  }

  float err;
  if (bitCount(m) >= JUNCTION_MIN_BITS) {
    err = 0.0f;                               /* crossing a line: go straight */
  } else {
    err = (float)g_pos * 7.0f / 1000.0f;      /* -7 .. +7, same scale as slides */
  }

  float pidOut = pidFNC(err, 0, PID_KP, PID_KI, PID_KD);

  /* Anticipatory deceleration. Between junctions the robot dead-reckons how far
   * it has come; when it is within APPROACH_CM of where the next line should
   * be, it slows so the junction is counted cleanly and the following turn
   * starts from a known, low speed. */
  float togo = CELL_CM - cmSinceJunction();
  if (togo < APPROACH_CM) {
    if (sp > DUTY_APPROACH) sp = DUTY_APPROACH;
    tUpSp = millis();
  } else {
    upSpeed();
  }

  int16_t spOut  = (int16_t)(pidOut * sp / 7.0f);
  int16_t speedL = (int16_t)(sp - spOut);
  int16_t speedR = (int16_t)(sp + spOut);
  if (speedL > sp) speedL = sp;
  if (speedL < 0)  speedL = 0;
  if (speedR > sp) speedR = sp;
  if (speedR < 0)  speedR = 0;
  motors(speedL, speedR);

  detectLine = maskToString(m);
}

/* ============================================================ grid counting */

bool checkGrid() { return isJunction(g_mask); }

/* Counts a crossing line once.
 *
 * The original did `while(checkGrid());` after incrementing -- an unbounded
 * wait that can only end if the robot keeps moving, so calling it after
 * stopRobot() (which BRAKES) froze the robot on the field forever. Here the
 * re-arm is by distance travelled instead: a junction cannot be counted again
 * until the robot has moved JUNCTION_REARM_CM past it. Nothing blocks. */
int countGrid(int n) {
  bool now = isJunction(g_mask);

  if (now && !g_inJunction) {
    uint32_t t0 = millis();
    while ((uint32_t)(millis() - t0) < JUNCTION_DEBOUNCE_MS) {
      if (!isJunction(readMask())) return n;      /* a glitch, not a junction */
    }
    if (cmSinceJunction() < JUNCTION_REARM_CM) return n;

    g_inJunction   = true;
    g_junctionMask = g_mask;
    g_lastBranches = branchesOf(g_mask);
    g_odoAtJunction = g_odoCm;                    /* position fix */
    return n + 1;
  }
  if (!now) {
    /* Falling edge: the bar has just cleared the tape. The robot entered the
     * crossing at g_odoAtJunction and leaves it here, so half that span is how
     * far the junction fix sits BEFORE the centre of the line. Junctions the
     * route drives straight through -- there are ten of them before the last
     * pick -- measure it for free, with no extra motion and no new hardware.
     * Junctions the robot stops on are skipped, because braking mid-crossing
     * would report a half-width that is far too small. */
    if (g_inJunction) {
      float w = cmSinceJunction();
      if (w > 0.2f && w < 4.0f)
        g_crossHalfCm += 0.5f * (0.5f * w - g_crossHalfCm);   /* slow filter */
    }
    g_inJunction = false;
  }
  return n;
}

/* ================================================================== turning */

/* The slide-level turn names. Each now pivots about the wheel axle and ends
 * centred on the line it finds, with a timeout instead of while(true). */
void turnRight90()  { pivotDeg(+1, 90); }
void turnLeft90()   { pivotDeg(-1, 90); }
void turnRight180() { pivotDeg(+1, 90); pivotDeg(+1, 90); }
void turnLeft180()  { pivotDeg(-1, 90); pivotDeg(-1, 90); }

/* ================================================================== gripper */

void keepup_object() {
  gripTo(SERVO_GRIP_CLOSED);       /* close on the object */
  delay(200);
  armTo(SERVO_ARM_CARRY);          /* lift it clear of the field */
  delay(150);
}

void put_object() {
  armTo(SERVO_ARM_DOWN);           /* set it down */
  delay(150);
  gripTo(SERVO_GRIP_RELEASE);      /* let go */
  delay(150);
}

void arm_over_head() {
  armTo(SERVO_ARM_HIGH);
  delay(150);
}

/* ============================================================== telemetry */

static uint32_t g_telLast = 0;
void telemetry(int n) {
  if ((uint32_t)(millis() - g_telLast) < TELEMETRY_MS) return;
  g_telLast = millis();
  Serial.print("n=");    Serial.print(n);
  Serial.print(" m=");   Serial.print(maskToString(g_mask));
  Serial.print(" pos="); Serial.print((int)g_pos);
  Serial.print(" sp=");  Serial.print(sp);
  Serial.print(" cm=");  Serial.print((int)cmSinceJunction());
  if (g_fault) { Serial.print(" FAULT:"); Serial.print(g_fault); }
  Serial.println();
}

#endif
