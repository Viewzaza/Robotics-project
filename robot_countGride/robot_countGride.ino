/* =====================================================================
 *  robot_countGride.ino      Robot mission (worksheet e33)
 * =====================================================================
 *
 *  Field  (o = object, x = drop position, [R]> = robot start, facing right)
 *
 *          o1                                  o3
 *     -----+-------+-------+-------+-----      <- TOP
 *          |       |       |       |
 *     --[R]>-------+-------+-------+-----      <- MID
 *          |       |       |       |
 *     -----+-------+-------+-------+-----      <- BOT
 *          o2      x3      x2      x1
 *          C1      C2      C3      C4
 *
 *    o1 = top of C1     -> x1 = bottom of C4
 *    o2 = bottom of C1  -> x2 = bottom of C3
 *    o3 = top of C4     -> x3 = bottom of C2
 *
 *  Order run: 3 -> 1 -> 2. The robot starts facing right, so it collects the
 *  right-most object first, and the route never travels along the bottom line,
 *  so it cannot disturb an object it has already placed.
 *
 * ---------------------------------------------------------------------
 *  THE 9.5 cm RULE
 *
 *  The sensor bar sits SENSOR_AHEAD_CM = 9.5 cm in front of the wheel axle,
 *  which is the point the robot turns about. So at the instant the bar reports
 *  a crossing, the pivot centre is still 9.5 cm short of it. Pivoting there
 *  turns about a point 9.5 cm before the intersection and throws the robot off
 *  the line. Every manoeuvre below therefore advances the measured remaining
 *  distance FIRST, at a fixed low duty so it is repeatable, and only then
 *  pivots. See turn90(), keep_item() and place_item().
 *
 * ---------------------------------------------------------------------
 *  Route -- n is the value of numGride after countGrid() sees the crossing.
 *  Action cases add 1 of their own, which is why the numbers skip.
 *
 *  n    Point      Heading         Action
 *   1   (C2,MID)   E               followLine
 *   2   (C3,MID)   E               followLine
 *   3   (C4,MID)   E -> N          turn90("LEFT")
 *   5   (C4,TOP)   N -> S          keep_item    pick object 3
 *   7   (C4,MID)   S -> W          turn90("RIGHT")
 *   9   (C3,MID)   W               followLine
 *  10   (C2,MID)   W -> S          turn90("LEFT")
 *  12   (C2,BOT)   S -> N          place_item   drop object 3 at x3
 *  14   (C2,MID)   N -> W          turn90("LEFT")
 *  16   (C1,MID)   W -> N          turn90("RIGHT")
 *  18   (C1,TOP)   N -> S          keep_item    pick object 1
 *  20   (C1,MID)   S -> E          turn90("LEFT")
 *  22   (C2,MID)   E               followLine
 *  23   (C3,MID)   E               followLine
 *  24   (C4,MID)   E -> S          turn90("RIGHT")
 *  26   (C4,BOT)   S -> N          place_item   drop object 1 at x1
 *  28   (C4,MID)   N -> W          turn90("LEFT")
 *  30   (C3,MID)   W               followLine
 *  31   (C2,MID)   W               followLine
 *  32   (C1,MID)   W -> S          turn90("LEFT")
 *  34   (C1,BOT)   S -> N          keep_item    pick object 2
 *  36   (C1,MID)   N -> E          turn90("RIGHT")
 *  38   (C2,MID)   E               followLine
 *  39   (C3,MID)   E -> S          turn90("RIGHT")
 *  41   (C3,BOT)   S               place_item("STOP")  drop object 2 at x2
 *  42   mission complete -> stopRobot()
 * ===================================================================== */

#include "config.h"
#include "controlLibrary.h"

#define END_GRIDE 42

int numGride = 0;

/* Why did the chip last reset? MCUSR holds the answer, but only until something
 * clears it, so grab it in .init3 -- before main(), before any constructor.
 * The variable lives in .noinit so the C runtime does not zero it on the way
 * past. If the bootloader already cleared MCUSR we read 0, which reads as "not
 * a crash" and starts the mission fresh: the safe way to be wrong. */
#if defined(__AVR__)
  #define RESET_POR _BV(PORF)
  #define RESET_EXT _BV(EXTRF)
  #define RESET_BOR _BV(BORF)
  #define RESET_WDT _BV(WDRF)
  uint8_t g_resetFlags __attribute__((section(".noinit")));
  void grabResetFlags(void) __attribute__((naked, used, section(".init3")));
  void grabResetFlags(void) { g_resetFlags = MCUSR; MCUSR = 0; }
#else
  #define RESET_POR 0x01
  #define RESET_EXT 0x02
  #define RESET_BOR 0x04
  #define RESET_WDT 0x08
  uint8_t g_resetFlags = 0;      /* host simulator: always a fresh start */
#endif

void turn90(String direction);
void keep_item(String direction);
void place_item(String direction);
static void missionCheckpoint();
static void runCalibration();
static void waitForStart();

/* After pivoting about a junction the bar is already SENSOR_AHEAD_CM into the
 * next cell, so that is where the distance-to-next-junction estimate restarts.
 * Getting this right is what lets followLine() decelerate in time. */
static void reanchor() {
  g_odoAtJunction = g_odoCm - SENSOR_AHEAD_CM;
  g_inJunction = false;
  clearPid();
  sp = DUTY_START;
  tUpSp = millis();
}

/* ===================================================================== setup */

void setup() {
  Serial.begin(SERIAL_BAUD);
  beginFnc();

  /* Holding the start button while powering up enters calibration instead of
   * running the mission. Without a button fitted the pin idles high and this
   * never fires, so the robot behaves exactly as before. */
  if (digitalRead(PIN_START) == LOW) {
    runCalibration();
  }

  if (!g_calValid) {
    Serial.println(F("WARNING: no EEPROM calibration, using compiled defaults"));
  }

#if ENABLE_CRASH_RESUME
  /* Recover the mission step if an earlier run was cut short by a brown-out.
   * A servo stalling on a gripped object can dip the shared 5 V rail far enough
   * to reset the Nano; restarting from zero with objects already moved is
   * unrecoverable, so the step counter is checkpointed after every action.
   *
   * It is NOT enough to find a checkpoint and resume from it. A student testing
   * on the bench interrupts runs constantly, and every one of those leaves a
   * checkpoint behind. Resuming on the next power-up would have them place the
   * robot on the start line only to watch it set off as though it were already
   * twenty steps in. So ask the chip WHY it reset: only a brown-out or a
   * watchdog bite is a crash. Powering up, or pressing reset, is deliberate and
   * starts the mission over. If the bootloader cleared the flags before we could
   * read them, g_resetFlags is 0 and we start fresh, which is the safe way to
   * be wrong. */
  bool crashed = (g_resetFlags & (RESET_BOR | RESET_WDT)) != 0
              && (g_resetFlags & (RESET_POR | RESET_EXT)) == 0;

  uint16_t saved[2];
  EEPROM.get(EE_ADDR_CHECKPOINT, saved);
  bool haveCheckpoint = (saved[0] == (uint16_t)(EE_MAGIC & 0xFFFF)
                         && saved[1] > 0 && saved[1] < END_GRIDE);

  if (crashed && haveCheckpoint) {
    numGride = (int)saved[1];
    Serial.print(F("RESUME after brown-out at numGride=")); Serial.println(numGride);
  } else if (haveCheckpoint) {
    Serial.println(F("checkpoint found but this was a normal power-up: starting over"));
  }
#endif

  waitForStart();
  g_odoLastUs = micros();
  g_odoAtJunction = g_odoCm;
}

/* ====================================================================== loop */

void loop() {
  if (numGride >= END_GRIDE) {
    stopRobot();
    digitalWrite(PIN_LED, 1);
    return;
  }

  numGride = countGrid(numGride);
  telemetry(numGride);

  switch (numGride) {

    /* ---------- object 3: top of C4  ->  drop at x3, bottom of C2 ------- */
    case  3: turn90("LEFT");       numGride++; break;  /* (C4,MID) face north */
    case  5: keep_item("RIGHT");   numGride++; break;  /* (C4,TOP) pick obj 3 */
    case  7: turn90("RIGHT");      numGride++; break;  /* (C4,MID) face west  */
    case 10: turn90("LEFT");       numGride++; break;  /* (C2,MID) face south */
    case 12: place_item("RIGHT");  numGride++; break;  /* (C2,BOT) drop obj 3 */

    /* ---------- object 1: top of C1  ->  drop at x1, bottom of C4 ------- */
    case 14: turn90("LEFT");       numGride++; break;  /* (C2,MID) face west  */
    case 16: turn90("RIGHT");      numGride++; break;  /* (C1,MID) face north */
    case 18: keep_item("RIGHT");   numGride++; break;  /* (C1,TOP) pick obj 1 */
    case 20: turn90("LEFT");       numGride++; break;  /* (C1,MID) face east  */
    case 24: turn90("RIGHT");      numGride++; break;  /* (C4,MID) face south */
    case 26: place_item("RIGHT");  numGride++; break;  /* (C4,BOT) drop obj 1 */

    /* ---------- object 2: bottom of C1  ->  drop at x2, bottom of C3 ---- */
    case 28: turn90("LEFT");       numGride++; break;  /* (C4,MID) face west  */
    case 32: turn90("LEFT");       numGride++; break;  /* (C1,MID) face south */
    case 34: keep_item("RIGHT");   numGride++; break;  /* (C1,BOT) pick obj 2 */
    case 36: turn90("RIGHT");      numGride++; break;  /* (C1,MID) face east  */
    case 39: turn90("RIGHT");      numGride++; break;  /* (C3,MID) face south */
    case 41: place_item("STOP");   numGride++; break;  /* (C3,BOT) drop obj 2 */

    default: followLine();
  }
}

/* ================================================================= actions */

/* Turn 90 degrees ABOUT THE INTERSECTION, not about wherever the robot happened
 * to stop. cmSinceJunction() is how far the robot has rolled since the bar
 * crossed the line, so the remaining advance is 9.5 cm minus that. */
void turn90(String direction) {
  float togo = SENSOR_AHEAD_CM - cmSinceJunction();
  brake();
  advanceCm(togo);
  if (direction == "RIGHT") turnRight90(); else turnLeft90();
  brake();
  reanchor();
  missionCheckpoint();
}

/* Pick up the object sitting OBJECT_BEYOND_CM past the line, then turn around.
 * The jaws are GRIP_REACH_CM ahead of the pivot and the bar SENSOR_AHEAD_CM,
 * so from the moment of detection the pivot must advance
 *     SENSOR_AHEAD_CM + OBJECT_BEYOND_CM - GRIP_REACH_CM
 * for the jaws to close around the object. With 9.5 + 5.0 - 12.0 that is
 * 2.5 cm; if your gripper reaches further than the bar sees, it goes negative
 * and the robot correctly stops short.
 *
 * The advance is measured from the CENTRE of the outer line -- cmPastJunction()
 * rather than cmSinceJunction() -- because a junction is declared when the bar's
 * leading edge reaches the tape, half a tape-width early. That is worth a flat
 * 0.9 cm of extra reach on every pick, which is most of the margin a gripper
 * that is 1 cm shorter than GRIP_REACH_CM needs.
 *
 * It is then given straight back, by creepBack(), BEFORE the robot spins. The
 * pivot is already only about cell-7 cm inside the outer line, and the bar
 * sweeps a circle of radius SENSOR_AHEAD_CM about the pivot, so pushing the
 * pivot a further centimetre towards that line widens the arc over which the
 * bar sits squarely on it -- and pivotDeg() then ends the 180 on the outer line
 * instead of the column line and the robot leaves the field. Measured: taking
 * the extra reach without giving it back costs 240/315 against a 267 baseline.
 * Reaching further to grip and backing off to turn keeps both. */
void keep_item(String direction) {
  float want = SENSOR_AHEAD_CM + OBJECT_BEYOND_CM - GRIP_REACH_CM;
  float togo = want - cmPastJunction();
  brake();
  float lead = 0;
  if (togo > 0) { advanceCm(togo); lead = togo < g_crossHalfCm ? togo : g_crossHalfCm; }
  delay(150);

  keepup_object();

  creepBack(lead);                   /* spin from where an uncorrected pick would have */
  if (direction == "RIGHT") turnRight180(); else turnLeft180();
  brake();
  reanchor();
  missionCheckpoint();
}

/* Place the object. "RIGHT"/"LEFT" turn around afterwards to carry on;
 * "STOP" leaves the robot where it is, for the final drop. */
void place_item(String direction) {
  float want = SENSOR_AHEAD_CM + OBJECT_BEYOND_CM - GRIP_REACH_CM;
  float togo = want - cmPastJunction();
  brake();
  float lead = 0;
  if (togo > 0) { advanceCm(togo); lead = togo < g_crossHalfCm ? togo : g_crossHalfCm; }
  delay(150);

  put_object();

  if (direction == "STOP") {
    reverseCm(4.0f);                 /* back off so the arm clears the object */
    armTo(SERVO_ARM_HIGH);
    brake();
    missionCheckpoint();
    return;
  }

  arm_over_head();                   /* lift clear before swinging round */
  creepBack(lead);                   /* see keep_item() */
  if (direction == "RIGHT") turnRight180(); else turnLeft180();
  brake();
  armTo(SERVO_ARM_DOWN);
  gripTo(SERVO_GRIP_OPEN);
  reanchor();
  missionCheckpoint();
}

/* ============================================================== checkpoint */

static void missionCheckpoint() {
#if ENABLE_CRASH_RESUME
  /* One 4-byte record, written only after an action completes -- about 18
   * writes per run. EEPROM is rated for 100k writes per cell, so this is
   * thousands of practice runs. */
  uint16_t rec[2] = { (uint16_t)(EE_MAGIC & 0xFFFF), (uint16_t)(numGride + 1) };
  EEPROM.put(EE_ADDR_CHECKPOINT, rec);
#endif
}

static void clearCheckpoint() {
  uint16_t rec[2] = { 0, 0 };
  EEPROM.put(EE_ADDR_CHECKPOINT, rec);
}

/* =================================================================== start */

/* Gives the student time to put the robot down and let go. Without a button
 * fitted it falls through after a short wait, so nothing breaks. */
static void waitForStart() {
  Serial.println(F("ready -- press start (or wait 3 s)"));
  uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < 3000) {
    digitalWrite(PIN_LED, ((millis() / 150) & 1));
    if (digitalRead(PIN_START) == LOW) {
      while (digitalRead(PIN_START) == LOW) { }   /* wait for release */
      break;
    }
  }
  digitalWrite(PIN_LED, 0);
  clearCheckpoint();
  Serial.println(F("GO"));
}

/* ============================================================= calibration */

/* Measures, in order:
 *   1. per-channel background and tape levels, by sweeping across a line
 *   2. the dead band, by ramping the duty until the robot actually moves
 *   3. forward speed at DUTY_CAL, by driving straight for 3 s
 *   4. the time for a 90 degree pivot
 * and stores the result in EEPROM. Everything is reported over serial so a
 * student can check the numbers look sane before trusting them. */
static void runCalibration() {
  Serial.println(F("=== CALIBRATION ==="));
  calDefaults();
  g_calValid = false;

  Serial.println(F("1. sensors: sweeping across the line"));
  uint16_t lo[8], hi[8];
  for (uint8_t i = 0; i < 8; i++) { lo[i] = 1023; hi[i] = 0; }
  digitalWrite(STBY, 1);
  for (uint8_t pass = 0; pass < 4; pass++) {
    spin(pass & 1 ? -1 : 1, DUTY_PIVOT - 20);
    uint32_t t0 = millis();
    while ((uint32_t)(millis() - t0) < 700) {
      for (uint8_t i = 0; i < 8; i++) {
        uint16_t v = analogRead(SENSOR_PIN[i]);
        if (v < lo[i]) lo[i] = v;
        if (v > hi[i]) hi[i] = v;
      }
    }
  }
  brake();
  for (uint8_t i = 0; i < 8; i++) {
    g_cal.lo[i] = lo[i];
    g_cal.hi[i] = hi[i];
    Serial.print(F("  ch")); Serial.print(i);
    Serial.print(F(" lo=")); Serial.print(lo[i]);
    Serial.print(F(" hi=")); Serial.print(hi[i]);
    if (hi[i] - lo[i] < 120) Serial.print(F("  <-- WEAK, check this sensor"));
    Serial.println();
  }

  Serial.println(F("2. speed at DUTY_CAL for 3 s -- measure the distance"));
  delay(1500);
  driveStraight(DUTY_CAL);
  delay(3000);
  brake();
  Serial.println(F("   enter cm/3 into CM_PER_S_AT_CAL in config.h"));

  Serial.println(F("3. pivot: 4 full turns, time them"));
  delay(1500);
  uint32_t p0 = millis();
  spin(1, DUTY_PIVOT);
  delay((uint32_t)MS_PER_90DEG * 16ul);
  brake();
  Serial.print(F("   commanded ")); Serial.print((int)(millis() - p0));
  Serial.println(F(" ms for 1440 deg; adjust MS_PER_90DEG by the error"));

  calSave();
  Serial.println(F("=== saved to EEPROM -- power-cycle to run ==="));
  while (1) { digitalWrite(PIN_LED, ((millis() / 400) & 1)); }
}
