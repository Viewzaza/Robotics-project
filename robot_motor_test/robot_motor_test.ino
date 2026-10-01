/* =====================================================================
 *  robot_motor_test.ino -- just run the motors
 * =====================================================================
 *
 *  Upload it and watch. No menu, no typing. It loops forever through:
 *
 *      A forward, A back, B forward, B back, both forward, both back, spin
 *
 *  announcing each step over serial at 115200.
 *
 *  WIRING
 *      Left  motor -> AO1 and AO2   (driven by PWMA/AIN1/AIN2)
 *      Right motor -> BO1 and BO2   (driven by PWMB/BIN1/BIN2)
 *
 *  A motor has two plain wires and AO1/AO2 are two plain terminals, so there
 *  is no correct order -- swapping them just reverses that wheel. Connect them
 *  either way, run this, and see which way each wheel actually turns.
 *
 *  "Forward" below means the wheel should roll the way the gripper points.
 *  If a wheel goes the wrong way you can either swap its two wires, or leave
 *  the wiring alone and set INVERT_LEFT / INVERT_RIGHT in the mission
 *  firmware's config.h. Both work; the flag is easier.
 *
 *  Put the robot on a book so the wheels spin free.
 *
 *  IF ONE WHEEL TURNS FURTHER THAN THE OTHER
 *      Check it mechanically first -- spin both wheels by hand with the power
 *      off. If one feels stiffer, that is friction, and no amount of trim fixes
 *      a wheel that is rubbing or a gearbox that is binding.
 *      If they feel the same, set TRIM_RIGHT_PCT below until the STRAIGHT test
 *      keeps a straight line, then copy that number into TRIM_R_PCT in
 *      robot_countGride/config.h.
 * ===================================================================== */

/* ---- TB6612FNG, same pins as the mission firmware -------------------- */
#define PWMA   6    /* left  speed     */
#define AIN1  12    /* left  direction */
#define AIN2  11
#define PWMB   3    /* right speed     */
#define BIN1   7    /* right direction */
#define BIN2   8
#define STBY  10    /* driver enable: LOW = both motors off */

#define SPEED      120   /* 0-255 */
#define RUN_MS    2000   /* how long each step runs  */
#define PAUSE_MS   800   /* stopped between steps    */

/* Right-motor trim, percent. 100 = untouched. If one wheel turns further than
 * the other in the same time, scale the faster one down until they match, then
 * copy the value into TRIM_R_PCT in the mission firmware's config.h.
 *   right turns 360 while left turns 270  ->  270/360 = 75  ->  set 75 */
#define TRIM_RIGHT_PCT  100

/* duty > 0 forward, < 0 reverse, 0 brake */
void leftMotor(int duty) {
  digitalWrite(AIN1, duty >= 0);
  digitalWrite(AIN2, duty <= 0);
  analogWrite(PWMA, duty < 0 ? -duty : duty);
}

void rightMotor(int duty) {
  long d = (long)duty * TRIM_RIGHT_PCT / 100;
  if (d > 255) d = 255;
  if (d < -255) d = -255;
  digitalWrite(BIN1, d >= 0);
  digitalWrite(BIN2, d <= 0);
  analogWrite(PWMB, (int)(d < 0 ? -d : d));
}

void stopBoth() {
  leftMotor(0);
  rightMotor(0);
  delay(PAUSE_MS);
}

void step(const __FlashStringHelper *what, int left, int right) {
  Serial.println(what);
  leftMotor(left);
  rightMotor(right);
  delay(RUN_MS);
  stopBoth();
}

void setup() {
  Serial.begin(115200);
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  /* Put both motors on the SAME PWM frequency before comparing them.
   *
   * Arduino's defaults do not: D6 (left) is on Timer0 at ~976 Hz, D3 (right) is
   * on Timer2 at ~490 Hz. A brushed motor does not respond identically to two
   * different switching frequencies, so with stock analogWrite the two wheels
   * can turn at visibly different speeds for the same duty even when the motors
   * and the wiring are fine. Timer2 prescaler /32 puts D3 on ~980 Hz to match.
   *
   * The mission firmware does the same thing, so trim measured here is the trim
   * that firmware needs. Timer0 is left alone -- millis() and delay() live on
   * it and changing its prescaler would break their timing. */
#if defined(__AVR__)
  TCCR2B = (uint8_t)((TCCR2B & 0xF8) | 0x03);
#endif

  digitalWrite(STBY, LOW);          /* stay off while everything settles */
  stopBoth();
  delay(1000);
  digitalWrite(STBY, HIGH);         /* driver enabled */

  Serial.println();
  Serial.println(F("motor test -- wheels off the ground"));
  Serial.println(F("forward = the way the gripper points"));
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);

  step(F("A (LEFT)  forward"),  SPEED,  0);
  step(F("A (LEFT)  reverse"), -SPEED,  0);
  step(F("B (RIGHT) forward"),  0,      SPEED);
  step(F("B (RIGHT) reverse"),  0,     -SPEED);
  step(F("BOTH forward  -- robot would drive straight"),  SPEED,  SPEED);
  step(F("BOTH reverse"),                                -SPEED, -SPEED);
  step(F("SPIN right    -- left forward, right back"),    SPEED, -SPEED);

  /* Long straight run: the one that actually shows a speed mismatch. Count how
   * far each wheel turns, or put it on the floor and see whether it curves. */
  Serial.println(F("STRAIGHT 5 s -- count the turns of each wheel"));
  leftMotor(SPEED);
  rightMotor(SPEED);
  delay(5000);
  stopBoth();
  Serial.print(F("  trim now "));
  Serial.print(TRIM_RIGHT_PCT);
  Serial.println(F("% -- if the right still turns further, lower it"));

  digitalWrite(LED_BUILTIN, LOW);
  Serial.println(F("--- repeating in 2 s ---"));
  Serial.println();
  delay(2000);
}
