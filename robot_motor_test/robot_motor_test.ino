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

/* duty > 0 forward, < 0 reverse, 0 brake */
void leftMotor(int duty) {
  digitalWrite(AIN1, duty >= 0);
  digitalWrite(AIN2, duty <= 0);
  analogWrite(PWMA, duty < 0 ? -duty : duty);
}

void rightMotor(int duty) {
  digitalWrite(BIN1, duty >= 0);
  digitalWrite(BIN2, duty <= 0);
  analogWrite(PWMB, duty < 0 ? -duty : duty);
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

  digitalWrite(LED_BUILTIN, LOW);
  Serial.println(F("--- repeating in 2 s ---"));
  Serial.println();
  delay(2000);
}
