/* =====================================================================
 *  robot_test.ino -- bench test rig for the e33 robot
 * =====================================================================
 *
 *  A standalone sketch. It shares NO code with the mission firmware on
 *  purpose: it has to still work when the mission firmware does not, and it
 *  must never be the thing under test. The pin numbers below are the only
 *  thing it duplicates -- check they match config.h if you ever move a wire.
 *
 *  Upload, open the Serial Monitor at 115200, set line ending to "Newline",
 *  and type a number. Nothing moves until you ask it to.
 *
 *    1  sensors      live raw values, with the swing each channel has seen
 *    2  motors       each wheel forward then back, to check wiring
 *    3  servos       set the gripper and arm by hand, to find your angles
 *    4  speed        drive straight for 3 s   -> CM_PER_S_AT_CAL
 *    5  pivot        spin for 4 turns         -> MS_PER_90DEG
 *    6  deadband     ramp the duty until the wheels start -> DUTY_DEADBAND
 *    0  stop         brake everything and return to the menu
 *
 *  Type 0 at any time to stop. Safety: the motor driver is left disabled
 *  until a test that needs it starts, and re-disabled when it finishes.
 * ===================================================================== */

#include <Servo.h>

/* ---- pins: must match robot_countGride/config.h ---------------------- */
#define sp_L   6     /* TB6612 PWMA - left  */
#define F_L   12     /* TB6612 AIN1         */
#define B_L   11     /* TB6612 AIN2         */
#define sp_R   3     /* TB6612 PWMB - right */
#define F_R    7     /* TB6612 BIN1         */
#define B_R    8     /* TB6612 BIN2         */
#define STBY  10     /* TB6612 standby      */
#define x_pin  5     /* gripper servo       */
#define y_pin  4     /* arm lift servo      */
#define PIN_LED 13

/* Duty used by the speed and pivot calibrations. Keep these equal to
 * DUTY_CAL and DUTY_PIVOT in config.h or the numbers you measure will not
 * mean what config.h thinks they mean. */
#define DUTY_CAL    120
#define DUTY_PIVOT   95

static const uint8_t SENSOR_PIN[8] = { A0, A1, A2, A3, A4, A5, A6, A7 };

Servo servo_x, servo_y;
int gripAngle = 140;
int armAngle  = 103;

/* Rolling extremes per channel, so you can slide the robot across a line and
 * read the actual swing instead of guessing at it. */
uint16_t seenLo[8], seenHi[8];

/* ------------------------------------------------------------------ ADC --
 * Same trick the mission firmware uses: after switching channel the ADC's
 * sample-and-hold needs time to charge through a high-impedance sensor, and
 * it only gets the first 1.5 ADC clocks of a conversion to do it. Convert
 * twice and keep the second, or a black line reads less black than it is. */
static uint16_t adcRead(uint8_t ch) {
#if defined(__AVR__)
  ADMUX = (uint8_t)((1 << REFS0) | (ch & 0x0F));
  ADCSRA |= (uint8_t)(1 << ADSC);
  while (ADCSRA & (1 << ADSC)) { }
  (void)ADC;
  ADCSRA |= (uint8_t)(1 << ADSC);
  while (ADCSRA & (1 << ADSC)) { }
  return ADC;
#else
  return analogRead(SENSOR_PIN[ch]);
#endif
}

/* ---------------------------------------------------------------- motors -- */
static void motorL(int duty) {
  if (duty > 0)      { digitalWrite(F_L, 1); digitalWrite(B_L, 0); analogWrite(sp_L, duty); }
  else if (duty < 0) { digitalWrite(F_L, 0); digitalWrite(B_L, 1); analogWrite(sp_L, -duty); }
  else               { digitalWrite(F_L, 1); digitalWrite(B_L, 1); analogWrite(sp_L, 0); }
}
static void motorR(int duty) {
  if (duty > 0)      { digitalWrite(F_R, 1); digitalWrite(B_R, 0); analogWrite(sp_R, duty); }
  else if (duty < 0) { digitalWrite(F_R, 0); digitalWrite(B_R, 1); analogWrite(sp_R, -duty); }
  else               { digitalWrite(F_R, 1); digitalWrite(B_R, 1); analogWrite(sp_R, 0); }
}
static void driverOn()  { digitalWrite(STBY, 1); }
static void driverOff() { motorL(0); motorR(0); digitalWrite(STBY, 0); }

/* True if the user typed 0 (or anything) while a test is running. */
static bool aborted() {
  if (!Serial.available()) return false;
  while (Serial.available()) Serial.read();
  Serial.println(F("  -- stopped --"));
  return true;
}

/* Wait ms, but bail out early if the user types. */
static bool holdFor(uint32_t ms) {
  uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < ms) {
    if (aborted()) return false;
  }
  return true;
}

/* =================================================================== menu */

static void menu() {
  Serial.println();
  Serial.println(F("=========== robot_test ==========="));
  Serial.println(F(" 1  sensors   live values + swing"));
  Serial.println(F(" 2  motors    wiring / direction"));
  Serial.println(F(" 3  servos    find your angles"));
  Serial.println(F(" 4  speed     3 s straight  -> CM_PER_S_AT_CAL"));
  Serial.println(F(" 5  pivot     4 turns       -> MS_PER_90DEG"));
  Serial.println(F(" 6  deadband  ramp up       -> DUTY_DEADBAND"));
  Serial.println(F(" 0  stop"));
  Serial.println(F("=================================="));
}

/* ================================================================ 1: sensors */

static void testSensors() {
  Serial.println(F("SENSORS. Slide the robot on and off a line. Type anything to stop."));
  Serial.println(F("Want each channel LOW over the mat, HIGH over the tape,"));
  Serial.println(F("and a swing of at least 150 counts -- ideally 400+."));
  for (uint8_t i = 0; i < 8; i++) { seenLo[i] = 1023; seenHi[i] = 0; }

  while (!aborted()) {
    Serial.print(F("raw"));
    for (uint8_t i = 0; i < 8; i++) {
      uint16_t v = adcRead(i);
      if (v < seenLo[i]) seenLo[i] = v;
      if (v > seenHi[i]) seenHi[i] = v;
      Serial.print(' ');
      if (v < 100) Serial.print(' ');
      if (v < 10)  Serial.print(' ');
      Serial.print((int)v);
    }
    Serial.print(F("   swing"));
    uint8_t weak = 0;
    for (uint8_t i = 0; i < 8; i++) {
      uint16_t sw = (seenHi[i] > seenLo[i]) ? (uint16_t)(seenHi[i] - seenLo[i]) : 0;
      if (sw < 150) weak++;
      Serial.print(' ');
      if (sw < 100) Serial.print(' ');
      if (sw < 10)  Serial.print(' ');
      Serial.print((int)sw);
    }
    if (weak) { Serial.print(F("   <-- ")); Serial.print((int)weak); Serial.print(F(" weak")); }
    Serial.println();
    digitalWrite(PIN_LED, (millis() / 250) & 1);
    delay(200);
  }
  digitalWrite(PIN_LED, 0);

  Serial.println(F("--- summary: lo / hi / swing per channel ---"));
  for (uint8_t i = 0; i < 8; i++) {
    uint16_t sw = (seenHi[i] > seenLo[i]) ? (uint16_t)(seenHi[i] - seenLo[i]) : 0;
    Serial.print(F("  A")); Serial.print(i);
    Serial.print(F("  lo ")); Serial.print((int)seenLo[i]);
    Serial.print(F("  hi ")); Serial.print((int)seenHi[i]);
    Serial.print(F("  swing ")); Serial.print((int)sw);
    if (sw < 150) Serial.print(F("   WEAK: check height (5-10 mm), dirt, trimpot"));
    Serial.println();
  }
}

/* ================================================================= 2: motors */

static void testMotors() {
  Serial.println(F("MOTORS. Put the robot on a book so the wheels spin free."));
  Serial.println(F("'Forward' means the wheel rolls the way the gripper points."));
  driverOn();
  const __FlashStringHelper *label[6] = {
    F("LEFT  forward"), F("LEFT  reverse"),
    F("RIGHT forward"), F("RIGHT reverse"),
    F("BOTH  forward  (robot would drive straight)"),
    F("SPIN  right    (left fwd, right back)")
  };
  for (uint8_t i = 0; i < 6; i++) {
    Serial.print(F("  ")); Serial.println(label[i]);
    switch (i) {
      case 0: motorL(DUTY_CAL);  motorR(0);          break;
      case 1: motorL(-DUTY_CAL); motorR(0);          break;
      case 2: motorL(0);         motorR(DUTY_CAL);   break;
      case 3: motorL(0);         motorR(-DUTY_CAL);  break;
      case 4: motorL(DUTY_CAL);  motorR(DUTY_CAL);   break;
      default: motorL(DUTY_CAL); motorR(-DUTY_CAL);  break;
    }
    bool ok = holdFor(1500);
    motorL(0); motorR(0);
    if (!ok) { driverOff(); return; }
    if (!holdFor(700)) { driverOff(); return; }
  }
  driverOff();
  Serial.println(F("If a wheel ran backwards, set INVERT_LEFT or INVERT_RIGHT"));
  Serial.println(F("to 1 in config.h. No need to change the wiring."));
}

/* ================================================================= 3: servos */

static void testServos() {
  Serial.println(F("SERVOS. Type: g<angle> for the gripper, a<angle> for the arm."));
  Serial.println(F("  e.g.  g75   a103     Type 0 to finish."));
  Serial.print(F("  now: grip ")); Serial.print(gripAngle);
  Serial.print(F("  arm ")); Serial.println(armAngle);

  String line = "";
  while (true) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\n' || c == '\r') {
        line.trim();
        if (line.length() == 0) { continue; }
        if (line == "0") { Serial.println(F("  -- done --")); return; }
        char what = line.charAt(0);
        int ang = line.substring(1).toInt();
        if ((what == 'g' || what == 'a') && ang >= 0 && ang <= 180) {
          /* step there rather than jumping: an unrestrained sweep pulls enough
           * current to dip the shared 5 V rail and reset the Nano. */
          int *cur = (what == 'g') ? &gripAngle : &armAngle;
          Servo *sv = (what == 'g') ? &servo_x : &servo_y;
          int step = (ang > *cur) ? 3 : -3;
          while (*cur != ang) {
            if ((step > 0 && *cur + step > ang) || (step < 0 && *cur + step < ang)) *cur = ang;
            else *cur += step;
            sv->write(*cur);
            delay(12);
          }
          Serial.print(F("  grip ")); Serial.print(gripAngle);
          Serial.print(F("  arm ")); Serial.println(armAngle);
        } else {
          Serial.println(F("  ? use g<0-180> or a<0-180>, or 0 to finish"));
        }
        line = "";
      } else if (line.length() < 8) {
        line += c;
      }
    }
  }
}

/* ================================================================== 4: speed */

static void testSpeed() {
  Serial.print(F("SPEED. Driving straight for 3 s at duty "));
  Serial.print(DUTY_CAL);
  Serial.println(F(". Give it room. Starting in 3 s..."));
  if (!holdFor(3000)) return;
  driverOn();
  motorL(DUTY_CAL); motorR(DUTY_CAL);
  uint32_t t0 = millis();
  bool ok = holdFor(3000);
  uint32_t ran = millis() - t0;
  driverOff();
  if (!ok) { Serial.println(F("  aborted, ignore the distance")); return; }
  Serial.print(F("  ran for ")); Serial.print((int)ran); Serial.println(F(" ms"));
  Serial.println(F("  Measure the distance with a ruler, then:"));
  Serial.println(F("     CM_PER_S_AT_CAL = distance_in_cm / 3"));
  Serial.println(F("  If it curved, the wheels are mismatched -- use TRIM_R_PCT."));
}

/* ================================================================== 5: pivot */

static void testPivot() {
  Serial.print(F("PIVOT. Spinning on the spot at duty "));
  Serial.print(DUTY_PIVOT);
  Serial.println(F(". Count FOUR full turns and note the seconds."));
  Serial.println(F("Type anything the moment the fourth turn completes."));
  if (!holdFor(2500)) return;
  driverOn();
  motorL(DUTY_PIVOT); motorR(-DUTY_PIVOT);
  uint32_t t0 = millis();
  while (!aborted()) {
    if ((uint32_t)(millis() - t0) > 30000ul) break;   /* never spin forever */
  }
  uint32_t ran = millis() - t0;
  driverOff();
  Serial.print(F("  spun for ")); Serial.print((int)ran); Serial.println(F(" ms"));
  Serial.print(F("  if that was 4 full turns:  MS_PER_90DEG = "));
  Serial.println((int)(ran / 16ul));          /* 4 turns = sixteen 90s */
  Serial.println(F("  (this value is tied to DUTY_PIVOT -- re-measure if you change it)"));
}

/* =============================================================== 6: deadband */

static void testDeadband() {
  Serial.println(F("DEADBAND. Wheels off the ground. Duty ramps up from 0;"));
  Serial.println(F("type anything the INSTANT both wheels start turning."));
  if (!holdFor(2500)) return;
  driverOn();
  int duty = 0;
  while (duty < 160) {
    motorL(duty); motorR(duty);
    Serial.print(F("  duty ")); Serial.println(duty);
    if (aborted()) {
      driverOff();
      Serial.print(F("  DUTY_DEADBAND is about ")); Serial.println(duty);
      Serial.println(F("  Use a value just BELOW where they started."));
      return;
    }
    if (!holdFor(600)) { driverOff(); return; }
    duty += 5;
  }
  driverOff();
  Serial.println(F("  reached 160 without you stopping it -- motors may not be wired"));
}

/* ==================================================================== setup */

void setup() {
  Serial.begin(115200);
  pinMode(sp_L, OUTPUT); pinMode(F_L, OUTPUT); pinMode(B_L, OUTPUT);
  pinMode(sp_R, OUTPUT); pinMode(F_R, OUTPUT); pinMode(B_R, OUTPUT);
  pinMode(STBY, OUTPUT); pinMode(PIN_LED, OUTPUT);
  driverOff();                       /* nothing moves until asked */

#if defined(__AVR__)
  ADCSRA = (uint8_t)((ADCSRA & ~0x07) | 0x05);   /* 500 kHz ADC, as the firmware uses */
#endif

  servo_x.attach(x_pin); servo_x.write(gripAngle);
  servo_y.attach(y_pin); servo_y.write(armAngle);

  delay(400);
  Serial.println();
  Serial.println(F("robot_test ready -- set the Serial Monitor to 'Newline'"));
  menu();
}

void loop() {
  if (!Serial.available()) return;
  char c = (char)Serial.read();
  while (Serial.available()) Serial.read();      /* drop the rest of the line */

  switch (c) {
    case '1': testSensors();  break;
    case '2': testMotors();   break;
    case '3': testServos();   break;
    case '4': testSpeed();    break;
    case '5': testPivot();    break;
    case '6': testDeadband(); break;
    case '0': driverOff(); Serial.println(F("stopped")); break;
    default : return;                            /* ignore stray newlines */
  }
  menu();
}
