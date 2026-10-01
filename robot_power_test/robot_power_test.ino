/* =====================================================================
 *  robot_power_test  (worksheet e33)
 *
 *  A test of the robot's power supply. No route, no line following.
 *  Put the robot on a box with the WHEELS IN THE AIR, switch it on, and
 *  it runs a fixed sequence:
 *    - both servos move the way a pick and a place move them,
 *    - the motors start, and reverse at once, like a spin,
 *    - servos and motors together (the moment your robot restarted).
 *  During every step it measures the 5 V supply all the time (thousands of
 *  readings) and keeps the lowest and the average. At the end it prints a
 *  verdict: OK / WEAK / WILL RESTART. Serial Monitor at 115200.
 *
 *  It measures the supply the same way robot_e33_v2 does (the POWER
 *  lines): the ADC reads the chip's own 1.1 V reference (the bandgap)
 *  against the supply, so supply = 1.1 V x 1023 / reading. No extra wire.
 *
 *  The results are also saved in the EEPROM, step by step, so you can run
 *  the test on the battery ONLY (no USB cable: the USB cable holds the 5 V
 *  up and hides the problem), then plug the cable in and open the Serial
 *  Monitor: the start of the next run prints the saved results first.
 *  If the Nano restarts during the test, the next start says so and names
 *  the step. See README.md.
 *
 *  It uses the last 34 bytes of the EEPROM (0x3DE..0x3FF). Your
 *  calibration (below 0x140) is not touched; the run log of the mission
 *  firmware only loses its last records, if it had that many.
 * ===================================================================== */
#include <Servo.h>
#include <EEPROM.h>
#if defined(__AVR__)
  #include <avr/boot.h>
#endif

/* ---- the robot (same pins as robot_e33_v2) ---- */
#define sp_L      6     /* left motor PWM  */
#define F_L      12
#define B_L      11
#define sp_R      3     /* right motor PWM */
#define F_R       7
#define B_R       8
#define STBY     10     /* TB6612 standby: 1 = motors on */
#define GRIP_PIN  5     /* gripper servo (servo_x) */
#define ARM_PIN   4     /* arm servo (servo_y) */
#define LED_PIN  13

/* ---- your servo angles (calibration.h of robot_e33_v2) ---- */
#define GRIP_OPEN       140
#define GRIP_CLOSED      75
#define GRIP_RELEASE    149
#define GRIP_BACKOFF_DEG  3
#define ARM_DOWN        103
#define ARM_CARRY        70
#define ARM_HIGH         50

/* ---- the motor duty for the spin steps: about what a turn at TURN_SP
 * uses on your robot (dead band + scale). 255 = full. ---- */
#define SPIN_DUTY_L     110
#define SPIN_DUTY_R      90

/* ---- the measurement ---- */
#define BANDGAP_MV     1100   /* the chip's 1.1 V in mV; same as BANDGAP_MV in
                               * calibration.h (1.0 to 1.2 V on a real chip)  */

/* ---- the verdict, from the LOWEST reading of a step ----
 * Your real runs: at rest 5.02 V; at the picks and places the lowest
 * readings were 4.41 to 4.63 V, and the Nano restarted 12 times, at a
 * servo move or the turn right after it. A reading is a snapshot that
 * misses the shortest and deepest part of a dip (README: "Why the
 * minimum is not the real bottom"), so the real dips went all the way
 * down to the reset level while the readings said about 4.5 V. */
#define V_OK_MV        4750   /* lowest >= 4.75 V: OK                         */
#define V_RESTART_MV   4600   /* lowest <  4.60 V: WILL RESTART (as your runs) */
#define V_REST_MV      4850   /* standing still below 4.85 V: the supply is low
                               * even with nothing moving                       */

#define COUNTDOWN_S      10   /* seconds before the test starts */

/* ---- the steps ---- */
#define N_STEPS          15

const __FlashStringHelper* stepName(uint8_t k){
  switch(k){
    case 0:  return F("rest: servos off, motors off");
    case 1:  return F("servos switched on (attach, hold)");
    case 2:  return F("pick: gripper closes 140 -> 75");
    case 3:  return F("pick: gripper eases 3, arm lifts 103 -> 70");
    case 4:  return F("place: arm down 70 -> 103");
    case 5:  return F("place: gripper opens 78 -> 149");
    case 6:  return F("place: arm high 103 -> 50");
    case 7:  return F("back: arm 50 -> 103, gripper 149 -> 140");
    case 8:  return F("slides' code: gripper JUMPS 140 -> 75 -> 140");
    case 9:  return F("motors: spin starts from standing (1 s)");
    case 10: return F("motors: spin reversed at once (1 s)");
    case 11: return F("motors: stop, then rest");
    case 12: return F("together: spin starts, gripper closes, arm lifts");
    case 13: return F("together: spin reversed, arm high");
    default: return F("rest at the end, everything back");
  }
}

/* =====================================================================
 *  WHY DID THE CHIP LAST RESET?  (the same code as robot_e33_v2)
 *  The chip writes the reason into MCUSR. It is read here, before the
 *  Arduino start-up code runs, and cleared, so the next start shows only
 *  its own reason.
 * ===================================================================== */
#define RUN_MARK 0xE33B0F75UL          /* "the power test is running" */
#if defined(__AVR__)
  uint8_t resetFlags __attribute__((section(".noinit")));
  uint8_t resetR2    __attribute__((section(".noinit")));
  /* this word survives a restart (not switching off): it holds RUN_MARK
   * while the test runs, so a restart in the middle of it is recognised */
  uint32_t runMark   __attribute__((section(".noinit")));
  void grabResetFlags(void) __attribute__((naked, used, section(".init3")));
  void grabResetFlags(void){
    uint8_t r;
    __asm__ __volatile__("mov %0, r2" : "=r"(r));   /* newer Optiboot passes MCUSR in r2 */
    resetR2 = r;
    resetFlags = MCUSR;
    MCUSR = 0;
  }
  #define RF_POR _BV(PORF)
  #define RF_EXT _BV(EXTRF)
  #define RF_BOR _BV(BORF)
  #define RF_WDT _BV(WDRF)
  /* Only Optiboot 6 and newer put MCUSR in r2 (its version is the last byte
   * of the flash); older bootloaders leave r2 as it was. */
  bool r2HasFlags(){ uint8_t v = pgm_read_byte(FLASHEND); return v >= 6 && v != 0xFF; }
#else
  /* host simulator: it sets resetFlags and keeps runMark across --reset */
  uint8_t resetFlags = 0x01, resetR2 = 0;
  uint32_t runMark = 0;
  int numGride = 0;                    /* (the simulator prints the mission count) */
  bool r2HasFlags(){ return false; }
  #define RF_POR 0x01
  #define RF_EXT 0x02
  #define RF_BOR 0x04
  #define RF_WDT 0x08
#endif

uint8_t resetCause(){
  uint8_t f = resetFlags & 0x0F;
  if(f == 0 && r2HasFlags() && (resetR2 & 0xF0) == 0) f = resetR2;
  return f;
}

void printResetCause(){
  uint8_t f = resetCause();
  Serial.print(F("reset cause (MCUSR): "));
  if(f & RF_POR) Serial.print(F("power-on "));
  if(f & RF_EXT) Serial.print(F("reset-button/USB "));
  if(f & RF_BOR) Serial.print(F("BROWN-OUT "));
  if(f & RF_WDT) Serial.print(F("watchdog "));
  if(f == 0)     Serial.print(F("unknown (the bootloader cleared it)"));
  Serial.println();
}

/* The brown-out detector level, from the chip's fuses (the "extended fuse").
 * A Nano with the usual fuses resets at 2.7 V; some clones have it off. */
void printBrownOutLevel(){
#if defined(__AVR__)
  uint8_t e = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS) & 0x07;
  Serial.print(F("brown-out detector (fuse): "));
  switch(e){
    case 7: Serial.println(F("OFF (a dip makes the chip misbehave, then restart as power-on)")); break;
    case 6: Serial.println(F("1.8 V")); break;
    case 5: Serial.println(F("2.7 V (the usual Nano setting)")); break;
    case 4: Serial.println(F("4.3 V (very sensitive: restarts at small dips)")); break;
    default: Serial.println(F("unknown")); break;
  }
#else
  Serial.println(F("brown-out detector (fuse): 2.7 V (simulator: --bod)"));
#endif
}

/* =====================================================================
 *  THE SUPPLY VOLTAGE
 *  The ADC compares its input with the supply (AVcc). Its input is set
 *  once, to the chip's 1.1 V, and left there (this sketch reads no
 *  sensor), so every conversion is one reading of the supply. ADC clock
 *  500 kHz, as robot_e33_v2 uses: one reading every ~30 microseconds.
 * ===================================================================== */
#if defined(__AVR__)
void adcSetup(){
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0);   /* on, clock 16 MHz / 32 */
  ADMUX  = _BV(REFS0) | 0x0E;                     /* reference AVcc; input = the 1.1 V */
  delay(5);                                       /* the 1.1 V needs time to settle */
  for(uint8_t k=0;k<8;k++){ ADCSRA |= _BV(ADSC); while(ADCSRA & _BV(ADSC)); }
}
uint16_t bandgapAdc(){
  ADCSRA |= _BV(ADSC);
  while(ADCSRA & _BV(ADSC));
  return ADC;
}
#else
uint16_t sim_bandgap_adc();              /* host simulator: its supply model */
void adcSetup(){}
uint16_t bandgapAdc(){ delayMicroseconds(28); return sim_bandgap_adc(); }
#endif

/* supply in mV, or 0 when the reading cannot be real (outside about
 * 2.5 to 6.6 V), exactly as vccMv() in robot_e33_v2 */
uint16_t vccMv(){
  uint16_t a = bandgapAdc();
  if(a < 170 || a > 450) return 0;
  return (uint16_t)((uint32_t)BANDGAP_MV * 1023UL / a);
}

/* what one step saw */
uint16_t accMin;           /* lowest reading, mV (0 = none yet) */
uint32_t accSum;           /* sum of the readings */
uint32_t accN;             /* number of readings */
uint32_t accLow;           /* readings below V_RESTART_MV */
uint32_t accBad;           /* readings that cannot be real */

void accClear(){ accMin = 0; accSum = 0; accN = 0; accLow = 0; accBad = 0; }

void sampleOnce(){
  uint16_t v = vccMv();
  if(!v){ accBad++; return; }
  if(!accMin || v < accMin) accMin = v;
  accSum += v;
  accN++;
  if(v < V_RESTART_MV) accLow++;
}

/* wait ms, reading the supply all the time */
void sampleFor(uint16_t ms){
  unsigned long t0 = millis();
  do sampleOnce(); while(millis() - t0 < ms);
}

uint16_t accAvg(){ return accN ? (uint16_t)(accSum / accN) : 0; }

/* =====================================================================
 *  SERVOS AND MOTORS (the same moves as robot_e33_v2)
 * ===================================================================== */
Servo servo_x;             /* gripper */
Servo servo_y;             /* arm */
int gripNow = GRIP_OPEN;
int armNow  = ARM_DOWN;

/* v2's servoTo: 3 degrees per 20 ms (one Servo pulse), reading all the time */
void servoTo(Servo &s, int &now, int target){
  while(now != target){
    int d = target - now;
    if(d > 3) d = 3;
    if(d < -3) d = -3;
    now += d;
    s.write(now);
    sampleFor(20);
  }
}

/* the slides' code: write the angle at once; the servo runs at full speed */
void servoJump(Servo &s, int &now, int target){
  now = target;
  s.write(target);
  sampleFor(600);
}

/* both servos one step each, at the same time (for the "together" steps) */
void servosTo(int gTarget, int aTarget){
  while(gripNow != gTarget || armNow != aTarget){
    int d = gTarget - gripNow;
    if(d > 3) d = 3;
    if(d < -3) d = -3;
    gripNow += d;
    d = aTarget - armNow;
    if(d > 3) d = 3;
    if(d < -3) d = -3;
    armNow += d;
    servo_x.write(gripNow);
    servo_y.write(armNow);
    sampleFor(20);
  }
}

/* spin: dir 1 = left wheel forward, right wheel back; -1 the other way;
 * 0 = stop (brake off, wheels free) */
void spin(int8_t dir){
  digitalWrite(F_L, dir > 0); digitalWrite(B_L, dir < 0);
  digitalWrite(F_R, dir < 0); digitalWrite(B_R, dir > 0);
  analogWrite(sp_L, dir ? SPIN_DUTY_L : 0);
  analogWrite(sp_R, dir ? SPIN_DUTY_R : 0);
}

/* the rest of a step: keep reading until ms have passed since t0 */
void sampleUntil(unsigned long t0, uint16_t ms){
  while(millis() - t0 < ms) sampleOnce();
}

void runStep(uint8_t k){
  unsigned long t0 = millis();
  switch(k){
    case 0:  sampleFor(1000); break;
    case 1:  /* write the angle BEFORE attach (as v2): the first pulse is right */
             servo_x.write(GRIP_OPEN); servo_x.attach(GRIP_PIN); sampleFor(300);
             servo_y.write(ARM_DOWN);  servo_y.attach(ARM_PIN);  sampleFor(700);
             gripNow = GRIP_OPEN; armNow = ARM_DOWN;
             break;
    case 2:  servoTo(servo_x, gripNow, GRIP_CLOSED); sampleFor(200); break;
    case 3:  servoTo(servo_x, gripNow, GRIP_CLOSED + GRIP_BACKOFF_DEG); sampleFor(100);
             servoTo(servo_y, armNow, ARM_CARRY); sampleFor(300);
             break;
    case 4:  servoTo(servo_y, armNow, ARM_DOWN); sampleFor(200); break;
    case 5:  servoTo(servo_x, gripNow, GRIP_RELEASE); sampleFor(200); break;
    case 6:  servoTo(servo_y, armNow, ARM_HIGH); sampleFor(200); break;
    case 7:  servoTo(servo_y, armNow, ARM_DOWN); servoTo(servo_x, gripNow, GRIP_OPEN);
             sampleFor(200);
             break;
    case 8:  servoJump(servo_x, gripNow, GRIP_CLOSED);
             servoJump(servo_x, gripNow, GRIP_OPEN);
             break;
    case 9:  digitalWrite(STBY, 1); spin(1); sampleFor(1000); break;
    case 10: spin(-1); sampleFor(1000); break;           /* no stop in between */
    case 11: spin(0); sampleFor(1000); break;
    case 12: spin(1);                                    /* the turn after a pick */
             servosTo(GRIP_CLOSED, ARM_CARRY);
             sampleUntil(t0, 1500);
             break;
    case 13: spin(-1);
             servosTo(GRIP_CLOSED, ARM_HIGH);
             sampleUntil(t0, 1000);
             break;
    default: spin(0);
             servosTo(GRIP_OPEN, ARM_DOWN);
             sampleFor(500);
             digitalWrite(STBY, 0);
             servo_x.detach(); servo_y.detach();
             sampleFor(500);
             break;
  }
}

/* =====================================================================
 *  THE SAVED RESULTS (EEPROM, the last 34 bytes)
 *  byte 0,1  0xE3 0x50 (a saved test is there)
 *  byte 2    the step running (0..14), DONE when the test finished,
 *            SEEN when an interrupted test was already reported
 *  byte 3    how many steps finished
 *  then per step: the lowest and the average, in 25 mV units (0xFF none)
 * ===================================================================== */
#define EE_BASE   (0x400 - 4 - 2 * N_STEPS)
#define EE_DONE   0xFE
#define EE_SEEN   0xFD

bool eeValid(){ return EEPROM.read(EE_BASE) == 0xE3 && EEPROM.read(EE_BASE + 1) == 0x50; }
uint8_t toEe(uint16_t mv){ uint16_t u = (mv + 12) / 25; return u > 254 ? 254 : (uint8_t)u; }
uint16_t fromEe(uint8_t b){ return (uint16_t)b * 25; }

void eeStart(){
  EEPROM.update(EE_BASE, 0xE3);
  EEPROM.update(EE_BASE + 1, 0x50);
  EEPROM.update(EE_BASE + 2, 0);
  EEPROM.update(EE_BASE + 3, 0);
  for(uint8_t k=0;k<2*N_STEPS;k++) EEPROM.update(EE_BASE + 4 + k, 0xFF);
}

/* ---- printing ---- */
void printMv(uint16_t mv){                 /* 4612 -> "4.61 V" */
  uint16_t c = (mv + 5) / 10;
  Serial.print((int)(c / 100));
  Serial.print('.');
  if(c % 100 < 10) Serial.print('0');
  Serial.print((int)(c % 100));
  Serial.print(F(" V"));
}

uint8_t gradeOf(uint16_t low){             /* 0 OK, 1 weak, 2 will restart */
  if(low >= V_OK_MV) return 0;
  if(low >= V_RESTART_MV) return 1;
  return 2;
}

void printGrade(uint8_t g){
  if(g == 0) Serial.print(F("OK"));
  else if(g == 1) Serial.print(F("WEAK"));
  else Serial.print(F("WILL RESTART"));
}

void printStepNo(uint8_t k){
  Serial.print(F("step "));
  if(k < 10) Serial.print(' ');
  Serial.print((int)k);
  Serial.print(F("  "));
}

/* the verdict and what it means; returns the overall grade.
 * g: from the readings; stopped: 0 the test finished, 1 the Nano restarted
 * (sure), 2 the test stopped in the middle (a restart, or switched off) */
uint8_t nEmpty = 0;                     /* steps with no readings at all */

uint8_t printVerdict(uint8_t g, uint8_t stopped, uint16_t rest){
  Serial.println();
  if(nEmpty && !stopped){
    Serial.println(F("VERDICT: CANNOT TELL: some steps had no readings (the 1.1 V input"));
    Serial.println(F("  gave impossible values). Is this an ATmega328P Nano?"));
    return 255;
  }
  Serial.print(F("VERDICT: "));
  printGrade(stopped ? 2 : g);
  Serial.println();
  if(stopped == 1){
    Serial.println(F("  The Nano RESTARTED during the test: for a moment the supply fell to"));
    Serial.println(F("  the reset level."));
  } else if(stopped == 2){
    Serial.println(F("  The test stopped in the middle: a restart, unless you switched it off."));
  }
  if(stopped && g < 2){
    Serial.println(F("  The readings before it looked better than that: the dip was too short"));
    Serial.println(F("  to catch. It is still the power. README: How to fix the power."));
  } else if(g == 0){
    Serial.println(F("  The supply held above 4.75 V in every step. The power is fine."));
  } else if(g == 1){
    Serial.println(F("  The supply dipped to 4.60..4.75 V. The real dips go lower than this"));
    Serial.println(F("  (the readings miss the shortest part): it may restart now and then,"));
    Serial.println(F("  and more often as the battery runs down. README: How to fix the power."));
  } else {
    Serial.println(F("  The supply dipped below 4.60 V, as in your real runs, where the Nano"));
    Serial.println(F("  restarted 12 times at a servo move or the turn after it. The real dips"));
    Serial.println(F("  reach the reset level. README: How to fix the power."));
  }
  if(rest && rest < V_REST_MV){
    Serial.print(F("  Standing still the supply is only "));
    printMv(rest);
    Serial.println(F(": a flat battery, or the battery switch is off (USB only)."));
  }
  Serial.println(F("  (OK: lowest >= 4.75 V; WEAK: 4.60 to 4.75 V; WILL RESTART: below 4.60 V"));
  Serial.println(F("  or a restart. The chip itself resets at 2.7 V, but each reading here is a"));
  Serial.println(F("  snapshot taken between the deepest spikes: README explains.)"));
  return stopped ? 2 : g;
}

/* the saved results; returns the overall grade (0..2), or 255 if none */
uint8_t printSaved(bool restartedNow){
  if(!eeValid()){
    Serial.println(F("No saved power test yet."));
    return 255;
  }
  uint8_t state = EEPROM.read(EE_BASE + 2);
  uint8_t nDone = EEPROM.read(EE_BASE + 3);
  if(nDone > N_STEPS) nDone = N_STEPS;
  Serial.println(F("=== SAVED POWER TEST (the last one, values rounded to 25 mV) ==="));
  uint8_t g = 0;
  nEmpty = 0;
  for(uint8_t k=0;k<nDone;k++){
    uint8_t lo = EEPROM.read(EE_BASE + 4 + 2 * k);
    uint8_t av = EEPROM.read(EE_BASE + 5 + 2 * k);
    printStepNo(k);
    if(lo == 0xFF){ Serial.println(F("no readings")); nEmpty++; continue; }
    Serial.print(F("lowest ")); printMv(fromEe(lo));
    Serial.print(F("  average ")); printMv(fromEe(av));
    Serial.print(F("  "));
    uint8_t s = gradeOf(fromEe(lo));
    printGrade(s);
    if(s > g) g = s;
    Serial.print(F("  "));
    Serial.println(stepName(k));
  }
  bool interrupted = (state != EE_DONE);
  if(interrupted){
    uint8_t k = (state == EE_SEEN) ? nDone : state;
    if(k < N_STEPS){
      printStepNo(k);
      Serial.print(F("STOPPED HERE: "));
      Serial.println(stepName(k));
    }
    if(restartedNow){
      Serial.println(F("  The Nano RESTARTED during this step (the memory mark survived)."));
    } else {
      Serial.println(F("  The test did not finish: the Nano restarted in this step, or it was"));
      Serial.println(F("  switched off. If you did not switch it off, it restarted."));
    }
  }
  uint16_t rest = 0;
  if(nDone > 0 && EEPROM.read(EE_BASE + 5) != 0xFF) rest = fromEe(EEPROM.read(EE_BASE + 5));
  g = printVerdict(g, interrupted ? (restartedNow ? 1 : 2) : 0, rest);
  Serial.println(F("=== END OF THE SAVED TEST ==="));
  return g;
}

/* LED: the verdict, over and over. 1 blink OK, 2 WEAK, 3 WILL RESTART */
uint8_t ledCode = 0;
void ledVerdict(){
  for(uint8_t k=0;k<=ledCode;k++){
    digitalWrite(LED_PIN, 1); delay(250);
    digitalWrite(LED_PIN, 0); delay(250);
  }
  delay(1200);
}

/* =====================================================================
 *  START
 * ===================================================================== */
void setup(){
  digitalWrite(STBY, 0);                /* motors off before anything else */
  pinMode(STBY, OUTPUT);
  pinMode(sp_L, OUTPUT); pinMode(F_L, OUTPUT); pinMode(B_L, OUTPUT);
  pinMode(sp_R, OUTPUT); pinMode(F_R, OUTPUT); pinMode(B_R, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  spin(0);
  Serial.begin(115200);
  adcSetup();

  Serial.println();
  Serial.println(F("=== E33 POWER TEST ==="));
  printResetCause();
  printBrownOutLevel();

  bool restarted = (runMark == RUN_MARK);
  runMark = 0;
  uint8_t state = eeValid() ? EEPROM.read(EE_BASE + 2) : EE_DONE;
  bool interrupted = (state < N_STEPS);

  if(interrupted){
    /* the last test stopped in the middle: report it and do NOT run again
     * (on the box it would only restart again). Reset button, or switch
     * off and on, to run the test again. */
    EEPROM.update(EE_BASE + 3, state);   /* (the steps that finished) */
    EEPROM.update(EE_BASE + 2, EE_SEEN);
    if(restarted) Serial.println(F("THE NANO RESTARTED DURING THE POWER TEST."));
    printSaved(restarted);
    Serial.println(F("Not running the test again now. Press reset (or switch off and on)"));
    Serial.println(F("to run it again. LED: 3 blinks = WILL RESTART."));
    ledCode = 2;
    return;
  }

  printSaved(false);
  Serial.println();
  Serial.println(F("Robot on a box, WHEELS IN THE AIR, servos free to move."));
  Serial.println(F("For the real result: battery switched on, USB cable OUT."));
  Serial.print(F("The test starts in "));
  Serial.print((int)COUNTDOWN_S);
  Serial.println(F(" s (LED blinks fast). Send any key to cancel."));
  for(uint16_t t=0;t<COUNTDOWN_S * 10;t++){
    digitalWrite(LED_PIN, (t & 1));
    delay(100);
    if(Serial.available()){
      while(Serial.available()) Serial.read();
      digitalWrite(LED_PIN, 0);
      Serial.println(F("Cancelled. Press reset to start again."));
      ledCode = 255;
      return;
    }
  }

  Serial.println(F("Each step: the lowest and the average supply, and how many readings."));
  eeStart();
  nEmpty = 0;
  runMark = RUN_MARK;
  digitalWrite(LED_PIN, 1);             /* LED on: the test is running */
  uint8_t g = 0;
  uint16_t rest = 0;
  for(uint8_t k=0;k<N_STEPS;k++){
    EEPROM.update(EE_BASE + 2, k);      /* "step k is running" (a restart keeps it) */
    accClear();
    runStep(k);
    uint16_t lo = accMin, av = accAvg();
    if(k == 0) rest = av;
    EEPROM.update(EE_BASE + 4 + 2 * k, accN ? toEe(lo) : 0xFF);
    EEPROM.update(EE_BASE + 5 + 2 * k, accN ? toEe(av) : 0xFF);
    EEPROM.update(EE_BASE + 3, k + 1);
    printStepNo(k);
    if(!accN){
      Serial.print(F("no readings (this chip does not give the 1.1 V)  "));
      nEmpty++;
    } else {
      Serial.print(F("lowest ")); printMv(lo);
      Serial.print(F("  average ")); printMv(av);
      Serial.print(F("  ")); Serial.print((unsigned long)accN);
      Serial.print(F(" readings, "));
      Serial.print((unsigned long)((accLow * 100UL + accN / 2) / accN));
      Serial.print(F("% below 4.60 V  "));
      uint8_t s = gradeOf(lo);
      printGrade(s);
      if(s > g) g = s;
    }
    Serial.println();
    Serial.print(F("         "));
    Serial.println(stepName(k));
  }
  EEPROM.update(EE_BASE + 2, EE_DONE);
  runMark = 0;
  digitalWrite(LED_PIN, 0);
  ledCode = printVerdict(g, 0, rest);
  Serial.println(F("Saved. LED: 1 blink = OK, 2 = WEAK, 3 = WILL RESTART."));
  Serial.println(F("To read it later with the USB cable: plug in, open the Serial Monitor."));
}

void loop(){
  if(ledCode <= 2) ledVerdict();
  else delay(100);                      /* cancelled, or no verdict: LED off */
}
