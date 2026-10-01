/* =====================================================================
 *  controlLibrary.h: the course library (robot04 to robot11)
 * =====================================================================
 *
 *  This keeps the slides' functions, names and shapes: beginFnc, getSensor,
 *  getErrorInput, followLine (pidFNC 0,1,0,0.7), checkGrid, countGrid,
 *  stopRobot, upSpeed, moveFor, turnRight90/Left90/Right180/Left180,
 *  keepup_object, put_object, arm_over_head.
 *
 *  Every place that differs from the slides is marked  [E33]  with the
 *  reason. Each reason is something that went wrong on the real robot.
 *
 *  You should not need to edit this file. Your numbers are in calibration.h.
 * ===================================================================== */
#ifndef CONTROLLIBRARY_H
#define CONTROLLIBRARY_H

#include <Servo.h>
#include <EEPROM.h>
#include "pidLibrary.h"
#if defined(__AVR__)
  #include <avr/io.h>
#endif

/* ------------------------------------------------ pins (robot05 p07) --- */
#define sp_L 6
#define F_L 12
#define B_L 11

#define sp_R 3
#define F_R 7
#define B_R 8
#define STBY 10

#define x_pin 5
#define y_pin 4
#define LED_PIN 13
Servo servo_x;
Servo servo_y;

int sensorPin[8] = {A0,A1,A2,A3,A4,A5,A6,A7};
#if ROBOT_MODE != MODE_CALIBRATE
String detectLine = "00000000";
#endif

int sp = SP_START;
int maxSp = MAX_SP;          /* [E33] slides: 255. You asked for a slower walk. */
unsigned long tUpSp = 0;

extern int numGride;          /* defined in robot_e33.ino */

/* What the route expects at each count number. caseInfo(n) is written in
 * robot_e33.ino, next to the switch, so the route is in one place. */
#define K_PASS  0             /* drive straight over it                 */
#define K_TURN  1             /* turn90 on a MID crossing               */
#define K_END   2             /* keep_item / place_item at a column end */
#define HALF_OK_L  0x04       /* at the field's edge the outside line may be too
                               * short to reach the bar: only the LEFT end of
                               * the bar has to light                    */
#define HALF_OK_R  0x08       /* ... only the RIGHT end has to light     */
#define AHEAD_MAY_BLANK 0x10  /* the line ahead is only the short overhang */
uint8_t caseInfo(int n);
uint8_t caseKind(int n){ return caseInfo(n) & 3; }

/* ------------------------------------------------------- prototypes ---- */
void beginFnc();
String getSensor();
int getErrorInput(String L);
void followLine();
bool checkGrid();
int countGrid(int n);
void stopRobot();
void upSpeed();
void moveFor();
void turnRight90();   //เลี้ยวขวา 90 องศา
void turnLeft90();    //เลี้ยวซ้าย 90 องศา
void turnRight180();  //เลี้ยวขวา 180 องศา
void turnLeft180();   //เลี้ยวซ้าย 180 องศา
void keepup_object();
void put_object();
void arm_over_head(); //ยกแขนสูง

void setMotor(int uL, int uR);
void fault(uint8_t code, const __FlashStringHelper *why);
void traceEvent(const __FlashStringHelper *what);

/* =====================================================================
 *  [E33]  CALIBRATION VALUES
 *  Loaded at start-up from EEPROM (written by the calibration mode), or
 *  from the CAL VALUES block in calibration.h. Loading never moves the
 *  robot.
 * ===================================================================== */
struct CalData {
  uint16_t lo[8];            /* white level per channel, black-high scale */
  uint16_t hi[8];            /* black level per channel                   */
  uint8_t  lineLow;          /* 1 = the bar reads black LOW               */
  uint8_t  deadMask;         /* channels too weak to use (bit7 = A0)      */
  uint8_t  dbLf, dbLr, dbRf, dbRr;   /* duty where each wheel starts      */
  uint16_t trimL, trimR;     /* per mille, forward; the weaker motor is 1000 */
  uint16_t trimLr, trimRr;   /* the same for backward (spins use one of each) */
  uint16_t vCruise100;       /* cm/s x100 at MAX_SP                       */
  uint16_t vMove100;         /* cm/s x100 at MOVE_SP                      */
  uint16_t lagMove;          /* ms lost when a move starts from rest      */
  uint16_t t90L, t90R;       /* ms per 90 degrees at TURN_SP              */
  uint16_t fromRun;          /* calibration run number, 0 = defaults      */
};

struct CalRec {              /* what is stored in EEPROM                  */
  uint16_t magic;            /* 0xE33C                                    */
  uint8_t  version;
  uint8_t  size;
  uint16_t seq;              /* newer record wins                         */
  uint16_t status;           /* 2 bits per item: 0 not run 1 pass 2 weak 3 fail */
  uint8_t  maxSp, moveSp, turnSp;   /* speeds the values were measured at */
  uint8_t  spare;
  CalData  d;
  uint16_t fletcher;
};

#define CAL_MAGIC     0xE33C
#define CAL_VERSION   3
#define CAL_SLOT_A    0x000
#define CAL_SLOT_B    0x080
#define CAL_REARM     0x100   /* 1 = calibration finished, lift to run again */
/* status item numbers */
#define CI_SENSOR  0
#define CI_DEADBD  1
#define CI_TRIM    2
#define CI_MOVE    3
#define CI_CRUISE  4
#define CI_TURN    5
#define CI_SPIN    6          /* backward trims, from the two spin directions */
#define CI_COUNT   7

#define ST_NONE 0
#define ST_PASS 1
#define ST_WEAK 2
#define ST_FAIL 3

CalData cal;
uint8_t calSource = 0;       /* 0 defaults, 1 EEPROM, 2 frozen in source  */
uint16_t calStatus = 0;
uint16_t calSeq = 0;

uint8_t calItem(uint16_t st, uint8_t k){ return (st >> (2*k)) & 3; }

uint16_t fletcher16(const uint8_t *p, uint8_t n){
  uint16_t a = 0, b = 0;
  while(n--){ a = (a + *p++) % 255; b = (b + a) % 255; }
  return (uint16_t)((b << 8) | a);
}

bool calSlotValid(const CalRec &r){
  return r.magic == CAL_MAGIC && r.version == CAL_VERSION && r.size == sizeof(CalRec)
      && r.fletcher == fletcher16((const uint8_t*)&r, sizeof(CalRec) - 2);
}

void calFromMacros(){
  const uint16_t w[8] = CAL_LO;
  const uint16_t b[8] = CAL_HI;
  cal.lineLow = CAL_LINE_LOW;
  for(uint8_t i=0;i<8;i++){
    /* the macros hold RAW readings; store them on the black-high scale */
    cal.lo[i] = cal.lineLow ? (uint16_t)(1023 - w[i]) : w[i];
    cal.hi[i] = cal.lineLow ? (uint16_t)(1023 - b[i]) : b[i];
  }
  cal.deadMask = CAL_DEAD_MASK;
  cal.dbLf = CAL_DB_LF;  cal.dbLr = CAL_DB_LR;
  cal.dbRf = CAL_DB_RF;  cal.dbRr = CAL_DB_RR;
  cal.trimL = CAL_TRIM_L; cal.trimR = CAL_TRIM_R;
  cal.trimLr = CAL_TRIM_LR; cal.trimRr = CAL_TRIM_RR;
  cal.vCruise100 = CAL_V_CRUISE_X100;
  cal.vMove100   = CAL_V_MOVE_X100;
  cal.lagMove    = CAL_LAG_MOVE_MS;
  cal.t90L = CAL_T90_L_MS;  cal.t90R = CAL_T90_R_MS;
  cal.fromRun = CAL_VALUES_FROM_RUN;
}

/* Copy one calibration item from an EEPROM record. */
void calCopyItem(CalData &to, const CalData &s, uint8_t item){
  switch(item){
    case CI_SENSOR: for(uint8_t i=0;i<8;i++){ to.lo[i]=s.lo[i]; to.hi[i]=s.hi[i]; }
                    to.lineLow = s.lineLow; to.deadMask = s.deadMask; break;
    case CI_DEADBD: to.dbLf=s.dbLf; to.dbLr=s.dbLr; to.dbRf=s.dbRf; to.dbRr=s.dbRr; break;
    case CI_TRIM:   to.trimL=s.trimL; to.trimR=s.trimR; break;
    case CI_MOVE:   to.vMove100=s.vMove100; to.lagMove=s.lagMove; break;
    case CI_CRUISE: to.vCruise100=s.vCruise100; break;
    case CI_TURN:   to.t90L=s.t90L; to.t90R=s.t90R; break;
    case CI_SPIN:   to.trimLr=s.trimLr; to.trimRr=s.trimRr; break;
  }
}

/* Read the newer valid of the two slots. Returns 0 if neither is valid,
 * otherwise the slot address + 1. */
int calReadEeprom(CalRec &out){
  CalRec a, b;
  EEPROM.get(CAL_SLOT_A, a);
  EEPROM.get(CAL_SLOT_B, b);
  bool va = calSlotValid(a), vb = calSlotValid(b);
  if(va && vb){
    if((int16_t)(a.seq - b.seq) > 0){ out = a; return CAL_SLOT_A + 1; }
    out = b; return CAL_SLOT_B + 1;
  }
  if(va){ out = a; return CAL_SLOT_A + 1; }
  if(vb){ out = b; return CAL_SLOT_B + 1; }
  return 0;
}

void calLoad(){
  calFromMacros();
  calSource = CAL_VALUES_FROM_RUN ? 2 : 0;
  calStatus = 0;
#if CAL_USE_EEPROM
  CalRec r;
  if(calReadEeprom(r)){
    for(uint8_t k=0;k<CI_COUNT;k++){
      uint8_t s = calItem(r.status, k);
      if(s == ST_PASS || s == ST_WEAK) calCopyItem(cal, r.d, k);   /* pass or weak: use it */
    }
    /* speeds were measured at the calibration's MAX_SP / MOVE_SP / TURN_SP.
     * If you have changed those since, scale them (the motor layer is close
     * to linear). Recalibrate when you can. */
    if(r.maxSp != MAX_SP && r.maxSp)
      cal.vCruise100 = (uint16_t)((uint32_t)cal.vCruise100 * MAX_SP / r.maxSp);
    if(r.moveSp != MOVE_SP && r.moveSp)
      cal.vMove100 = (uint16_t)((uint32_t)cal.vMove100 * MOVE_SP / r.moveSp);
    if(r.turnSp != TURN_SP && r.turnSp){
      cal.t90L = (uint16_t)((uint32_t)cal.t90L * r.turnSp / TURN_SP);
      cal.t90R = (uint16_t)((uint32_t)cal.t90R * r.turnSp / TURN_SP);
    }
    /* [E33] one of the two speeds measured and the other not: derive it from
     * the measured one (the motor layer is close to linear) instead of mixing a
     * measurement with the guess; the mix can make the robot "faster" at
     * MOVE_SP than at MAX_SP and every move before a turn wrong */
    bool mv = calItem(r.status, CI_MOVE) == ST_PASS || calItem(r.status, CI_MOVE) == ST_WEAK;
    bool cr = calItem(r.status, CI_CRUISE) == ST_PASS || calItem(r.status, CI_CRUISE) == ST_WEAK;
    if(mv && !cr) cal.vCruise100 = (uint16_t)((uint32_t)cal.vMove100 * MAX_SP / MOVE_SP);
    if(cr && !mv) cal.vMove100   = (uint16_t)((uint32_t)cal.vCruise100 * MOVE_SP / MAX_SP);
    calStatus = r.status;
    calSeq = r.seq;
    cal.fromRun = r.d.fromRun;
    calSource = 1;
  }
#endif
}

void printItemStatus(uint8_t s){
  if(s == ST_PASS)      Serial.print(F("PASS"));
  else if(s == ST_WEAK) Serial.print(F("WEAK"));
  else if(s == ST_FAIL) Serial.print(F("FAIL(default used)"));
  else                  Serial.print(F("default"));
}

void printItemName(uint8_t k){
  switch(k){
    case CI_SENSOR: Serial.print(F("sensors"));  break;
    case CI_DEADBD: Serial.print(F("deadband")); break;
    case CI_TRIM:   Serial.print(F("trim"));     break;
    case CI_MOVE:   Serial.print(F("move"));     break;
    case CI_CRUISE: Serial.print(F("cruise"));   break;
    case CI_TURN:   Serial.print(F("turn"));     break;
    case CI_SPIN:   Serial.print(F("spin"));     break;
  }
}

void printCalSource(){
  Serial.print(F("CAL: "));
  if(calSource == 1){
    Serial.print(F("EEPROM run #")); Serial.print(cal.fromRun);
    for(uint8_t k=0;k<CI_COUNT;k++){
      Serial.print(' '); printItemName(k); Serial.print('=');
      printItemStatus(calItem(calStatus, k));
    }
    Serial.println();
  }else if(calSource == 2){
    Serial.print(F("values pasted into calibration.h from run #"));
    Serial.println(cal.fromRun);
  }else{
    Serial.println(F("DEFAULTS - never calibrated. Run MODE_CALIBRATE once for good turns."));
  }
}

/* =====================================================================
 *  [E33]  WHY DID THE CHIP LAST RESET?
 *  A servo gripping an object can pull the 5 V rail down far enough to
 *  reset the Nano. With the slides' code that looks exactly like what you
 *  saw: the gripper opens (beginFnc writes the open angle), then the robot
 *  drives off as if it were at the start. The chip records a brown-out in
 *  MCUSR; it is read here before anything clears it.
 * ===================================================================== */
#if defined(__AVR__)
  uint8_t resetFlags __attribute__((section(".noinit")));
  uint8_t resetR2    __attribute__((section(".noinit")));
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
  /* [E33] Only Optiboot 6 and newer put MCUSR in r2 (its major version is the
   * last byte of the flash). The usual Nano bootloader, Optiboot 4.4, clears
   * MCUSR and leaves r2 as the program left it, for example the crossing count:
   * reading that as reset flags could stop the mission as a false brown-out. */
  bool r2HasFlags(){ uint8_t v = pgm_read_byte(FLASHEND); return v >= 6 && v != 0xFF; }
#else
  bool r2HasFlags(){ return false; }
  uint8_t resetFlags = 0x01, resetR2 = 0;           /* host simulator: power-on */
  #define RF_POR 0x01
  #define RF_EXT 0x02
  #define RF_BOR 0x04
  #define RF_WDT 0x08
#endif

/* [E33] The usual Nano bootloader clears the reset flags, so a brown-out is
 * also recognised another way: while the mission runs, this word in memory
 * that survives a reset (but not switching off) holds a mark. Finding the
 * mark at start-up means the Nano restarted in the middle of a run. */
#if defined(__AVR__)
  uint32_t runMark __attribute__((section(".noinit")));
#else
  uint32_t runMark = 0;
#endif
#define RUN_MARK 0xE33A55C3UL

uint8_t resetCause(){
  uint8_t f = resetFlags & 0x0F;
  if(f == 0 && r2HasFlags() && (resetR2 & 0xF0) == 0) f = resetR2;
  return f;
}
bool brownOutReset(){
  uint8_t f = resetCause();
  return (f & RF_BOR) && !(f & RF_POR);
}
void printResetCause(){
  uint8_t f = resetCause();
  Serial.print(F("reset cause: "));
  if(f & RF_POR) Serial.print(F("power-on "));
  if(f & RF_EXT) Serial.print(F("reset-button/USB "));
  if(f & RF_BOR) Serial.print(F("BROWN-OUT "));
  if(f & RF_WDT) Serial.print(F("watchdog "));
  if(f == 0)     Serial.print(F("unknown (the bootloader cleared it)"));
  Serial.println();
}

/* =====================================================================
 *  [E33]  MOTOR LAYER
 *  Every motor command goes through setMotor(). It takes the slides' own
 *  speed units (0..255, negative = backwards) and turns them into each
 *  motor's duty: it skips that motor's dead band and scales the stronger
 *  motor down to match the weaker one. So "speed 50" really moves both
 *  wheels, and the same number turns both wheels at the same speed.
 *
 *  Your left wheel turned about 270 degrees while the right turned 360.
 *  Without this the robot curves on every straight move, the PID holds the
 *  line off-centre, right turns come out short and left turns long.
 * ===================================================================== */
int curUL = 0, curUR = 0;             /* last command, for the odometer */

uint8_t toDuty(int u, uint8_t db, uint16_t trim){
  if(u <= 0) return 0;
  if(u > 255) u = 255;
  uint32_t span = (uint32_t)(255 - db) * trim;               /* per mille */
  uint32_t d = db + (span * (uint32_t)u + 127500UL) / 255000UL;
  return d > 255 ? 255 : (uint8_t)d;
}

/* The slides' motor lines, for one wheel. side 0 = left, 1 = right. */
void motorRaw(uint8_t side, int8_t dir, uint8_t duty){
  if(side == 0 && LEFT_REVERSED)  dir = -dir;
  if(side == 1 && RIGHT_REVERSED) dir = -dir;
  uint8_t pf = side ? F_R : F_L;
  uint8_t pb = side ? B_R : B_L;
  uint8_t pp = side ? sp_R : sp_L;
  if(dir > 0)      { digitalWrite(pf,1); digitalWrite(pb,0); analogWrite(pp,duty); }
  else if(dir < 0) { digitalWrite(pf,0); digitalWrite(pb,1); analogWrite(pp,duty); }
  else             { digitalWrite(pf,1); digitalWrite(pb,1); analogWrite(pp,0); }   /* brake */
}

void odoTick();

void setMotor(int uL, int uR){
  odoTick();                                   /* close the odometer on the old command */
  curUL = uL; curUR = uR;
  if(uL > 0)      motorRaw(0,  1, toDuty( uL, cal.dbLf, cal.trimL));
  else if(uL < 0) motorRaw(0, -1, toDuty(-uL, cal.dbLr, cal.trimLr));
  else            motorRaw(0,  0, 0);
  if(uR > 0)      motorRaw(1,  1, toDuty( uR, cal.dbRf, cal.trimR));
  else if(uR < 0) motorRaw(1, -1, toDuty(-uR, cal.dbRr, cal.trimRr));
  else            motorRaw(1,  0, 0);
}

/* =====================================================================
 *  [E33]  ODOMETER (no encoders, so it adds up commanded speed x time)
 *  Used to check that each crossing arrives at a sensible distance, and to
 *  put the wheels on a crossing before a turn. vScale follows the battery:
 *  every time the robot drives one whole cell between two crossings, it
 *  compares the odometer with CELL_CM and corrects itself.
 * ===================================================================== */
float odoCm = 0;
float odoU = 0;                        /* the same in speed units x seconds (calibration) */
float odoEffL = 0, odoEffR = 0;        /* speed given to each wheel x seconds (calibration) */
unsigned long odoLastUs = 0;
float vScale = 1.0;
uint8_t vSamples = 0;                  /* cells measured during this run */

#define BRAKE_COAST_MS  40             /* guess: how long the robot rolls after a brake */

float speedCms(int u){
  float a = (u < 0) ? -u : u;
  float vm = cal.vMove100 / 100.0;
  float vc = cal.vCruise100 / 100.0;
  float v;
  if(a <= MOVE_SP) v = vm * a / MOVE_SP;
#if MAX_SP > MOVE_SP
  else v = vm + (vc - vm) * (a - MOVE_SP) / (float)(MAX_SP - MOVE_SP);
#else
  else v = vm * a / MOVE_SP;
#endif
  if(v < 0) v = 0;
  return v * vScale;
}

void odoTick(){
  unsigned long now = micros();
  unsigned long dt = now - odoLastUs;
  odoLastUs = now;
  if(dt > 200000UL) return;                    /* first call, or a long pause */
  float u2 = (curUL + curUR) * 0.5;
  float s = (float)dt * 1e-6;
  odoU += u2 * s;
  odoEffL += curUL * s;
  odoEffR += curUR * s;
  int u = (int)u2;
  float d = speedCms(u) * s;
  odoCm += (u >= 0) ? d : -d;
}


/* =====================================================================
 *  [E33]  SENSOR LAYER
 *  Each channel gets its own threshold, placed between the white and black
 *  that THIS channel reads (SENS_ON_PCT / SENS_OFF_PCT of its own contrast).
 *  The slides use one number for all eight (800 on robot04 p12, 500 on
 *  robot11 p06). If that number is above what your bar reads over black,
 *  the robot sees nothing and does not move, which is what happened with
 *  800.
 * ===================================================================== */
uint16_t onLvl[8], offLvl[8];
uint16_t rawV[8];                      /* last readings, black-high scale  */
uint8_t  lineMask = 0;                 /* bit7 = A0 (left) ... bit0 = A7   */
int8_t   lastLineErr = 0;              /* where the line was last seen     */
unsigned long tLineSeen = 0;
float    odoSeen = 0;                  /* odometer when a line was last seen */

uint8_t popcount8(uint8_t m){ uint8_t c = 0; while(m){ c += m & 1; m >>= 1; } return c; }

/* true if the lit sensors are side by side (one line, not two spots) */
bool isRun(uint8_t m){
  if(m == 0) return false;
  while(!(m & 1)) m >>= 1;
  return (m & (m + 1)) == 0;
}

void calApply(){
  for(uint8_t i=0;i<8;i++){
    uint16_t lo = cal.lo[i], hi = cal.hi[i];
    if(hi <= lo + 20){ onLvl[i] = 520; offLvl[i] = 480; }   /* no contrast: slides' 500 */
    else{
      uint16_t c = hi - lo;
      onLvl[i]  = lo + (uint32_t)c * SENS_ON_PCT  / 100;
      offLvl[i] = lo + (uint32_t)c * SENS_OFF_PCT / 100;
    }
  }
}

/* [E33] Each channel is read twice and the first reading thrown away. The
 * ADC's input needs a moment after switching from the previous channel; read
 * at once, a channel shows a little of its neighbour, which blurs the edges
 * of a line, worst while a line sweeps across the bar during a turn. The
 * ADC clock is 4 times faster (beginFnc()), so a scan of two reads per
 * channel still takes only half the time of the slides' one read. */
uint16_t readNorm(uint8_t i){
  analogRead(sensorPin[i]);
  uint16_t v = analogRead(sensorPin[i]);
  return cal.lineLow ? (uint16_t)(1023 - v) : v;
}

/* The rule the slides' error table is built on: sensor i sits at 2i-7
 * (A0 = -7 ... A7 = +7) and the error is the average position of the lit
 * sensors. All 15 entries in the table follow it. */
int errorOfMask(uint8_t m){
  int sum = 0, n = 0;
  for(uint8_t i=0;i<8;i++) if(m & (0x80 >> i)){ sum += 2*i - 7; n++; }
  if(n == 0) return 100;
  return (sum >= 0) ? (2*sum + n) / (2*n) : -((-2*sum + n) / (2*n));
}

/* Read all eight channels with their own thresholds and hysteresis. */
uint8_t scanBar(){
  odoTick();
  uint8_t m = 0;
  for(uint8_t i=0;i<8;i++){
    uint16_t v = readNorm(i);
    rawV[i] = v;
    bool was = lineMask & (0x80 >> i);
    bool on  = was ? (v >= offLvl[i]) : (v >= onLvl[i]);
    if(cal.deadMask & (0x80 >> i)) on = false;
    if(on) m |= (0x80 >> i);
  }
  lineMask = m;
  if(m){
    tLineSeen = millis();
    odoSeen = odoCm;
    if(popcount8(m) <= 3 && isRun(m)) lastLineErr = errorOfMask(m);
  }
  return m;
}

/* How black channel i is, 0 (white) .. 255 (black), from the last scan. */
uint8_t blackness(uint8_t i){
  uint16_t lo = cal.lo[i], hi = cal.hi[i], v = rawV[i];
  if(cal.deadMask & (0x80 >> i)) return 0;
  if(hi <= lo + 20) return v >= 500 ? 255 : 0;
  if(v <= lo) return 0;
  if(v >= hi) return 255;
  return (uint8_t)((uint32_t)(v - lo) * 255 / (hi - lo));
}

/* Where the line is under the bar, in 1/256 of a sensor step, from the
 * analog readings (0 = A0 ... 1792 = A7, 896 = the middle). -1 if there is
 * no single line (nothing, a crossing, or two separate spots). Used by the
 * calibration and the meter; the mission steers with the slides' table. */
int centroid256(){
  uint8_t n[8], best = 0;
  for(uint8_t i=0;i<8;i++){ n[i] = blackness(i); if(n[i] > n[best]) best = i; }
  if(n[best] < 96) return -1;
  int8_t a = best, b = best;
  while(a > 0 && n[a-1] > 64) a--;
  while(b < 7 && n[b+1] > 64) b++;
  if(b - a + 1 > 4) return -1;
  for(int8_t i=0;i<8;i++) if((i < a-1 || i > b+1) && n[i] > 96) return -1;
  if(a > 0) a--;
  if(b < 7) b++;
  uint32_t s = 0, w = 0;
  for(int8_t i=a;i<=b;i++){ s += (uint32_t)n[i] * i * 256; w += n[i]; }
  if(w == 0) return -1;
  return (int)(s / w);
}

/* =====================================================================
 *  [E33]  STANDING STILL: IS THERE ONE LINE UNDER THE BAR?
 *  Reads the bar without moving and splits the eight readings into two
 *  groups at the biggest jump between them: the few channels on the line
 *  and the rest on white. Up to two odd channels (for example one stuck
 *  reading black) are allowed; they are marked and not used.
 * ===================================================================== */
struct RawLook { uint8_t lineLow, mask, odd; uint16_t white, black, gap; uint16_t v[8]; };

bool rawLook(RawLook &r){
  uint8_t idx[8];
  for(uint8_t i=0;i<8;i++){
    uint32_t sum = 0;
    for(uint8_t k=0;k<8;k++) sum += analogRead(sensorPin[i]);
    r.v[i] = sum / 8;
    idx[i] = i;
  }
  for(uint8_t a=1;a<8;a++){                       /* sort by reading */
    uint8_t x = idx[a]; int8_t b = a - 1;
    while(b >= 0 && r.v[idx[b]] > r.v[x]){ idx[b+1] = idx[b]; b--; }
    idx[b+1] = x;
  }
  uint16_t g = 0; uint8_t at = 0;                 /* biggest jump = white vs line */
  for(uint8_t a=0;a<7;a++){
    uint16_t d = r.v[idx[a+1]] - r.v[idx[a]];
    if(d > g){ g = d; at = a; }
  }
  r.gap = g;
  if(g < 120) return false;
  uint8_t nLow = at + 1, nHigh = 7 - at;
  if(nLow == nHigh) return false;
  bool lineHigh = nHigh < nLow;                   /* the few channels are the line */
  uint8_t grp = 0;
  for(uint8_t a=0;a<8;a++) if(lineHigh ? (a > at) : (a <= at)) grp |= (0x80 >> idx[a]);
  /* the line itself: the group of side-by-side channels nearest the middle */
  uint8_t best = 0; int bestE = 100;
  for(uint8_t i=0;i<8;){
    if(grp & (0x80 >> i)){
      uint8_t run = 0;
      while(i < 8 && (grp & (0x80 >> i))){ run |= (0x80 >> i); i++; }
      int e = abs(errorOfMask(run));
      if(e < bestE){ bestE = e; best = run; }
    }else i++;
  }
  if(!best || popcount8(best) > 3 || bestE > 5) return false;
  r.odd = grp & ~best;
  if(popcount8(r.odd) > 2) return false;
  r.mask = best;
  r.lineLow = lineHigh ? 0 : 1;
  uint32_t sw = 0, sb = 0; uint8_t nw = 0, nb = 0;
  for(uint8_t i=0;i<8;i++){
    uint8_t bit = 0x80 >> i;
    if(best & bit){ sb += r.v[i]; nb++; }
    else if(!(grp & bit)){ sw += r.v[i]; nw++; }
  }
  uint16_t w = sw / nw, b = sb / nb;
  r.white = lineHigh ? w : (uint16_t)(1023 - w);  /* black-high scale */
  r.black = lineHigh ? b : (uint16_t)(1023 - b);
  return true;
}

/* Starting levels for each channel from one standing-still look: its own
 * reading is its white (or its black, for the channels on the line), and
 * the jump between the two groups gives the other end. */
void calFromLook(const RawLook &r){
  cal.lineLow = r.lineLow;
  uint16_t c = r.black - r.white;
  for(uint8_t i=0;i<8;i++){
    uint16_t v = r.lineLow ? (uint16_t)(1023 - r.v[i]) : r.v[i];
    if(r.mask & (0x80 >> i)){ cal.hi[i] = v; cal.lo[i] = (v > c) ? v - c : 0; }
    else                    { cal.lo[i] = v; cal.hi[i] = v + c; }
  }
  cal.deadMask = r.odd;
  calApply();
}

String getSensor(){
  String x = "";
  uint8_t m = scanBar();               /* [E33] per-channel thresholds */
  for(int i=0;i<8;i++){
    if(m & (0x80 >> i)){
      x += "1";
    }else{
      x += "0";
    }
  }
  return(x);
}

uint8_t maskOf(String L){
  uint8_t m = 0;
  for(uint8_t k=0;k<8;k++){ m <<= 1; if(L.charAt(k) == '1') m |= 1; }
  return m;
}

/* [E33] Anything the table does not list. A crossing (4 or more lit) keeps
 * the last line position so the robot drives straight over it. Several
 * separate spots (noise, a clipped branch): take the one nearest to where
 * the line was. */
int errorFallback(uint8_t m){
  if(popcount8(m) >= 4) return lastLineErr;
  int best = 100, bestD = 100;
  uint8_t i = 0;
  while(i < 8){
    if(m & (0x80 >> i)){
      uint8_t run = 0;
      while(i < 8 && (m & (0x80 >> i))){ run |= (0x80 >> i); i++; }
      int c = errorOfMask(run);
      int d = abs(c - lastLineErr);
      if(d < bestD){ bestD = d; best = c; }
    }else{
      i++;
    }
  }
  return best;
}

int getErrorInput(String L){
  int e;
  if(L == "10000000"){e = -7;}
  else if(L == "11000000"){e = -6;}
  else if(L == "11100000"){e = -5;}
  else if(L == "01100000"){e = -4;}
  else if(L == "01110000"){e = -3;}
  else if(L == "00110000"){e = -2;}
  else if(L == "00111000"){e = -1;}
  else if(L == "00011000"){e = 0;}
  else if(L == "00011100"){e = 1;}
  else if(L == "00001100"){e = 2;}
  else if(L == "00001110"){e = 3;}
  else if(L == "00000110"){e = 4;}
  else if(L == "00000111"){e = 5;}
  else if(L == "00000011"){e = 6;}
  else if(L == "00000001"){e = 7;}
  /* [E33] the six single-sensor patterns the table leaves out. Same rule as
   * the rows above. Without them the robot got 100 and stopped steering. */
  else if(L == "01000000"){e = -5;}
  else if(L == "00100000"){e = -3;}
  else if(L == "00010000"){e = -1;}
  else if(L == "00001000"){e = 1;}
  else if(L == "00000100"){e = 3;}
  else if(L == "00000010"){e = 5;}
  else if(L == "00000000"){e = 100;}               /* nothing lit: line lost */
  else{e = errorFallback(maskOf(L));}              /* [E33] was: e = 100    */
  return(e);
}

/* [E33] The same numbers as getErrorInput(), straight from the sensor bits:
 * every row of the table is one line of 1 to 3 lit sensors side by side,
 * and its number is errorOfMask(). Used in MODE_CALIBRATE, where the Nano's
 * flash is too full for the String code. */
int errorInputOf(uint8_t m){
  if(m == 0) return 100;
  if(popcount8(m) <= 3 && isRun(m)) return errorOfMask(m);
  return errorFallback(m);
}

/* =====================================================================
 *  LINE FOLLOWING  (robot06 p12, robot08 p04, robot11 p08)
 * ===================================================================== */
unsigned long tLost = 0;               /* when the line was lost, 0 = not lost */

#define LOST_HOLD_MS    200

/* [E33] The slides write nothing when the line is lost, so the motors keep
 * their last command for ever. Instead: keep the last steering slowly for a
 * moment (the line is often just under a gap between sensors), then swing
 * the bar about 27 degrees toward where the line was last seen, then the
 * same the other way, then back. Only then stop. */
void lineLost(){
  unsigned long now = millis();
  if(tLost == 0) tLost = now;
  unsigned long t = now - tLost;
  int e = lastLineErr;
  if(t < LOST_HOLD_MS){
    int s = sp < MOVE_SP ? sp : MOVE_SP;
    int speedL = s + (s*e)/7;
    int speedR = s - (s*e)/7;
    if(speedL > s) speedL = s;
    if(speedL < 0) speedL = 0;
    if(speedR > s) speedR = s;
    if(speedR < 0) speedR = 0;
    setMotor(speedL, speedR);
    return;
  }
  int d = (e < 0) ? -1 : 1;                    /* line last seen on the right: look right first */
  unsigned long ts = t - LOST_HOLD_MS;
  unsigned long a = (unsigned long)(cal.t90L + cal.t90R) * 15 / 100;   /* ~27 degrees */
  if(ts < a)          setMotor( d * TURN_SLOW_SP, -d * TURN_SLOW_SP);
  else if(ts < 3 * a) setMotor(-d * TURN_SLOW_SP,  d * TURN_SLOW_SP);
  else if(ts < 4 * a) setMotor( d * TURN_SLOW_SP, -d * TURN_SLOW_SP);
  else fault(1, F("line lost"));
}

void followLine(){
#if ROBOT_MODE == MODE_CALIBRATE
  int errorInput = errorInputOf(scanBar());      /* [E33] same numbers, less flash */
#else
  detectLine = getSensor();
  int errorInput = getErrorInput(detectLine);
#endif
  if(errorInput != 100){
    if(tLost){ tLost = 0; clearPid(); }          /* [E33] fresh D term */
    float pidOut = pidFNC(errorInput,0,1,0,0.7);
    upSpeed();

    int spOut = map(round(pidOut),-7,7,-sp,sp);
    int speedL = sp - spOut;
    int speedR = sp + spOut;
    if(speedL>sp){speedL=sp;}
    if(speedL<0){speedL=0;}
    if(speedR>sp){speedR=sp;}
    if(speedR<0){speedR=0;}
    setMotor(speedL, speedR);                    /* [E33] was the two motor lines */
    //Serial.println(String(speedL) + "," + String(speedR));
  }else{
    lineLost();                                  /* [E33] */
  }
}

/* [E33] One step of the same PID steering at a fixed speed s, without the
 * ramp. Used for the last few cm before a turn. Over a crossing or with no
 * line it drives straight. */
void steerAt(int s){
#if ROBOT_MODE == MODE_CALIBRATE
  int errorInput = errorInputOf(scanBar());
#else
  detectLine = getSensor();
  int errorInput = getErrorInput(detectLine);
#endif
  if(errorInput == 100 || popcount8(lineMask) >= 4){ setMotor(s, s); return; }
  float pidOut = pidFNC(errorInput,0,1,0,0.7);
  int spOut = map(round(pidOut),-7,7,-s,s);
  int speedL = s - spOut;
  int speedR = s + spOut;
  if(speedL>s){speedL=s;}
  if(speedL<0){speedL=0;}
  if(speedR>s){speedR=s;}
  if(speedR<0){speedR=0;}
  setMotor(speedL, speedR);
}

/* =====================================================================
 *  COUNTING CROSSINGS  (robot07 p09 - p11, robot11 p09 - p10)
 *
 *  [E33] countGrid() still returns n+1 at every crossing, and the switch in
 *  loop() still acts on the number. What is new is that each crossing is
 *  CHECKED before it is counted:
 *    - it must arrive at a sensible distance (not a double count),
 *    - it must light both ends of the bar (not a branch seen from the side),
 *    - the robot looks just past it: is there line straight ahead?
 *      A MID crossing has one, the end of a column does not.
 *  One line is printed for every crossing, so a failed run can be read.
 * ===================================================================== */
#define AH_UNSURE 0
#define AH_LINE   1
#define AH_BLANK  2
#define AH_WIDE   3
#define AH_LOST   4           /* the line ran out without a crossing */

#define CROSS_MAX_MS   1000   /* on a "crossing" longer than this = driving along a line */
#define MIN_GAP_FRAC   0.45   /* a crossing closer than this share of the expected
                               * distance is a double count or a smudge        */
#define APPROACH_CM    8.0    /* slow to MOVE_SP this far before a turn or an end */
#define SETTLE_MS      150

unsigned long tEdge = 0;      /* when the bar left the last counted crossing */
float    edgeOdo = 0;         /* odometer at that moment                     */
uint8_t  crossMask  = 0;      /* every sensor that lit while on it           */
uint16_t crossDwell = 0;      /* ms the bar spent on it                      */
uint8_t  crossAhead = AH_UNSURE;
float    crossDist  = 0;      /* odometer distance from the leg start         */
float    crossGap   = CELL_CM; /* how far it was expected to be              */
float    legRef = 0, legGap = CELL_CM;   /* where this leg started, and how far
                                          * the next crossing should be        */
bool     legRolling = false;  /* leg started at a crossing passed at speed   */
float    prevLegRef = 0, prevLegGap = CELL_CM;
bool     prevLegRolling = false;
bool     gridArmed = true;
uint8_t  nRejected = 0, nCredited = 0, nRetried = 0;

bool checkGrid(){
  uint8_t m = scanBar();           /* [E33] slides: String ch = getSensor(); */
  uint8_t n = popcount8(m);
  /* [E33] the slides list 9 exact patterns. One weak or dirty sensor turns
   * 11111111 into 11110111 and the crossing is not counted, which shifts
   * every later step of the route. Counting lit sensors accepts all 9 slide
   * patterns and still works with two dead channels. */
  if(n >= 5) return (true);
  if(n == 4 && ((m & 0xF0) == 0xF0 || (m & 0x0F) == 0x0F)) return (true);
  return (false);
}

/* Is there a narrow line near the middle of the bar? (Other spots, such as
 * the tail of a crossing taken at a slant, are ignored.) */
bool lineNearMiddle(uint8_t m){
  uint8_t i = 0;
  while(i < 8){
    if(m & (0x80 >> i)){
      uint8_t run = 0, len = 0;
      while(i < 8 && (m & (0x80 >> i))){ run |= (0x80 >> i); i++; len++; }
      if(len <= 3 && abs(errorOfMask(run)) <= 5) return true;
    }else{
      i++;
    }
  }
  return false;
}

/* Drive straight past the tape and look: line, blank, or something wide?
 * The first skipCm also finishes the crossing: a crossing taken at a slant
 * reaches one end of the bar before the other, so the mask keeps growing. */
uint8_t lookAhead(float skipCm, float lookCm){
  int s = sp;
  float o0 = odoCm;
  unsigned long t0 = millis();
  /* straight while the tape of the crossing is still under the bar (its
   * tail can look like a line at one end of the bar) ... */
  setMotor(s, s);
  while(odoCm - o0 < skipCm && millis() - t0 < 300){ crossMask |= scanBar(); }
  uint8_t nLine = 0, nBlank = 0, nWide = 0, nAll = 0;
  o0 = odoCm; t0 = millis();
  while((odoCm - o0 < lookCm || nAll < 12) && millis() - t0 < 400 && nAll < 250){
    uint8_t m = scanBar();
    /* ... then keep steering, but only on a narrow line near the middle: with
     * unmatched motors a few cm of blind driving can lose the line */
    if(popcount8(m) <= 3 && lineNearMiddle(m)){
      /* [E33] PID first, then round: Arduino's round() is a macro that would
       * call pidFNC twice, and the second call has no D term */
      float pidOut = pidFNC(errorOfMask(m),0,1,0,0.7);
      int spOut = map(round(pidOut),-7,7,-s,s);
      int speedL = s - spOut, speedR = s + spOut;
      if(speedL > s) speedL = s;
      if(speedL < 0) speedL = 0;
      if(speedR > s) speedR = s;
      if(speedR < 0) speedR = 0;
      setMotor(speedL, speedR);
    }else{
      setMotor(s, s);
    }
    if(popcount8(m) >= 5) nWide++;
    else if(lineNearMiddle(m)) nLine++;
    else if((m & 0x7E) == 0) nBlank++;       /* all but the two end sensors see white */
    nAll++;
  }
  if((uint16_t)nLine*10  >= (uint16_t)nAll*6) return AH_LINE;
  if((uint16_t)nBlank*10 >= (uint16_t)nAll*7){
    /* a line right under a dead middle sensor also looks blank: then say unsure */
    return (cal.deadMask & 0x3C) ? AH_UNSURE : AH_BLANK;
  }
  if((uint16_t)nWide*2   >= (uint16_t)nAll)   return AH_WIDE;
  return AH_UNSURE;
}

void setLeg(float gapCm){
  legRef = odoCm;
  legGap = gapCm;
  legRolling = false;
  maxSp = MAX_SP;
}

/* Undo the last count: measure the next crossing from the old start again. */
void restoreLeg(){
  legRef = prevLegRef;
  legGap = prevLegGap;
  legRolling = prevLegRolling;
}

int countGrid(int n){
  uint8_t next = caseKind(n+1);
  float dNow = odoCm - legRef;

  /* [E33] slow down before a turn or an end, so the robot crosses, looks
   * and stops at a known speed */
  if(next != K_PASS && dNow > legGap - APPROACH_CM){
    maxSp = MOVE_SP;
    if(sp > maxSp) sp = maxSp;
  }else{
    maxSp = MAX_SP;
  }

  /* [E33] ran off the end of a column without seeing the T: that WAS the end */
  /* (only where the T should be: a line lost earlier is a bump or a drift,
   * and lineLost() looks for it; at 0.6 a knock 6 cm before the T was taken
   * for the end and the gripper closed on nothing) */
  if(next == K_END && tLost && millis() - tLost >= 40 && odoSeen - legRef >= 0.8 * legGap){
    prevLegRef = legRef; prevLegGap = legGap; prevLegRolling = legRolling;
    crossDist = odoSeen - legRef;
    crossGap = legGap;
    crossMask = 0; crossDwell = 0; crossAhead = AH_LOST;
    edgeOdo = odoSeen; tEdge = tLineSeen;
    legRef = odoSeen; legGap = CELL_CM; legRolling = false;
    tLost = 0;
    n ++;
    traceEvent(F("END found by the line running out"));
    return (n);
  }

  /* [E33] do not count a crossing the robot is already standing on */
  if(!gridArmed){
    if(!checkGrid()) gridArmed = true;
    /* [E33] still "on" it a whole cell later: the bar reads black everywhere */
    else if(odoCm - legRef > legGap + CELL_CM / 2) fault(2, F("the bar sees black everywhere for too long: sensor levels?"));
    return (n);
  }
  if(checkGrid()){
    delay(10);
    if(checkGrid()){
      unsigned long tIn = millis();
      crossMask = lineMask;
      setMotor(sp, sp);                  /* [E33] ride straight over it          */
      uint8_t clear = 0;
      while(clear < 2){                  /* slides: while(checkGrid());           */
        if(checkGrid()) clear = 0; else clear++;
        crossMask |= lineMask;
        if(millis() - tIn > CROSS_MAX_MS) fault(2, F("on a crossing for too long: driving along a line"));
      }
      tEdge = millis();
      edgeOdo = odoCm;
      crossDwell = (uint16_t)(tEdge - tIn);
      crossDist  = edgeOdo - legRef;
      crossGap   = legGap;
      /* look past it: a short look at an end, so the jaws do not push the object */
      if(next == K_END) crossAhead = lookAhead(0.5, 0.8);
      else              crossAhead = lookAhead(1.0, 1.5);

      /* [E33] The odometer only knows the speed the calibration measured, and
       * a 9 V battery is weaker later in the day. So the distance checks wait
       * until THIS run has measured its own speed: the first crossing sets it
       * (its distance from the start is known), and every crossing passed at
       * speed one cell after the last corrects it. Until then nothing is
       * judged "missed" (a slow robot would look like one that missed a line). */
      bool speedKnown = vSamples > 0;

      /* [E33] CHECK 1: far too early -> a double count or a smudge */
      if(crossDist < legGap * (speedKnown ? MIN_GAP_FRAC : 0.3)){
        /* [E33] nothing counted yet: this is C1 (the robot was put down with
         * the bar just before it) or a mark near the start. The distance from
         * the start to C2 is then unknown: legGap 0 = do not learn the speed
         * at C2, learn it over the first whole cell */
        if(n == 0) legGap = 0;
        nRejected ++;
        traceEvent(F("not counted: too early"));
        return (n);
      }
      /* [E33] CHECK 3: about one cell too far -> a crossing was missed */
      if(speedKnown && crossDist > legGap + 0.5 * CELL_CM){
        if(next == K_PASS){
          n ++;                          /* count the one that was missed */
          nCredited ++;
          next = caseKind(n+1);
          traceEvent(F("a crossing was missed: counted it"));
        }else{
          traceEvent(F("missed"));
          fault(4, F("missed a turn or an end: the robot is past it"));
        }
      }
      /* [E33] speed check (the battery): correct the odometer */
      float ratio = legGap / crossDist;          /* > 1: the robot is faster than it thought */
      if(!speedKnown){
        if(ratio > 0.4 && ratio < 2.5){ vScale *= ratio; vSamples = 1; }
      }else if(legRolling && legGap == CELL_CM && vSamples == 1 && ratio > 0.67 && ratio < 1.5){
        /* [E33] the first leg is only as exact as where the robot was put
         * down; the first whole cell is exact: take all of it (below 0.67 a
         * crossing was missed) */
        vScale *= ratio;
        vSamples ++;
      }else if(legRolling && legGap == CELL_CM && ratio > 0.7 && ratio < 1.43){
        vScale *= sqrt(ratio);
        if(vSamples < 255) vSamples ++;
      }
      if(vScale > 2.5) vScale = 2.5;
      if(vScale < 0.4) vScale = 0.4;
      /* [E33] CHECK 2: a real crossing lights BOTH ends of the bar. Only one
       * end lit means the robot is driving ALONG a TOP or BOT line past a
       * column: it is on the wrong line (this is how the robot ended up
       * following the TOP line after a bad 180). The one exception: at the
       * field's edge the outside line (the overhang) may be too short to
       * reach the bar, so only the end on the field side lights. */
      uint8_t info = caseInfo(n+1);
      bool leftEnd = crossMask & 0xE0, rightEnd = crossMask & 0x07;
      if(!(leftEnd && rightEnd)){
        bool ok = (leftEnd && (info & HALF_OK_L)) || (rightEnd && (info & HALF_OK_R));
        if(!ok){
          numGride = n + 1;
          traceEvent(F("half crossing"));
          fault(3, F("half crossing: the robot is on the wrong line"));
        }
      }
      prevLegRef = legRef; prevLegGap = legGap; prevLegRolling = legRolling;
      legRef = edgeOdo;
      legGap = CELL_CM;
      legRolling = true;
      n ++;
      /* [E33] CHECK 4: a crossing the robot drives through must have line
       * ahead. No line means this is the end of a column, not MID. */
      if(caseKind(n) == K_PASS){
        numGride = n;
        traceEvent(F("pass"));
        if(crossAhead == AH_BLANK) fault(5, F("no line after a MID crossing: the count is wrong"));
      }
    }
  }
  return (n);
}

void stopRobot(){
  odoTick();
  int u = (curUL + curUR) / 2;         /* [E33] the robot rolls a little after a brake */
  float c = speedCms(u) * BRAKE_COAST_MS / 1000.0;
  odoCm += (u >= 0) ? c : -c;
  digitalWrite(F_L,1); digitalWrite(B_L,1);
  digitalWrite(F_R,1); digitalWrite(B_R,1);
  curUL = 0; curUR = 0;
  sp = SP_START;
}

void upSpeed(){
  if(millis()-tUpSp >= 10){
    sp += 2;
    tUpSp = millis();
  }
  if(sp>maxSp){sp=maxSp;}
}

void moveFor(){
  setMotor(sp, sp);                    /* [E33] through the motor layer */
}

/* =====================================================================
 *  [E33]  EXACT SHORT MOVES
 *  The slides move "moveFor(); delay(50);": 50 ms at whatever speed the
 *  ramp had reached, a different distance every time. The bar is 9.5 cm
 *  ahead of the wheels, so the robot must move a known DISTANCE to put the
 *  wheels on a crossing. These use the speed the calibration measured.
 * ===================================================================== */

/* How far the bar is past the centre of the last crossing, in cm. */
float barPast(){
  return TAPE_W_CM / 2 + (odoCm - edgeOdo);
}

/* From rest: drive cm (negative = backwards) at MOVE_SP, then stop. */
void moveCm(float cm){
  int dir = 1;
  float o0 = odoCm;
  if(cm < 0){ dir = -1; cm = -cm; }
  if(cm < 0.3) return;
  float v = speedCms(MOVE_SP);
  if(v < 1) v = 1;
  unsigned long ms = cal.lagMove + (unsigned long)(cm / v * 1000.0);
  if(ms > 3000) ms = 3000;
  setMotor(dir * MOVE_SP, dir * MOVE_SP);
  unsigned long t0 = millis();
  while(millis() - t0 < ms){ odoTick(); }
  setMotor(0, 0);
  odoCm = o0 + dir * cm;               /* the move model already includes the stop */
  sp = SP_START;
  delay(SETTLE_MS);
}

/* Keep rolling (steering on the line if there is one) until the bar is
 * target cm past the crossing, then stop there. If it is already further,
 * back up. */
void rollToBarPast(float target){
  int s = MOVE_SP;
  float coast = speedCms(s) * BRAKE_COAST_MS / 1000.0;
  unsigned long t0 = millis();
  while(barPast() < target - coast){
    /* [E33] still on the crossing's own tape: taken at a slant, its tail
     * lights one end of the bar and steerAt() would follow it (with no line
     * past C1 / C4 the robot then turned onto the column) */
    if(barPast() < TAPE_W_CM + 0.7) setMotor(s, s);
    else steerAt(s);
    if(millis() - t0 > 3000) break;
  }
  stopRobot();
  delay(SETTLE_MS);
  float err = target - barPast();
  if(err > 0.7 || err < -0.7) moveCm(err);
}

/* Roll forward off a crossing without counting it. */
void leaveCrossing(){
  unsigned long t0 = millis();
  while(checkGrid()){
    setMotor(SP_START, SP_START);
    if(millis() - t0 > 700) break;
  }
  stopRobot();
}

/* =====================================================================
 *  TURNING  (robot08, robot11 p14 - p17)
 *
 *  [E33] The slides' turnRight90 spins until one exact pattern appears,
 *  calls upSpeed() inside the spin (so it speeds up and sweeps past the
 *  pattern) and never gives up. The names are kept; the body now COUNTS the
 *  lines that sweep under the bar:
 *    - a line only counts if it comes in from the side the robot is turning
 *      toward (so the line under the bar at the start is ignored),
 *    - the turn ends on the right line even if the timing is off,
 *    - it slows down as that line comes in and stops when it is centred,
 *    - it gives up with a fault instead of spinning for ever.
 *  With the wheels on a MID crossing the first line in is the new road.
 *  At the end of a column the wheels are ~7 cm before the end line, so a
 *  180 first sweeps over the end line, then finds the column: skip 1.
 * ===================================================================== */
#define ENTER_ERR   3          /* line has come in at the leading end */
#define STOP_ERR    1          /* ... and reached the middle          */
#define KICK_SP     40         /* extra speed for the first moment of a spin */
#define KICK_MS     80
uint8_t skip180 = 1;           /* set by keep_item / place_item       */
unsigned long lastTurnMs = 0;
uint8_t lastTurnLines = 0;

/* Follows one line at a time as it sweeps across the bar during a spin.
 * A line "comes in" at the end of the bar the robot is turning toward,
 * crosses the middle, and "leaves" at the other end. */
#define SE_NONE    0
#define SE_IN      1          /* a new line came in              */
#define SE_MIDDLE  2          /* it crossed the middle of the bar */
#define SE_OUT     3          /* it left                          */
struct SpinTrack {
  uint8_t state;              /* 0 no line, 1 a line on the bar   */
  uint8_t gone;               /* reads with nothing lit            */
  bool counts;                /* came in from the leading end     */
  bool mid;                   /* has crossed the middle            */
  int  e;                     /* last position, + = leading side   */
  uint8_t prevM;              /* the reading before                */
};

void spinBegin(SpinTrack &k, int dir){
  uint8_t m = scanBar();
  k.gone = 0; k.mid = false; k.e = 0; k.prevM = m;
  /* a line already under the bar at the start never counts */
  if(popcount8(m) > 0){ k.state = 1; k.counts = false; k.e = dir * errorOfMask(m); }
  else                { k.state = 0; k.counts = false; }
}

uint8_t spinStep(SpinTrack &k, int dir, uint8_t m){
  /* [E33] a sensor only counts if it, or its neighbour, was also lit in the
   * reading before: a real line moves only a little between two readings,
   * a flicker of noise does not stay */
  uint8_t p = k.prevM;
  k.prevM = m;
  m &= (uint8_t)(p | (p << 1) | (p >> 1));
  uint8_t b = popcount8(m);
  if(b == 0){
    if(k.state == 1){
      /* Left at the trailing end: gone. Vanished in the middle of the bar:
       * that is a weak or dead sensor, not the end of the line. Wait for it
       * to show up on the next sensor instead of counting it as passed. */
      k.gone++;
      if((k.e <= -ENTER_ERR + 1 && k.gone >= 3) || k.gone >= 150){
        k.state = 0; k.gone = 0;
        return SE_OUT;
      }
    }
    return SE_NONE;
  }
  k.gone = 0;
  /* A line crossed at a slant lights up to 5-6 sensors side by side; it is
   * still one line. Two separate spots, or 7-8 lit, give no position. */
  if(b > 6 || !isRun(m)) return SE_NONE;
  int e = dir * errorOfMask(m);
  k.e = e;
  if(k.state == 0){
    if(e >= ENTER_ERR){ k.state = 1; k.counts = true; k.mid = false; return SE_IN; }
    return SE_NONE;
  }
  if(!k.mid && e <= 0){ k.mid = true; if(k.counts) return SE_MIDDLE; }
  if(e <= -ENTER_ERR){ k.state = 0; return SE_OUT; }   /* gone past: next line may come */
  return SE_NONE;
}

/* [E33] the left motor is the weak one: a pivot from rest may not start at
 * the plain turn speed, so start with a short kick. (If the turn then takes
 * much longer than expected, spinTurn() adds speed: the "stall ladder".) */
int spinKick(int dir, int base){
  setMotor(dir * (base + KICK_SP), -dir * (base + KICK_SP));
  unsigned long t0 = millis();
  while(millis() - t0 < KICK_MS) scanBar();
  setMotor(dir * base, -dir * base);
  return base;
}

bool spinTurn(int dir, int deg, uint8_t skip){
  unsigned long ms90 = (unsigned long)(((dir > 0) ? cal.t90R : cal.t90L) / vScale);   /* battery */
  unsigned long full = ms90 * (unsigned long)deg / 90UL;
  /* a 180 cannot be on its new road before about 72 degrees; a 90 only
   * ignores flicker right at the start */
  unsigned long tMin = full * (deg >= 180 ? 40UL : 10UL) / 100UL;
  unsigned long tMax = full * 250UL / 100UL;
  SpinTrack k;
  spinBegin(k, dir);
  /* [E33] At a column end the end line can already be under the bar when
   * the spin starts (the robot stopped a little short, or at a slant). If it
   * sits in the middle or on the side the robot turns toward, it sweeps
   * across and out: it IS the line to skip, so count it when it leaves.
   * (On the far side it leaves at once and comes in again: counted then.) */
  if(skip && k.state == 1 && k.e >= -1) k.counts = true;
  uint8_t passes = 0;
  bool target = false, kicking = true;
  unsigned long t0 = millis(), tBump = 0;
  int bump = 0;                          /* extra speed if the turn is too slow */
  /* [E33] start with a short kick for the weak motor; the bar is watched
   * during the kick too, so a line leaving and coming back is not missed */
  setMotor(dir * (TURN_SP + KICK_SP), -dir * (TURN_SP + KICK_SP));
  while(true){
    uint8_t m = scanBar();
    unsigned long now = millis();
    unsigned long t = now - t0;
    if(t > tMax){ stopRobot(); lastTurnMs = t; lastTurnLines = passes; return false; }
    if(kicking && t >= KICK_MS){
      kicking = false;
      if(!target) setMotor(dir * TURN_SP, -dir * TURN_SP);
    }
    /* Over white the bar cannot tell whether the robot is turning, so a
     * stalled wheel shows up only as a turn that takes too long: then add
     * speed, a little at a time (a "stall ladder"). */
    if(t > full * 13 / 10 && now - tBump > 300 && bump < 40){
      bump += 10; tBump = now;
      int u = (target ? TURN_SLOW_SP : TURN_SP) + bump;
      setMotor(dir * u, -dir * u);
    }
    uint8_t ev = spinStep(k, dir, m);
    if(ev == SE_OUT && k.counts){
      if(target){                          /* swept past it: look for the next one */
        target = false;
        setMotor(dir * (TURN_SP + bump), -dir * (TURN_SP + bump));
      }
      else passes ++;
    }
    if(ev == SE_IN && passes >= skip && t >= tMin && !target){
      target = true;                       /* the line we want is coming in: slow down */
      setMotor(dir * (TURN_SLOW_SP + bump), -dir * (TURN_SLOW_SP + bump));
    }
    if(target && k.state == 1 && k.e <= STOP_ERR) break;
  }
  stopRobot();
  lastTurnMs = millis() - t0;
  lastTurnLines = passes;
  delay(60);
  return true;
}

/* [E33] After a turn: a narrow line must be under the bar, near the middle.
 * Nudges the robot until it is; looks a little each way if the line is not
 * there. Returns false if it cannot find it. */
bool searchLine(int dir, unsigned long ms){
  unsigned long t0 = millis();
  while(millis() - t0 < ms){
    setMotor(dir * TURN_SLOW_SP, -dir * TURN_SLOW_SP);
    delay(12);
    setMotor(0, 0);
    delay(8);
    uint8_t m = scanBar();
    if(popcount8(m) >= 1 && popcount8(m) <= 3 && isRun(m)) return true;
  }
  return false;
}

bool centreOnLine(int lastDir){
  unsigned long t90 = (cal.t90L + cal.t90R) / 2;
  uint8_t odd = 0;                       /* readings in a row that make no sense */
  for(uint8_t k=0;k<30;k++){
    delay(15);
    uint8_t m = scanBar();
    uint8_t b = popcount8(m);
    if(b >= 1 && b <= 3 && isRun(m)){
      int e = errorOfMask(m);
      if(abs(e) <= 2) return true;
      int d = (e > 0) ? 1 : -1;          /* line on the right: turn right */
      setMotor(d * TURN_SLOW_SP, -d * TURN_SLOW_SP);
      delay(12);
      setMotor(0, 0);
    }else if(b == 0){
      /* turns stop late more often than early: look back first */
      if(!searchLine(-lastDir, t90 * 35 / 100) && !searchLine(lastDir, t90 * 70 / 100)) return false;
    }else{
      /* something wide under the bar, or two separate spots: one noisy
       * reading can do that, so only give up if it stays that way */
      if(++odd >= 3) return false;
      continue;
    }
    odd = 0;
  }
  return false;
}

void afterTurn(int dir){
  if(!centreOnLine(dir)) fault(7, F("no line under the bar after the turn"));
  stopRobot();
  clearPid();
  gridArmed = !checkGrid();
}

bool turnTimed(){                          /* can turn times be checked? */
  if(calSource == 2) return true;
  uint8_t s = calItem(calStatus, CI_TURN);
  return calSource == 1 && (s == ST_PASS || s == ST_WEAK);
}

void traceTurn(const __FlashStringHelper *what, int deg, int dir){
#if TRACE_ROUTE
  unsigned long ms90 = (unsigned long)(((dir > 0) ? cal.t90R : cal.t90L) / vScale);
  Serial.print(F("  ")); Serial.print(what);
  Serial.print(F(" ")); Serial.print(lastTurnMs); Serial.print(F(" ms (expected about "));
  Serial.print(ms90 * deg / 90); Serial.print(F("), lines passed ")); Serial.println(lastTurnLines);
#endif
}

/* [E33] With the wheels on a MID crossing, the new road is the FIRST line
 * that comes in. If a line was passed first, the robot turned too far. */
void turnBy90(int dir){
  if(!spinTurn(dir, 90, 0)) fault(6, F("90 degree turn never found the line"));
  traceTurn(dir > 0 ? F("turnRight90") : F("turnLeft90"), 90, dir);
  if(lastTurnLines > 0) fault(10, F("90 degree turn passed a line first: turned too far"));
  afterTurn(dir);
}

/* [E33] one 180-degree spin that counts lines, instead of 90 + wait + 90:
 * at the end of a column there is no line at 90 degrees for the first half
 * to stop on, which is how the robot ended up driving along the TOP line. */
void turnBy180(int dir){
  if(!spinTurn(dir, 180, skip180)) fault(6, F("180 degree turn never found the line"));
  traceTurn(dir > 0 ? F("turnRight180") : F("turnLeft180"), 180, dir);
  unsigned long full = (unsigned long)(((dir > 0) ? cal.t90R : cal.t90L) * 2UL / vScale);
  if(turnTimed() && (lastTurnMs < full / 2 || lastTurnMs > full * 2))
    fault(11, F("180 degree turn took far too short or too long: stopped on the wrong line?"));
  afterTurn(dir);
}

void turnRight90(){  turnBy90(+1); }
void turnLeft90(){   turnBy90(-1); }
void turnRight180(){ turnBy180(+1); }
void turnLeft180(){  turnBy180(-1); }

/* =====================================================================
 *  GRIPPER  (robot09 p07 - p08, robot11 p18 - p20)
 *  [E33] Servos move a few degrees at a time instead of jumping, and after
 *  closing, the gripper opens GRIP_BACKOFF_DEG again so it holds without
 *  pushing at full current. Both cut the current peaks that can reset the
 *  Nano on a 9 V battery.
 * ===================================================================== */
int gripNow = GRIP_OPEN;
int armNow  = ARM_DOWN;

/* The Servo library sends one pulse every 20 ms, so steps are 3 degrees per
 * 20 ms (150 degrees a second): faster steps would only arrive as jumps. */
void servoTo(Servo &s, int &now, int target){
  while(now != target){
    int d = target - now;
    if(d > 3) d = 3;
    if(d < -3) d = -3;
    now += d;
    s.write(now);
    delay(20);
  }
}

void keepup_object(){
  servoTo(servo_x, gripNow, GRIP_CLOSED);//หนีบของ
  delay(200);
  servoTo(servo_x, gripNow, GRIP_CLOSED + GRIP_BACKOFF_DEG);   /* [E33] hold, do not stall */
  delay(100);
  servoTo(servo_y, armNow, ARM_CARRY);//ยกของ
  delay(300);
}

void put_object(){
  servoTo(servo_y, armNow, ARM_DOWN);//วางของลง
  delay(200);
  servoTo(servo_x, gripNow, GRIP_RELEASE);//คลายแขนหนีบ
  delay(200);
}

void arm_over_head(){
  servoTo(servo_y, armNow, ARM_HIGH);
  delay(200);
}

/* ===================================================================== */

void beginFnc(){
  pinMode(sp_L,OUTPUT); pinMode(F_L,OUTPUT); pinMode(B_L,OUTPUT);
  pinMode(sp_R,OUTPUT); pinMode(F_R,OUTPUT); pinMode(B_R,OUTPUT);
  pinMode(STBY,OUTPUT);
  pinMode(LED_PIN,OUTPUT);           /* [E33] */
  digitalWrite(STBY,0);              /* [E33] motors stay off until we are sure */

  /* [E33] Match the two motors' PWM frequency. D6 (left) is on Timer0 at
   * 976 Hz. D3 (right) is on Timer2, which Arduino leaves at 490 Hz in a
   * different PWM mode. Fast PWM /64 puts D3 on the same 976 Hz with the
   * same duty formula as D6. Timer0 is not touched (millis and delay use
   * it) and Timer1 is not touched (Servo.h uses it). This explains a few
   * percent of the mismatch at most; the trim handles the rest. */
#if defined(__AVR__)
  TCCR2A |= _BV(WGM21) | _BV(WGM20);
  TCCR2B  = (TCCR2B & ~(_BV(CS22)|_BV(CS21)|_BV(CS20))) | _BV(CS22);
  /* [E33] ADC clock 500 kHz (prescaler 32) instead of 125 kHz: still accurate
   * to about 9-10 bits, and pays for reading every channel twice. */
  ADCSRA = (ADCSRA & ~(_BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0))) | _BV(ADPS2) | _BV(ADPS0);
#endif

  calLoad();                         /* [E33] no motion */
  calApply();

  /* [E33] After a brown-out reset we do NOT start again: the robot is
   * somewhere in the middle of the field and would drive off. */
  printResetCause();
  printCalSource();
  bool midRun = (runMark == RUN_MARK);
  runMark = 0;                           /* the next reset starts normally */
  if(brownOutReset() || midRun){
    Serial.println(F("RESTARTED IN THE MIDDLE OF A RUN (brown-out: weak battery; or reset / USB)."));
    Serial.println(F("Not driving. Fresh battery, robot at the start, then press reset."));
    while(1){ digitalWrite(LED_PIN, (millis()/100) & 1); }
  }

  delay(1000);

  /* [E33] write the angle BEFORE attach, so the first pulse is the angle we
   * want and not the library's 90 degrees; one servo at a time. */
  servo_x.write(GRIP_OPEN);//คลายแขนจับ
  servo_x.attach(x_pin);//เชื่อมต่อขาสัญญาณ
  delay(300);
  servo_y.write(ARM_DOWN);//ยกแขนลง
  servo_y.attach(y_pin);//เชื่อมต่อขาสัญญาณ
  delay(300);
  gripNow = GRIP_OPEN;
  armNow  = ARM_DOWN;

  digitalWrite(STBY,1);
  clearPid();
}

/* =====================================================================
 *  [E33]  TRACE AND FAULT
 *  One short line per event so a failed run can be read afterwards.
 *  A fault stops the robot, keeps the gripper as it is, prints why, and
 *  blinks the LED (D13) the fault number of times, over and over.
 * ===================================================================== */
void printAhead(uint8_t a){
  if(a == AH_LINE)       Serial.print(F("LINE"));
  else if(a == AH_BLANK) Serial.print(F("BLANK"));
  else if(a == AH_WIDE)  Serial.print(F("WIDE"));
  else if(a == AH_LOST)  Serial.print(F("RAN-OUT"));
  else                   Serial.print(F("UNSURE"));
}

void printMask(uint8_t m){
  for(uint8_t i=0;i<8;i++) Serial.print((m & (0x80 >> i)) ? '1' : '0');
}

void traceEvent(const __FlashStringHelper *what){
#if TRACE_ROUTE
  Serial.print(F("n="));   Serial.print(numGride);
  Serial.print(F(" "));    Serial.print(what);
  Serial.print(F(" d="));  Serial.print(crossDist, 1);
  Serial.print(F("/"));    Serial.print(crossGap, 1);
  Serial.print(F(" m="));  printMask(crossMask);
  Serial.print(F(" ahead=")); printAhead(crossAhead);
  Serial.print(F(" v="));  Serial.println(vScale, 2);
#endif
}

/* The calibration mode sets this so a fault there saves what it measured
 * and prints a report instead of only stopping. */
typedef void (*FaultHook)(uint8_t code, const __FlashStringHelper *why);
FaultHook faultHook = 0;

void fault(uint8_t code, const __FlashStringHelper *why){
  stopRobot();
  digitalWrite(STBY,0);
  runMark = 0;                           /* stopped on purpose, not a restart */
  if(faultHook) faultHook(code, why);
  Serial.print(F("FAULT ")); Serial.print(code); Serial.print(F(": ")); Serial.println(why);
  Serial.print(F("  at n=")); Serial.print(numGride);
  Serial.print(F("  bar ")); printMask(lineMask);
  Serial.print(F("  d=")); Serial.print((int)(odoCm - legRef)); Serial.print('/'); Serial.println((int)legGap);
  Serial.println(F("  (the LED on D13 blinks the fault number)"));
  while(1){
    for(uint8_t k=0;k<code;k++){ digitalWrite(LED_PIN,1); delay(200); digitalWrite(LED_PIN,0); delay(200); }
    delay(1000);
  }
}

#endif
