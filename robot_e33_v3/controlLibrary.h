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
#include "runLog.h"
#if defined(__AVR__)
  #include <avr/io.h>
#else                                   /* [E33] host simulator: flash is plain memory */
  #ifndef PROGMEM
    #define PROGMEM
  #endif
  #ifndef memcpy_P
    #define memcpy_P memcpy
  #endif
  #ifndef pgm_read_word
    #define pgm_read_word(p) (*(const uint16_t *)(p))
  #endif
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

extern int numGride;          /* defined in robot_e33_v3.ino */

/* What the route expects at each count number. caseInfo(n) is written in
 * robot_e33_v3.ino, next to the switch, so the route is in one place. */
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
  uint8_t  spare;            /* [E33] 0 = finished, else 1 + the step it stopped in */
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

/* ([E33] noinline: one copy saves flash) */
__attribute__((noinline)) uint8_t calItem(uint16_t st, uint8_t k){ return (st >> (2*k)) & 3; }

uint16_t fletcher16(const uint8_t *p, uint8_t n){
  uint16_t a = 0, b = 0;
  while(n--){ a = (a + *p++) % 255; b = (b + a) % 255; }
  return (uint16_t)((b << 8) | a);
}

bool calSlotValid(const CalRec &r){
  return r.magic == CAL_MAGIC && r.version == CAL_VERSION && r.size == sizeof(CalRec)
      && r.fletcher == fletcher16((const uint8_t*)&r, sizeof(CalRec) - 2);
}

/* [E33] the CAL VALUES block of calibration.h as one record in flash, in
 * CalData's order: one copy is much smaller than setting every value */
const CalData calMacros PROGMEM = {
  CAL_LO, CAL_HI, CAL_LINE_LOW, CAL_DEAD_MASK,
  CAL_DB_LF, CAL_DB_LR, CAL_DB_RF, CAL_DB_RR,
  CAL_TRIM_L, CAL_TRIM_R, CAL_TRIM_LR, CAL_TRIM_RR,
  CAL_V_CRUISE_X100, CAL_V_MOVE_X100, CAL_LAG_MOVE_MS,
  CAL_T90_L_MS, CAL_T90_R_MS, CAL_VALUES_FROM_RUN
};

void calFromMacros(){
  memcpy_P(&cal, &calMacros, sizeof(CalData));
  /* the macros hold RAW readings; store them on the black-high scale */
  if(cal.lineLow){
    for(uint8_t i=0;i<8;i++){ cal.lo[i] = 1023 - cal.lo[i]; cal.hi[i] = 1023 - cal.hi[i]; }
  }
}

/* Copy one calibration item from an EEPROM record.
 * [E33] each item's values sit side by side in CalData, so one byte copy
 * per item does it (much less flash than one copy per value). */
#define CAL_SPAN(first, next) at = offsetof(CalData, first); n = offsetof(CalData, next) - at
void calCopyItem(CalData &to, const CalData &s, uint8_t item){
  uint8_t at, n;
  switch(item){
    case CI_SENSOR: CAL_SPAN(lo, dbLf);         break;   /* lo, hi, lineLow, deadMask */
    case CI_DEADBD: CAL_SPAN(dbLf, trimL);      break;   /* dbLf, dbLr, dbRf, dbRr    */
    case CI_TRIM:   CAL_SPAN(trimL, trimLr);    break;   /* trimL, trimR              */
    case CI_MOVE:   CAL_SPAN(vMove100, t90L);   break;   /* vMove100, lagMove         */
    case CI_CRUISE: CAL_SPAN(vCruise100, vMove100); break;
    case CI_TURN:   CAL_SPAN(t90L, fromRun);    break;   /* t90L, t90R                */
    case CI_SPIN:   CAL_SPAN(trimLr, vCruise100); break; /* trimLr, trimRr            */
    default: return;
  }
  memcpy((uint8_t*)&to + at, (const uint8_t*)&s + at, n);
}

/* Read the newer valid of the two slots. Returns 0 if neither is valid,
 * otherwise the slot address + 1 (and out is then undefined).
 * [E33] each slot is read straight into out: no second record in memory
 * and no copying, which saves flash. */
int calReadEeprom(CalRec &out){
  EEPROM.get(CAL_SLOT_B, out);
  bool vb = calSlotValid(out);
  uint16_t seqB = out.seq;
  EEPROM.get(CAL_SLOT_A, out);
  if(calSlotValid(out) && (!vb || (int16_t)(out.seq - seqB) > 0)) return CAL_SLOT_A + 1;
  if(!vb) return 0;
  EEPROM.get(CAL_SLOT_B, out);
  return CAL_SLOT_B + 1;
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

/* [E33] a value far outside what a robot can have (a damaged record, or
 * one from another sketch that happens to pass the check): use the default.
 * The nine values from trimL to t90R sit side by side in CalData, so one
 * loop over a table of their limits does it (much less flash than nine calls). */
const uint16_t calLimits[9][2] PROGMEM = {
  { 300, 1000 }, { 300, 1000 },      /* trimL, trimR   */
  { 300, 1000 }, { 300, 1000 },      /* trimLr, trimRr */
  { 100, 10000 }, { 100, 10000 },    /* vCruise100, vMove100 */
  { 0, 1000 },                       /* lagMove        */
  { 150, 5000 }, { 150, 5000 }       /* t90L, t90R     */
};
static_assert(offsetof(CalData, t90R) - offsetof(CalData, trimL) == 8 * sizeof(uint16_t),
              "calSanitize: trimL .. t90R must be side by side");
static_assert(offsetof(CalData, vCruise100) - offsetof(CalData, trimL) == 4 * sizeof(uint16_t) &&
              offsetof(CalData, t90L) - offsetof(CalData, trimL) == 7 * sizeof(uint16_t),
              "calSanitize: calLimits must follow the order of the values in CalData");
void calSanitize(){
  uint16_t *v = &cal.trimL;
  const uint16_t *def = &calMacros.trimL;     /* the defaults: calibration.h */
  for(uint8_t i=0;i<9;i++){
    if(v[i] < pgm_read_word(&calLimits[i][0]) || v[i] > pgm_read_word(&calLimits[i][1]))
      v[i] = pgm_read_word(&def[i]);
  }
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
    Serial.println(F("DEFAULTS: never calibrated. Run MODE_CALIBRATE once for good turns."));
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
  #ifndef SIM_RUNMARK                    /* (the simulator can fake a restart) */
  #define SIM_RUNMARK 0
  #endif
  uint32_t runMark = SIM_RUNMARK;
#endif
#define RUN_MARK 0xE33A55C3UL

/* [E33] v3: what a restart in the middle of a run needs to carry on. It is
 * in the same kind of memory as runMark (kept through a reset, lost when the
 * robot is switched off for more than a moment), with a check sum, so a
 * switch-on never finds it valid by chance. keepSave() writes it at every
 * count, before every turn and at every pick and place. */
#if defined(__AVR__)
  #define NOINIT __attribute__((section(".noinit")))
#else
  #define NOINIT                       /* (the simulator keeps these by name) */
#endif
#define KS_DRIVE  0                    /* on the way to the next crossing     */
#define KS_ACTION 1                    /* standing at a column end: the servos move */
#define KS_TURN   2                    /* spinning (a turn90 or a 180)        */
#define KEEP_LV   offsetof(CalData, dbLf)
struct Keep {
  uint8_t n;                           /* numGride                            */
  uint8_t step;                        /* KS_DRIVE / KS_ACTION / KS_TURN      */
  int8_t  dir;                         /* the turn: +1 right, -1 left         */
  int8_t  left;                        /* lines the spin must still pass      */
  uint8_t cur;                         /* under the bar: 1 a line to pass, 2 the new road */
  uint8_t held;                        /* 1 = an object is in the jaws        */
  float   bp;                          /* the bar past the end line at the action, cm */
  float   leg;                         /* the leg after the turn it is in     */
  float   vScale;                      /* the speed this run measured         */
  uint8_t vSamples;                    /* ... from this many cells            */
  uint8_t lv[KEEP_LV];                 /* this run's sensor levels (cal.lo, cal.hi,
                                        * lineLow, deadMask): the ones measured
                                        * standing still at the start, if it did */
  uint16_t sum;
};
Keep keep NOINIT;
extern int gripNow, armNow;
__attribute__((noinline)) uint16_t keepSum(){
  const uint8_t *p = (const uint8_t *)&keep;
  uint16_t s = 0x3E33 ^ gripNow ^ (armNow << 8);   /* (the servo angles too) */
  for(uint8_t i=0;i<offsetof(Keep, sum);i++) s = ((s << 1) | (s >> 15)) ^ p[i];
  return s;
}
__attribute__((noinline)) void keepSave(){ keep.sum = keepSum(); }
bool keepValid(){ return keep.sum == keepSum(); }   /* (runMark is checked first) */
float vScale = 1.0;
uint8_t vSamples = 0;                  /* cells measured during this run */
/* the count and what the robot is doing now (and this run's speed) */
__attribute__((noinline)) void keepStep(uint8_t step){
  keep.n = (uint8_t)numGride; keep.step = step;
  keep.vScale = vScale; keep.vSamples = vSamples;
  keepSave();
}
/* a spin: its direction, the lines it must still pass, and what is under
 * the bar (1 a line it is passing, 2 the new road); saved when it changes */
__attribute__((noinline)) void keepSpin(int8_t dir, int8_t left, uint8_t cur){
  if(dir == keep.dir && left == keep.left && cur == keep.cur) return;
  keep.dir = dir; keep.left = left; keep.cur = cur;
  keepSave();
}
bool resuming = false;                 /* [E33] v3: carry on after a restart */

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
#if ROBOT_MODE != MODE_MISSION          /* [E33] v3: only the calibration reads these */
  odoU += u2 * s;
  odoEffL += curUL * s;
  odoEffR += curUR * s;
#endif
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

/* [E33] The next group of side-by-side lit sensors in m, from sensor i on
 * (i moves past it); 0 when there is none. One copy for the three places
 * that look at each group in turn saves flash. */
uint8_t nextRun(uint8_t m, uint8_t &i){
  while(i < 8 && !(m & (0x80 >> i))) i++;
  uint8_t run = 0;
  while(i < 8 && (m & (0x80 >> i))){ run |= (0x80 >> i); i++; }
  return run;
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
  uint8_t best = 0, run, i = 0; int bestE = 100;
  while((run = nextRun(grp, i))){
    int e = abs(errorOfMask(run));
    if(e < bestE){ bestE = e; best = run; }
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
  uint8_t i = 0, run;
  while((run = nextRun(m, i))){
    int c = errorOfMask(run);
    int d = abs(c - lastLineErr);
    if(d < bestD){ bestD = d; best = c; }
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

/* [E33] Both wheels, each kept within 0 .. s (the slides' four clamp
 * lines, in one place for the steering below; noinline saves flash). */
__attribute__((noinline)) void setMotorCapped(int speedL, int speedR, int s){
  if(speedL>s){speedL=s;}
  if(speedL<0){speedL=0;}
  if(speedR>s){speedR=s;}
  if(speedR<0){speedR=0;}
  setMotor(speedL, speedR);
}

/* [E33] Spin on the spot at speed u, dir +1 = right (noinline saves flash). */
__attribute__((noinline)) void spinAt(int dir, int u){ setMotor(dir * u, -dir * u); }

/* [E33] The slides write nothing when the line is lost, so the motors keep
 * their last command for ever. Instead: keep the last steering slowly for a
 * moment (the line is often just under a gap between sensors), then swing
 * the bar about 27 degrees toward where the line was last seen, then the
 * same the other way, then back. Only then stop. */
#define GAP_BRIDGE_CM   5.0f   /* [E33] a gap in worn tape up to this long is driven over */
float odoLost = 0;                     /* [E33] the odometer when the line was lost */
unsigned long holdMs = LOST_HOLD_MS;   /* [E33] when the hold ended (later after a gap) */

void lineLost(){
  unsigned long now = millis();
  if(tLost == 0){ tLost = now; odoLost = odoCm; holdMs = LOST_HOLD_MS; }
  unsigned long t = now - tLost;
  int e = lastLineErr;
  /* [E33] Worn tape: the line vanished from the MIDDLE of the bar. Drifting
   * off a line, it leaves through one end of the bar instead. So this is a
   * gap in the tape: drive straight on over it, up to GAP_BRIDGE_CM, before
   * the swing (the swing only looks to the sides, never further ahead). */
  bool gap = (e >= -2 && e <= 2) && odoCm - odoLost < GAP_BRIDGE_CM;
  if(t < holdMs || gap){
    int s = sp < MOVE_SP ? sp : MOVE_SP;
    if(t >= holdMs){ holdMs = t; e = 0; }      /* [E33] past the usual hold: straight */
    setMotorCapped(s + (s*e)/7, s - (s*e)/7, s);
    return;
  }
  int d = (e < 0) ? -1 : 1;                    /* line last seen on the right: look right first */
  unsigned long ts = t - holdMs;
  unsigned long a = (unsigned long)(cal.t90L + cal.t90R) * 15 / 100;   /* ~27 degrees */
  if(ts < a)          spinAt( d, TURN_SLOW_SP);
  else if(ts < 3 * a) spinAt(-d, TURN_SLOW_SP);
  else if(ts < 4 * a) spinAt( d, TURN_SLOW_SP);
  else{
    fault(1, F("line lost"));
    /* [E33] (STOP_ON_FAULT 0) look again: on a little, then the swing */
    tLost = now; odoLost = odoCm; holdMs = LOST_HOLD_MS;
  }
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

/* [E33] followLine()'s steering from a PID output, at speed s (noinline
 * saves flash). */
__attribute__((noinline)) void steerWith(float pidOut, int s){
  int spOut = map(round(pidOut),-7,7,-s,s);
  setMotorCapped(s - spOut, s + spOut, s);
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
  steerWith(pidFNC(errorInput,0,1,0,0.7), s);
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
#define START_BLIND_CM   3.5  /* [E33] v3: after a standing start, no crossing this near */
#define START_BLIND_FRAC 0.5  /* ... nor within this share of the leg       */
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
  uint8_t i = 0, run;
  while((run = nextRun(m, i))){
    if(popcount8(run) <= 3 && abs(errorOfMask(run)) <= 5) return true;
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
      steerWith(pidFNC(errorOfMask(m),0,1,0,0.7), s);
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

/* [E33] Keep this leg, so restoreLeg() can go back to it (noinline: one
 * copy saves flash). */
__attribute__((noinline)) void saveLeg(){
  prevLegRef = legRef; prevLegGap = legGap; prevLegRolling = legRolling;
}

/* Undo the last count: measure the next crossing from the old start again. */
void restoreLeg(){
  legRef = prevLegRef;
  legGap = prevLegGap;
  legRolling = prevLegRolling;
}

/* [E33] a distance in cm as whole tenths, for the log and the trace
 * (noinline: one copy of the float code saves flash) */
__attribute__((noinline)) int16_t tenths(float cm){ return (int16_t)(cm * 10); }
/* [E33] vScale x 100, rounded (kept within 40 .. 250), the same way */
__attribute__((noinline)) int vScale100(){ return (int)(vScale * 100 + 0.5); }

/* [E33] one record of the run log for the crossing just checked */
void logCross(uint8_t type, int n){
  logAdd((uint8_t)(type | (crossAhead << 4)), n, tenths(crossDist), tenths(crossGap),
         crossMask, (uint8_t)vScale100());
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

  /* [E33] far past where the next crossing should be, and nothing: the
   * wheels are not really turning (motor switch off, only USB power) */
  if(dNow > legGap + 2.5 * CELL_CM) fault(12, F("no crossing for far too long: motors switched on?"));

  /* [E33] ran off the end of a column without seeing the T: that WAS the end */
  /* (only where the T should be: a line lost earlier is a bump or a drift,
   * and lineLost() looks for it; at 0.6 a knock 6 cm before the T was taken
   * for the end and the gripper closed on nothing) */
  /* [E33] worn tape: if the line vanished from the middle of the bar (a
   * gap, see lineLost), not at once but 2.5 cm on. A T just past a short
   * gap then still comes under the bar and is counted as usual, and the
   * jaws, 2.5 cm ahead of the bar, have not yet passed where they close on
   * the object. */
  bool gapLike = (lastLineErr >= -2 && lastLineErr <= 2);
  if(next == K_END && tLost && (gapLike ? odoCm - odoSeen >= 2.5f : millis() - tLost >= 40)
     && odoSeen - legRef >= 0.8 * legGap){
    saveLeg();
    crossDist = odoSeen - legRef;
    crossGap = legGap;
    crossMask = 0; crossDwell = 0; crossAhead = AH_LOST;
    edgeOdo = odoSeen; tEdge = tLineSeen;
    legRef = odoSeen; legGap = CELL_CM; legRolling = false;
    tLost = 0;
    n ++;
    traceEvent(F("END found by the line running out"));
    logCross(LG_ENDLOST, n);
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
        if(millis() - tIn > CROSS_MAX_MS){
          fault(2, F("on a crossing for too long: driving along a line"));
          break;                         /* [E33] (STOP_ON_FAULT 0) take it as passed */
        }
      }
      tEdge = millis();
      edgeOdo = odoCm;
      crossDwell = (uint16_t)(tEdge - tIn);
      crossDist  = edgeOdo - legRef;
      crossGap   = legGap;
      /* look past it: a short look at an end, so the jaws do not push the object */
      if(next == K_END) crossAhead = lookAhead(0.5, 0.8);
      else              crossAhead = lookAhead(1.0, 1.5);
      /* [E33] worn tape: a gap just past a MID crossing looks like the end
       * of a column. Before calling it that, look once more, 3 cm on
       * (not where blank is allowed anyway: out of the field at C1 / C4). */
      if(next != K_END && crossAhead == AH_BLANK && !(caseInfo(n+1) & AHEAD_MAY_BLANK))
        crossAhead = lookAhead(0, 3.0);

      /* [E33] The odometer only knows the speed the calibration measured, and
       * a 9 V battery is weaker later in the day. So the distance checks wait
       * until THIS run has measured its own speed: the first crossing sets it
       * (its distance from the start is known), and every crossing passed at
       * speed one cell after the last corrects it. Until then nothing is
       * judged "missed" (a slow robot would look like one that missed a line). */
      bool speedKnown = vSamples > 0;

      /* [E33] CHECK 1: far too early -> a double count or a smudge. A leg
       * shorter than half a cell (back to MID after a 180) is only a few cm:
       * a share of it is less than a small error in ROW_CM, so there only a
       * crossing right away is a double count */
      /* [E33] worn tape: a real crossing lies across the whole bar. One
       * that did not light both outermost sensors (a dead one aside), well
       * before the next crossing is due, is a black mark beside the line
       * (dirt, a piece of tape): not counted either. (The wrong line of
       * CHECK 2 meets its first column late, not early.) */
      bool mark = ((crossMask | cal.deadMask) & 0x81) != 0x81 && crossDist < legGap * (speedKnown ? 0.8 : 0.6);
      float tooNear = legGap * (speedKnown ? MIN_GAP_FRAC : 0.3);
      /* [E33] v3: after a standing start (the start, every turn, every 180)
       * the bar reads darker for a moment while the robot speeds up, and your
       * robot saw a "crossing" 1.4 to 4 cm on almost every time. After the
       * 180 at C4 TOP the leg back to MID is only 9.7 cm, and v2 (which
       * there rejected only one closer than 1.5 cm) counted one at 2.0 cm as
       * C4 MID and turned 7.7 cm early. Now not within START_BLIND_FRAC of a
       * leg from standing, nor within START_BLIND_CM. (Only a leg from
       * standing can be shorter than a cell: a leg rolling over a crossing
       * is always one cell or one row.) */
      if(!legRolling){
        tooNear = legGap * START_BLIND_FRAC;
        if(tooNear < START_BLIND_CM) tooNear = START_BLIND_CM;
      }
      if(mark || crossDist < tooNear){
        /* [E33] nothing counted yet: this is C1 (the robot was put down with
         * the bar just before it) or a mark near the start. The distance from
         * the start to C2 is then unknown: legGap 0 = do not learn the speed
         * at C2, learn it over the first whole cell */
        logCross(LG_REJECT, n);
        /* (route 132 starts with a turn, and its first leg, up C1 to the
         * top, has a known length: keep it) */
        if(n == 0 && next == K_PASS) legGap = 0;
        nRejected ++;
        traceEvent(mark ? F("not counted: a mark beside the line?") : F("not counted: too early"));
        return (n);
      }
      /* [E33] CHECK 3: about one cell too far -> a crossing was missed */
      if(speedKnown && crossDist > legGap + 0.5 * CELL_CM){
        if(next == K_PASS){
          n ++;                          /* count the one that was missed */
          nCredited ++;
          logCross(LG_CREDIT, n);
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
        if(ratio > 0.4 && ratio < 2.5){
          vScale *= ratio;
          /* [E33] the leg from the start is only as long as the robot was
           * put down: use it, but judge "missed" only after a whole cell
           * (put down 6 cm past C1, C3 looked like a missed crossing) */
          if(legRolling) vSamples = 1;
        }
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
        /* [E33] worn tape: one arm of the real crossing has faded. It is
         * still this crossing if it came where it was expected and what is
         * past it fits (line after a MID crossing, none after a column end).
         * On the wrong line (along TOP after a bad 180) the first column
         * comes far from where the next crossing should be. */
        if(ratio > 0.8 && ratio < 1.25 && (crossAhead == AH_LINE) == ((info & 3) != K_END)) ok = true;
        if(!ok){
          numGride = n + 1;
          traceEvent(F("half crossing"));
          logCross(LG_CROSS, n + 1);        /* the log keeps the half crossing's mask */
          fault(3, F("half crossing: the robot is on the wrong line"));
        }
      }
      saveLeg();
      legRef = edgeOdo;
      /* [E33] the next crossing: one cell on along MID, or one row on when
       * the robot drives straight on up a column to its end (route 132 at
       * C4). After a turn or an end, turn90() / turnAround() set it. */
      legGap = (caseKind(n + 2) == K_END) ? ROW_CM : CELL_CM;
      legRolling = true;
      n ++;
      logCross(LG_CROSS, n);
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
bool lastTurnHidden = false;   /* [E33] the new road was hidden: found it 5 cm on */
bool turnTimed();

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

/* blank: ms since a line was last lit (0 = unknown: only the leading end counts) */
uint8_t spinStep(SpinTrack &k, int dir, uint8_t m, unsigned long blank = 0){
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
    /* [E33] worn tape: a line that first shows up in the middle of the bar,
     * not at its leading end, after a long blank (the sweep between two
     * roads), has a gap where that end crosses it. It still came in. (Not
     * right after a line was lit: that is the line the spin started on,
     * which a weak sensor under it hid for a moment.) */
    if(e >= ENTER_ERR || (e >= 0 && blank > (cal.t90L + cal.t90R) / 8)){
      k.state = 1; k.counts = true; k.mid = false; return SE_IN;
    }
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
  spinAt(dir, base + KICK_SP);
  unsigned long t0 = millis();
  while(millis() - t0 < KICK_MS) scanBar();
  spinAt(dir, base);
  return base;
}

/* [E33] How long a turn of deg degrees should take (the calibration's
 * time, corrected for the battery by vScale). One copy for spinTurn, the
 * run log and the trace (noinline saves flash). */
__attribute__((noinline)) unsigned long turnFullMs(int dir, int deg){
  unsigned long ms90 = (unsigned long)(((dir > 0) ? cal.t90R : cal.t90L) / vScale);   /* battery */
  return ms90 * (unsigned long)deg / 90UL;
}

bool spinTurn(int dir, int deg, uint8_t skip){
  unsigned long full = turnFullMs(dir, deg);
  /* a 180 cannot be on its new road before about 72 degrees; a 90 only
   * ignores flicker right at the start */
  unsigned long tMin = full * (deg >= 180 ? 40UL : 10UL) / 100UL;
  unsigned long tMax = full * 250UL / 100UL;
  SpinTrack k;
  spinBegin(k, dir);
  /* [E33] v3: a restart in this spin carries on with it (resumeTurn) */
  keep.step = KS_TURN; keep.cur = 9;    /* (9: keepSpin saves it all now) */
  keepSpin(dir, skip, 0);
  /* [E33] At a column end the end line can already be under the bar when
   * the spin starts (the robot stopped a little short, or at a slant). If it
   * sits in the middle or on the side the robot turns toward, it sweeps
   * across and out: it IS the line to skip, so count it when it leaves.
   * (On the far side it leaves at once and comes in again: counted then.) */
  if(skip && k.state == 1 && k.e >= -1) k.counts = true;
  uint8_t passes = 0;
  bool target = false, kicking = true;
  unsigned long t0 = millis(), tBump = 0;
  /* [E33] when the last line left the bar (the robot is really turning
   * from then on), and whether the new road was hidden */
  unsigned long tClear = t0;
  lastTurnHidden = false;
  int bump = 0;                          /* extra speed if the turn is too slow */
  /* [E33] start with a short kick for the weak motor; the bar is watched
   * during the kick too, so a line leaving and coming back is not missed */
  spinAt(dir, TURN_SP + KICK_SP);
  while(true){
    uint8_t m = scanBar();
    unsigned long now = millis();
    unsigned long t = now - t0;
    if(t > tMax){ stopRobot(); lastTurnMs = t; lastTurnLines = passes; return false; }
    if(kicking && t >= KICK_MS){
      kicking = false;
      if(!target) spinAt(dir, TURN_SP);
    }
    /* Over white the bar cannot tell whether the robot is turning, so a
     * stalled wheel shows up only as a turn that takes too long: then add
     * speed, a little at a time (a "stall ladder"). */
    if(t > full * 13 / 10 && now - tBump > 300 && bump < 40){
      bump += 10; tBump = now;
      int u = (target ? TURN_SLOW_SP : TURN_SP) + bump;
      spinAt(dir, u);
    }
    uint8_t p = k.prevM;
    uint8_t ev = spinStep(k, dir, m, now - tClear);
    /* [E33] (the same two-readings-in-a-row rule as spinStep: no flicker) */
    m &= (uint8_t)(p | (p << 1) | (p >> 1));
    /* [E33] a line after a long blank that did not come in: it is at the
     * trailing end, so the bar has swept past a road it saw only there */
    bool lost = (k.state == 0 && m && now - tClear > full * 3 / 10);
    if(m) tClear = now;
    if(ev == SE_OUT && k.counts){
      if(target){                          /* swept past it: look for the next one */
        target = false; lost = true;
        spinAt(dir, TURN_SP + bump);
      }
      else passes ++;
    }
    /* [E33] Worn tape. Every sensor of the bar crosses the new road between
     * 9.5 and 10.4 cm from the wheels, so a gap in the tape just there hides
     * it, and the spin would go on to the next line: the wrong road. Signs
     * of that: a road that came in but vanished before the middle of the
     * bar, or a line that showed only at its trailing end after a long
     * blank (the tape either side of the gap). Then drive 5 cm on (past
     * such a gap) and turn back, so the bar crosses the road beyond the
     * gap. Only once. (Not on time alone: a slow or crooked turn on a real
     * floor takes far longer than expected and would be taken for one.) */
    if(!lastTurnHidden && lost){
      lastTurnHidden = true;
      stopRobot();
#if TRACE_ROUTE
      Serial.println(F("  turn: new road not seen (a gap?): 5 cm on, then back"));
#endif
      moveCm(GAP_BRIDGE_CM);
      dir = -dir;
      spinBegin(k, dir);
      /* the 5 cm may already have brought the road under the bar: it counts */
      if(k.state == 1){ k.counts = true; target = true; }
      spinKick(dir, target ? TURN_SLOW_SP : TURN_SP);
      kicking = false;
      tMax = millis() - t0 + full;         /* a whole turn more to find it */
      continue;
    }
    /* [E33] worn tape: in a 180 the end line to skip comes in within the
     * first quarter of the turn, the road only near its end. A first line
     * after well over half of it is the road: the end line had a gap there.
     * (60 %, not less: a weak wheel slow to start delays the end line.) */
    if(ev == SE_IN && passes < skip && turnTimed() && t > full * 60 / 100) passes = skip;
    if(ev == SE_IN && passes >= skip && t >= tMin && !target){
      target = true;                       /* the line we want is coming in: slow down */
      spinAt(dir, TURN_SLOW_SP + bump);
    }
    /* [E33] v3: how far the spin has got, for a restart */
    keepSpin(dir, skip - passes, target ? 2 : (k.state & k.counts));
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
    spinAt(dir, TURN_SLOW_SP);
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
  bool moved = false;
  for(uint8_t k=0;k<30;k++){
    delay(15);
    uint8_t m = scanBar();
    /* [E33] a black mark beside the line: of separate spots, take the
     * narrow one nearest the middle of the bar as the line */
    if(m && !isRun(m)){
      uint8_t i = 0, run, best = 0; int bd = 99;
      while((run = nextRun(m, i))){
        int d = abs(errorOfMask(run));
        if(popcount8(run) <= 3 && d < bd){ bd = d; best = run; }
      }
      if(best) m = best;
    }
    uint8_t b = popcount8(m);
    if(b >= 1 && b <= 3 && isRun(m)){
      int e = errorOfMask(m);
      if(abs(e) <= 2) return true;
      int d = (e > 0) ? 1 : -1;          /* line on the right: turn right */
      spinAt(d, TURN_SLOW_SP);
      delay(12);
      setMotor(0, 0);
    }else if(b == 0){
      /* turns stop late more often than early: look back first */
      if(!searchLine(-lastDir, t90 * 35 / 100) && !searchLine(lastDir, t90 * 70 / 100)) return false;
    }else{
      /* something wide under the bar, or two separate spots: one noisy
       * reading can do that, so only give up if it stays that way */
      if(++odd >= 3){
        /* [E33] ... but a black mark right beside the line, where the bar
         * stopped, makes one wide spot with it: drive 3 cm on, off the
         * mark, and look again (once) */
        if(moved) return false;
        moved = true; odd = 0;
        moveCm(3.0);
      }
      continue;
    }
    odd = 0;
  }
  return false;
}

void afterTurn(int dir){
  if(!centreOnLine(dir)) fault(7, F("no line under the bar after the turn"));
  stopRobot();
  logFlush();                            /* standing still: write the run log */
  clearPid();
  gridArmed = !checkGrid();
}

bool turnTimed(){                          /* can turn times be checked? */
  if(calSource == 2) return true;
  uint8_t s = calItem(calStatus, CI_TURN);
  return calSource == 1 && (s == ST_PASS || s == ST_WEAK);
}

void logTurn(int dir, int deg){
  logAdd((uint8_t)(LG_TURN | (((dir > 0) ? 1 : 0) | (deg >= 180 ? 2 : 0)) << 4), numGride,
         (int16_t)lastTurnMs, (int16_t)turnFullMs(dir, deg), lineMask, lastTurnLines);
}

void traceTurn(const __FlashStringHelper *what, int deg, int dir){
#if TRACE_ROUTE
  Serial.print(F("  ")); Serial.print(what);
  Serial.print(' '); Serial.print(lastTurnMs); Serial.print(F(" ms (expected about "));
  Serial.print(turnFullMs(dir, deg)); Serial.print(F("), lines passed ")); Serial.println(lastTurnLines);
#else
  (void)what; (void)deg; (void)dir;
#endif
}

/* [E33] With the wheels on a MID crossing, the new road is the FIRST line
 * that comes in. If a line was passed first, the robot turned too far. */
void turnBy90(int dir){
  if(!spinTurn(dir, 90, 0)) fault(6, F("90 degree turn never found the line"));
  traceTurn(dir > 0 ? F("turnRight90") : F("turnLeft90"), 90, dir);
  logTurn(dir, 90);
  if(lastTurnLines > 0) fault(10, F("90 degree turn passed a line first: turned too far"));
  afterTurn(dir);
}

/* [E33] one 180-degree spin that counts lines, instead of 90 + wait + 90:
 * at the end of a column there is no line at 90 degrees for the first half
 * to stop on, which is how the robot ended up driving along the TOP line. */
void turnBy180(int dir){
  if(!spinTurn(dir, 180, skip180)) fault(6, F("180 degree turn never found the line"));
  traceTurn(dir > 0 ? F("turnRight180") : F("turnLeft180"), 180, dir);
  logTurn(dir, 180);
  unsigned long full = (unsigned long)(((dir > 0) ? cal.t90R : cal.t90L) * 2UL / vScale);
  if(turnTimed() && !lastTurnHidden && (lastTurnMs < full / 2 || lastTurnMs > full * 2))
    fault(11, F("180 degree turn took far too short or too long: stopped on the wrong line?"));
  afterTurn(dir);
}

/* [E33] v3: finish a turn that a restart broke off. keep says how far the
 * spin had got: the lines it still had to pass, and what was under the bar
 * (a line it was passing, or already the new road). */
void resumeTurn(){
  int dir = keep.dir;
  if(keep.cur != 2){                     /* the new road had not come in yet */
    int8_t left = keep.left;
    if(keep.cur == 1) left--;            /* the line it was passing: passed */
    if(left < 0) left = 0;
    /* (as a 90: the part of the turn that is left is never more than 180) */
    if(!spinTurn(dir, 90, left)) fault(6, F("turn never found the line"));
  }
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
/* [E33] v3: the angles the servos were last sent, in the memory a reset
 * keeps: after a restart in the middle of a run beginFnc() sends the servos
 * these same angles, so a held object stays in the jaws */
int gripNow NOINIT;
int armNow  NOINIT;

/* =====================================================================
 *  [E33]  THE SUPPLY VOLTAGE (VCC), measured with no extra wire
 *  The ADC measures a voltage as a fraction of VCC (the same reference the
 *  sensors use). Pointed at the chip's own 1.1 V (the bandgap) instead of a
 *  pin, it reads 1.1 V as a fraction of VCC, so VCC = 1.1 x 1023 / reading.
 *  The 5 V regulator needs about 6.2 V from the battery: a 9 V battery that
 *  sags when the servos move shows here as VCC falling below 5 V, long
 *  before it gets as low as a brown-out (about 2.7 V). Measured only while
 *  the robot stands still (at the start, and while the servos move at a
 *  pick or a place), so line following is never disturbed.
 * ===================================================================== */
#if defined(__AVR__)
uint16_t bandgapAdc(){
  ADMUX = _BV(REFS0) | 0x0E;             /* reference stays AVcc; input = the 1.1 V */
  delayMicroseconds(200);                /* let the input settle */
  uint16_t a = 0;
  for(uint8_t k=0;k<4;k++){              /* the first readings are thrown away */
    ADCSRA |= _BV(ADSC);
    while(ADCSRA & _BV(ADSC));
    a = ADC;
  }
  /* back on a sensor channel (a throw-away read, as readNorm does), so the
   * next sensor reading is exactly as without this measurement */
  analogRead(sensorPin[0]);
  return a;
}
#else
uint16_t sim_bandgap_adc();              /* host simulator: its supply model */
uint16_t bandgapAdc(){ return sim_bandgap_adc(); }
#endif

/* VCC in mV, or 0 when the reading cannot be a real one (outside about
 * 2.5 to 6.6 V; an emulator without this input reads 0 or 1023) */
__attribute__((noinline)) uint16_t vccMv(){
  uint16_t a = bandgapAdc();
  if(a < 170 || a > 450) return 0;
  return (uint16_t)((uint32_t)BANDGAP_MV * 1023UL / a);
}

uint16_t vccRest = 0;                    /* standing still, servos not moving (0 = unknown) */
uint16_t vccLow = 0;                     /* lowest while the servos moved since then */
uint16_t vccRunLow = 0;                  /* lowest while the servos moved, whole run */

bool vccIsLow(uint16_t mv){ return mv && mv < VCC_WARN_MV; }

void vccTrack(){
  uint16_t v = vccMv();
  if(!v) return;
  if(!vccLow || v < vccLow) vccLow = v;
  if(!vccRunLow || v < vccRunLow) vccRunLow = v;
}

/* Standing still with the servos not moving: VCC at rest (8 readings
 * averaged), and a new "lowest while the servos move" starts. */
uint16_t powerRest(){
  vccLow = 0;
  uint16_t s = 0;
  for(uint8_t k=0;k<8;k++){
    uint16_t v = vccMv();
    if(!v){ s = 0; break; }
    s += v;                              /* (8 x 6619 fits) */
  }
  vccRest = s / 8;
  return vccRest;
}

/* Wait ms while a servo moves, measuring VCC all the time and keeping the
 * lowest: a servo takes its current in bursts of a few ms after each
 * pulse, and one reading at the same moment of every step could miss all
 * of them. (The simulator's supply model has no bursts: one reading, and
 * its clock runs exactly as before.) */
void powerWait(uint16_t ms){
#if defined(__AVR__)
  unsigned long t0 = millis();
  do vccTrack(); while(millis() - t0 < ms);
#else
  vccTrack();
  delay(ms);
#endif
}

/* The supply into the run log (a POWER record: not at the start, where the
 * log begins only at "go"), the live POWER line, and the warning if it was
 * low (what: 0 the start, 1 a pick, 2 / 3 a place, 4 the end; the end only
 * reports). Returns whether it was low. It never stops anything. */
bool powerShow(uint8_t what, uint16_t rest, uint16_t low){
  bool warn = vccIsLow(what ? low : rest);
  logAdd(LG_POWER | (warn ? 0x10 : 0), numGride, (int16_t)rest, (int16_t)low, 0, what);
#if TRACE_ROUTE
  labelInt(F("n="), numGride); Serial.print(' ');
  printPower(what, rest, low, warn); Serial.println();
#endif
  /* [E33] v3: the warning only (v2 also blinked the LED for 3 s here, and
   * the robot waited for it) */
  if(warn && what < 4) Serial.println(F("WARNING: low supply: weak battery (README: The battery)"));
  return warn;
}

/* The Servo library sends one pulse every 20 ms, so steps are 3 degrees per
 * 20 ms (150 degrees a second): faster steps would only arrive as jumps. */
void servoTo(Servo &s, int &now, int target){
  while(now != target){
    int d = target - now;
    if(d > 3) d = 3;
    if(d < -3) d = -3;
    now += d;
    s.write(now);
    keepSave();                          /* [E33] v3: the angle, for a restart */
    powerWait(20);                       /* [E33] delay(20), measuring the supply */
  }
}

void keepup_object(){
  servoTo(servo_x, gripNow, GRIP_CLOSED);//หนีบของ
  delay(200);
  /* [E33] hold, do not stall: open GRIP_BACKOFF_DEG again (toward GRIP_OPEN) */
  servoTo(servo_x, gripNow, GRIP_CLOSED + ((GRIP_OPEN > GRIP_CLOSED) ? GRIP_BACKOFF_DEG : -GRIP_BACKOFF_DEG));
  delay(100);
  servoTo(servo_y, armNow, ARM_CARRY);//ยกของ
  delay(300);
  keep.held = 1; keepSave();             /* [E33] v3: for a restart */
}

void put_object(){
  servoTo(servo_y, armNow, ARM_DOWN);//วางของลง
  delay(200);
  servoTo(servo_x, gripNow, GRIP_RELEASE);//คลายแขนหนีบ
  delay(200);
  keep.held = 0; keepSave();             /* [E33] v3: for a restart */
}

void arm_over_head(){
  servoTo(servo_y, armNow, ARM_HIGH);
  delay(200);
}

/* [E33] v3: is the robot held in the air? Then nothing reflects and every
 * sensor reads black. On the way to a crossing (not at a column end) the
 * bar must also see something: a line, or the crossing. 0.2 s, standing. */
bool lifted(){
  uint8_t blank = (keep.step == KS_DRIVE && caseKind(keep.n) != K_END) ? 0 : 99;
  uint8_t odd = 0;
  for(uint8_t k=0;k<10;k++){
    uint8_t b = popcount8(scanBar());
    if(b >= 7 || b == blank) odd++;
    delay(20);
  }
  return odd >= 8;
}

/* ===================================================================== */

/* ([E33] v3: noinline only because the Nano's flash is full: the compiler
 * makes the whole program about 180 bytes smaller this way) */
__attribute__((noinline)) void beginFnc(){
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
  calSanitize();
  calApply();

  printResetCause();
  printCalSource();
  bool midRun = (runMark == RUN_MARK);
#if ROBOT_MODE == MODE_MISSION
  /* [E33] v3: a restart in the middle of a run (the battery dipped when a
   * motor or a servo started) does not stop the robot: it carries on from
   * the count it kept (startMission). The chip's reset flags were read once
   * and cleared at start-up (grabResetFlags), so they are this reset's own.
   * Not carried on: a switch-on (this memory is lost, the check sum fails),
   * the reset button or USB alone (a new start on purpose), or the robot
   * lifted (lifted(): the bar sees black everywhere). On this Nano a dip of
   * the battery reports "power-on" too (all of your restarts did), so the
   * memory, not that flag, tells a dip from switching on. */
  if(midRun){
    Serial.println(F("RESTARTED IN THE MIDDLE OF A RUN (brown-out: weak battery; or reset / USB)."));
    resuming = keepValid() && resetCause() != RF_EXT;
    if(resuming && lifted()){
      resuming = false;
      Serial.println(F("Lifted: a new start."));
    }
    logRestart(resuming);                /* [E33] mark it in the run log (only a
                                          * restart after "go" belongs to a run) */
  }
  /* the next reset starts normally. (When it carries on, runMark stays: a
   * second restart before it drives on, for example when the servos take up
   * the load again below, is still this run, not a new start in the middle
   * of the field.) */
  if(!resuming){ runMark = 0; gripNow = GRIP_OPEN; armNow = ARM_DOWN; }
#else
  runMark = 0;                           /* the next reset starts normally */
  /* [E33] After a brown-out reset we do NOT start again: the robot is
   * somewhere in the middle of the field and would drive off. */
  if(brownOutReset() || midRun){
    Serial.println(F("RESTARTED IN THE MIDDLE OF A RUN (brown-out: weak battery; or reset / USB)."));
    Serial.println(F("Not driving. Fresh battery, robot at the start, then press reset."));
    while(1){ digitalWrite(LED_PIN, (millis()/100) & 1); }
  }
  gripNow = GRIP_OPEN; armNow = ARM_DOWN;
#endif

  delay(1000);

  /* [E33] write the angle BEFORE attach, so the first pulse is the angle we
   * want and not the library's 90 degrees; one servo at a time. (v3: after
   * a restart these are the angles it had, so the jaws stay as they were.) */
  servo_x.write(gripNow);//คลายแขนจับ
  servo_x.attach(x_pin);//เชื่อมต่อขาสัญญาณ
  delay(300);
  servo_y.write(armNow);//ยกแขนลง
  servo_y.attach(y_pin);//เชื่อมต่อขาสัญญาณ
  delay(300);

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
  labelInt(F("n="), numGride);
  Serial.print(' ');       Serial.print(what);
  /* whole-number printing (logTenth): Arduino's float printing is large */
  labelTenth(F(" d="), tenths(crossDist));
  labelTenth(F("/"),   tenths(crossGap));
  labelMask(F(" m="), crossMask);
  printAheadV(crossAhead, vScale100()); Serial.println();
#else
  (void)what;
#endif
}

/* The calibration mode sets this so a fault there saves what it measured
 * and prints a report instead of only stopping. */
typedef void (*FaultHook)(uint8_t code, const __FlashStringHelper *why);
FaultHook faultHook = 0;

void fault(uint8_t code, const __FlashStringHelper *why){
#if ROBOT_MODE == MODE_MISSION && !STOP_ON_FAULT
  /* [E33] STOP_ON_FAULT 0: report it (once at this count) and go on */
  static uint8_t lastCode = 0;
  static int lastN = -1;
  if(code == lastCode && numGride == lastN) return;
  lastCode = code; lastN = numGride;
  logAdd(LG_FAULT, numGride, tenths(odoCm - legRef), tenths(legGap), lineMask, code);
  Serial.print(F("FAULT ")); Serial.print(code); Serial.print(F(": ")); Serial.println(why);
  labelInt(F("  at n="), numGride);
  labelMask(F("  bar "), lineMask);
  Serial.println(F("  (STOP_ON_FAULT 0: it goes on)"));
  return;
#endif
  stopRobot();
  digitalWrite(STBY,0);
  runMark = 0;                           /* stopped on purpose, not a restart */
#if ROBOT_MODE == MODE_MISSION
  logAdd(LG_FAULT, numGride, tenths(odoCm - legRef), tenths(legGap), lineMask, code);
  logFlush();
  logEnd(LS_FAULT);
#endif
  if(faultHook) faultHook(code, why);
  Serial.print(F("FAULT ")); Serial.print(code); Serial.print(F(": ")); Serial.println(why);
  labelInt(F("  at n="), numGride);
  labelMask(F("  bar "), lineMask);
  labelInt(F("  d="), (int)(odoCm - legRef)); Serial.print('/'); Serial.println((int)legGap);
  Serial.println(F("  (the LED on D13 blinks the fault number)"));
  while(1){
    for(uint8_t k=0;k<code;k++){ digitalWrite(LED_PIN,1); delay(200); digitalWrite(LED_PIN,0); delay(200); }
    delay(1000);
  }
}

#endif
