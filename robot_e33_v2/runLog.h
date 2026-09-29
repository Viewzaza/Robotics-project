/* =====================================================================
 *  runLog.h: a record of the last run, kept in EEPROM   [E33]
 * =====================================================================
 *
 *  While the mission runs, every crossing, turn, pick, place and fault is
 *  written down in a few bytes ([E33] and the supply voltage: the POWER
 *  lines, at the start, at each pick and place, and at the end). The next
 *  time the Nano starts in
 *  MODE_MISSION (for example when you plug in USB and open the Serial
 *  Monitor) it prints the record of the last run. So a run without the
 *  cable can still be read afterwards: copy everything from BEGIN to END
 *  and send it.
 *
 *  Writing one byte of EEPROM takes about 3.4 ms, too long while driving,
 *  so records wait in memory and are written only while the robot is
 *  stopped (after a turn, before a pick or a place, at a fault, at the end).
 *
 *  The record of the last run is kept until a new run has really moved: it
 *  is replaced at the new run's first stop (its first turn). So switching on
 *  with the motors off, or on the line for a moment, does not wipe it. The
 *  record also says how the run ended: normally, with a fault, with the
 *  power switched off, or with the Nano restarting in the middle.
 *
 *  EEPROM use: 0x000-0x0FF calibration, 0x100 re-arm flag, 0x140-0x3FF log.
 * ===================================================================== */
#ifndef RUNLOG_H
#define RUNLOG_H

#define LOG_ADDR   0x140
#define LOG_MAGIC1 0xE3
#define LOG_MAGIC2 0x4C
#define LOG_RECSZ  10
#define LOG_MAX    ((0x400 - LOG_ADDR - 4) / LOG_RECSZ)   /* 70 records */
#define LOG_BUF    12

/* header: magic, magic, count, how the run ended */
#define LS_ENDED   0          /* ended normally (or no run yet)              */
#define LS_RUNNING 1          /* did not end: the power was switched off or lost */
#define LS_RESTART 2          /* the Nano restarted in the middle of the run */
#define LS_FAULT   3          /* stopped with a fault                        */
#define LS_CUT     4          /* was still LS_RUNNING when the next run started:
                               * the power was lost (a restart of the new run
                               * must not be written into this record)       */

struct LogRec {
  uint8_t  type;      /* low 4 bits: what happened; high 4 bits: extra   */
  uint8_t  n;         /* numGride                                         */
  uint16_t t10;       /* time since "go", in 0.1 s                        */
  int16_t  a, b;      /* two numbers, see logPrint()                      */
  uint8_t  mask;      /* sensors                                          */
  uint8_t  x;         /* one more small number                            */
};

/* the record size is part of the EEPROM layout: change LOG_MAGIC2 if it changes */
static_assert(sizeof(LogRec) == LOG_RECSZ, "LogRec must be LOG_RECSZ bytes");

#define LG_START   1
#define LG_CROSS   2
#define LG_REJECT  3
#define LG_CREDIT  4
#define LG_ENDLOST 5
#define LG_TURN    6
#define LG_ACTION  7
#define LG_RETRY   8
#define LG_FAULT   9
#define LG_DONE    10
#define LG_RESTART 11         /* written at start-up after a restart mid-run */
#define LG_POWER   12         /* [E33] the supply (VCC) in mV: a = at rest, b = lowest
                               * while the servos moved (the end: of the whole run),
                               * x = 0 start, 1 pick, 2 place, 3 last place, 4 end;
                               * high bits 1 = below VCC_WARN_MV */

/* [E33] FAULT and DONE end a run: the last place is kept for them (a
 * POWER record has a higher number but is an ordinary one) */
bool logEnds(uint8_t type){ type &= 0x0F; return type == LG_FAULT || type == LG_DONE; }

LogRec  logBuf[LOG_BUF];
uint8_t logN = 0;           /* waiting in memory      */
uint8_t logCount = 0;       /* already in EEPROM      */
bool    logOn = false;
bool    logFresh = false;   /* the old record is still in EEPROM, not replaced yet */
unsigned long logT0 = 0;

bool logValid(){
  return EEPROM.read(LOG_ADDR) == LOG_MAGIC1 && EEPROM.read(LOG_ADDR + 1) == LOG_MAGIC2;
}

/* How did the run in EEPROM end? */
uint8_t logLastState(){
#if RUN_LOG
  if(!logValid() || EEPROM.read(LOG_ADDR + 2) == 0) return LS_ENDED;
  return EEPROM.read(LOG_ADDR + 3);
#else
  return LS_ENDED;
#endif
}

/* Start recording a new run (called at "MISSION: go"). Nothing is written
 * yet: the old record stays until this run's first stop. */
void logBegin(){
#if RUN_LOG
  logN = 0; logOn = true; logFresh = true; logT0 = millis();
  if(EEPROM.read(LOG_ADDR + 3) == LS_RUNNING) EEPROM.update(LOG_ADDR + 3, LS_CUT);
#endif
}

/* [E33] a POWER record gets the time of the record before it (it belongs to
 * the START, PICK, PLACE or DONE just before it) and does not read the clock:
 * in the simulator a clock read takes time, and the runs must stay the same */
uint16_t logT10 = 0;
void logAdd(uint8_t type, int n, int16_t a, int16_t b, uint8_t mask, uint8_t x){
#if RUN_LOG
  if(!logOn || logN >= LOG_BUF) return;
  /* the last place is kept for the FAULT or DONE that ends the run */
  if(logN == LOG_BUF - 1 && !logEnds(type)) return;
  LogRec &r = logBuf[logN++];
  r.type = type; r.n = (uint8_t)n;
  if((type & 0x0F) != LG_POWER) logT10 = (uint16_t)((millis() - logT0) / 100);
  r.t10 = logT10;
  r.a = a; r.b = b; r.mask = mask; r.x = x;
#else
  (void)type; (void)n; (void)a; (void)b; (void)mask; (void)x;
#endif
}

/* Write what is waiting to EEPROM. Only call it while the robot stands still. */
void logFlush(){
#if RUN_LOG
  if(!logOn || logN == 0) return;
  if(logFresh){                          /* the first stop of this run: replace the old record */
    EEPROM.update(LOG_ADDR, LOG_MAGIC1);
    EEPROM.update(LOG_ADDR + 1, LOG_MAGIC2);
    EEPROM.update(LOG_ADDR + 2, 0);
    EEPROM.update(LOG_ADDR + 3, LS_RUNNING);
    logCount = 0;
    logFresh = false;
  }
  for(uint8_t i=0;i<logN && logCount < LOG_MAX;i++){
    /* the same for the last place in EEPROM */
    if(logCount == LOG_MAX - 1 && !logEnds(logBuf[i].type)) continue;
    EEPROM.put(LOG_ADDR + 4 + logCount * LOG_RECSZ, logBuf[i]);
    logCount++;
  }
  logN = 0;
  EEPROM.update(LOG_ADDR + 2, logCount);
#endif
}

/* The run is over (after its last logFlush): remember how it ended. */
void logEnd(uint8_t state){
#if RUN_LOG
  if(logOn && !logFresh) EEPROM.update(LOG_ADDR + 3, state);
  logOn = false;
#else
  (void)state;
#endif
}

/* At start-up after a restart in the middle of a run: note it in the record
 * of that run (if that run had got as far as its first stop). */
void logRestart(){
#if RUN_LOG
  if(!logValid() || EEPROM.read(LOG_ADDR + 3) != LS_RUNNING) return;
  uint8_t c = EEPROM.read(LOG_ADDR + 2);
  if(c < LOG_MAX){
    LogRec r;
    r.type = LG_RESTART; r.n = 0; r.t10 = 0; r.a = 0; r.b = 0; r.mask = 0; r.x = 0;
    EEPROM.put(LOG_ADDR + 4 + c * LOG_RECSZ, r);
    EEPROM.update(LOG_ADDR + 2, c + 1);
  }
  EEPROM.update(LOG_ADDR + 3, LS_RESTART);
#endif
}

/* (the same printers the trace uses, in controlLibrary.h) */
void printAhead(uint8_t a);
void printMask(uint8_t m);

void printHund(int v){                /* 112 -> 1.12 */
  Serial.print(v / 100); Serial.print('.');
  if(v % 100 < 10) Serial.print('0');
  Serial.print(v % 100);
}

void logTenth(int16_t v){             /* 123 -> 12.3 */
  if(v < 0){ Serial.print('-'); v = -v; }
  Serial.print(v / 10); Serial.print('.'); Serial.print(v % 10);
}

/* [E33] a label and the number after it: one call instead of two, which
 * over all the log and trace lines saves a lot of flash */
void labelTenth(const __FlashStringHelper *l, int16_t v){ Serial.print(l); logTenth(v); }
void labelHund(const __FlashStringHelper *l, int v){ Serial.print(l); printHund(v); }
void labelInt(const __FlashStringHelper *l, int v){ Serial.print(l); Serial.print(v); }
void labelMask(const __FlashStringHelper *l, uint8_t m){ Serial.print(l); printMask(m); }
/* [E33] pairs printed in more than one place: each text is stored once */
void printDistExpected(int16_t d10, int16_t e10){ labelTenth(F(" d="), d10); labelTenth(F(" expected="), e10); }
void printAheadV(uint8_t a, int v100){ Serial.print(F(" ahead=")); printAhead(a); labelHund(F(" v="), v100); }

/* [E33] volts from mV, two decimals ("?" = could not be measured) */
void labelVolt(const __FlashStringHelper *l, uint16_t mv){
  Serial.print(l);
  if(mv){ printHund((mv + 5) / 10); Serial.print('V'); }
  else Serial.print('?');
}
/* [E33] one POWER line, the same in the run log and in the live trace:
 * "POWER pick rest=4.98V min=4.61V" (LOW = below VCC_WARN_MV) */
void printPower(uint8_t what, uint16_t rest, uint16_t low, bool warn){
  Serial.print(F("POWER "));
  Serial.print(what == 0 ? F("start") : what == 1 ? F("pick") : what == 4 ? F("end") : F("place"));
  labelVolt(F(" rest="), rest);
  if(what) labelVolt(what == 4 ? F(" lowest=") : F(" min="), low);
  if(warn) Serial.print(F(" LOW"));
}

/* Print the record of the last run, if there is one. */
void logPrint(){
#if RUN_LOG
  if(EEPROM.read(LOG_ADDR) != LOG_MAGIC1 || EEPROM.read(LOG_ADDR + 1) != LOG_MAGIC2) return;
  uint8_t c = EEPROM.read(LOG_ADDR + 2);
  if(c == 0 || c > LOG_MAX) return;
  Serial.println(F("=== E33 RUN LOG BEGIN (last run) ==="));
  for(uint8_t i=0;i<c;i++){
    LogRec r;
    EEPROM.get(LOG_ADDR + 4 + i * LOG_RECSZ, r);
    uint8_t ty = r.type & 0x0F, hi = r.type >> 4;
    if(ty == LG_RESTART){
      Serial.println(F("RESTARTED here (brown-out, or reset / USB)"));
      continue;
    }
    labelTenth(F("t="), (int16_t)r.t10);
    labelInt(F(" n="), r.n); Serial.print(' ');
    switch(ty){
      case LG_START:
        labelInt(F("START cal="), r.x);
        labelInt(F(" run#"), r.a);
        labelInt(F(" light="), r.b);
        if(hi) Serial.print(F(" route=132"));
        break;
      case LG_CROSS: case LG_REJECT: case LG_CREDIT: case LG_ENDLOST:
        Serial.print(ty == LG_CROSS ? F("CROSS") : ty == LG_REJECT ? F("REJECT(too early)")
                   : ty == LG_CREDIT ? F("MISSED-ONE(counted)") : F("END(line ran out)"));
        printDistExpected(r.a, r.b);
        labelMask(F(" mask="), r.mask);
        printAheadV(hi, r.x);
        break;
      case LG_TURN:
        Serial.print(F("TURN ")); Serial.print((hi & 1) ? F("RIGHT ") : F("LEFT "));
        Serial.print((hi & 2) ? F("180") : F("90"));
        labelInt(F(" ms="), r.a); labelInt(F(" expected="), r.b);
        labelInt(F(" lines-passed="), r.x);
        labelMask(F(" end-mask="), r.mask);
        break;
      case LG_ACTION:
        Serial.print(r.x == 1 ? F("PICK") : r.x == 2 ? F("PLACE") : F("PLACE(last)"));
        labelTenth(F(" bar-past-line="), r.a); labelTenth(F(" target="), r.b);
        break;
      case LG_RETRY:
        labelTenth(F("RETRY(line ahead at an end) d="), r.a);
        break;
      case LG_FAULT:
        labelInt(F("FAULT "), r.x);
        printDistExpected(r.a, r.b);
        labelMask(F(" bar="), r.mask);
        break;
      case LG_DONE:
        labelInt(F("DONE rejected="), r.a);
        labelInt(F(" missed-counted="), r.b);
        labelInt(F(" retries="), r.x);
        labelHund(F(" speed=x"), r.mask);
        if(hi && r.mask < 80) Serial.print(F(" (battery weaker than at calibration)"));
        break;
      case LG_POWER:
        printPower(r.x, (uint16_t)r.a, (uint16_t)r.b, hi & 1);
        break;
      default:
        Serial.print('?');
    }
    Serial.println();
  }
  if(c >= LOG_MAX - 1) Serial.println(F("(log full: later events were not kept)"));
  uint8_t st = EEPROM.read(LOG_ADDR + 3);
  if(st == LS_RUNNING || st == LS_CUT)
    Serial.println(F("(the run did not end: the power was switched off or lost)"));
  Serial.println(F("=== E33 RUN LOG END ==="));
#endif
}

#endif
