/* =====================================================================
 *  calibrationMode.h: automatic calibration, sensor meter, motor check
 * =====================================================================
 *
 *  Only used when ROBOT_MODE in calibration.h is not MODE_MISSION.
 *
 *  MODE_CALIBRATE  (about 90 s, nothing to press)
 *    Place the robot exactly as for the mission: wheels (axle) over the C1
 *    line, on the MID line, facing east. Switch on and step back. When the
 *    bar has seen the line steadily for 3 s the robot:
 *      1. reads the sensors standing still           (which way is black)
 *      2. nudges ONE wheel at a time, very slowly    (dead band, each motor,
 *                                                     forward and backward,
 *                                                     and which way it turns)
 *      3. swings the bar left and right over the line (white and black of
 *                                                     every channel)
 *      4. drives MID to C4 slowly, spins 540 degrees on C4, drives back to
 *         C1 at walking speed, spins again, and does it once more.
 *         On the way it adds up how much speed the PID gave each wheel
 *         between crossings: to drive straight, the stronger wheel must
 *         have been given less. That is the left/right mismatch, and it is
 *         corrected after every cell.
 *         The time between crossings gives the speed; the spins give the
 *         time per 90 degrees. A spin LEFT runs the left wheel backward and
 *         the right forward; a spin RIGHT the other way round. If one takes
 *         longer, one wheel is weaker BACKWARD, and the backward trims are
 *         corrected so both spins match (this is the "turning back" case).
 *    It ends where it started, saves everything to EEPROM, and prints a
 *    report and a block you can paste into calibration.h.
 *    It only ever drives on MID between C1 and C4, so the objects can stay
 *    on the field.
 *
 *  MODE_METER        prints what the sensors see; never moves
 *  MODE_MOTOR_CHECK  wheels OFF the ground: runs each motor in turn
 * ===================================================================== */
#ifndef CALIBRATIONMODE_H
#define CALIBRATIONMODE_H

#define PITCH_CM   1.2     /* sensor spacing on the bar (only sets step sizes) */
#define TRACK_CM  11.0     /* distance between the wheels (only sets step sizes) */
#define SWEEP_SP   45      /* spin speed for the sensor sweep                  */

uint16_t newStatus = 0;
uint8_t  curItem = CI_SENSOR;
const __FlashStringHelper *calPhase = 0;
bool     calAborted = false;

float    burstM[8];   uint8_t nBurst = 0;      /* left/right effort per cell - 1 */
float    kMove[2];    uint8_t nKMove = 0;     /* cm per (speed unit x s) */
float    kCruise[6];  uint8_t nKCruise = 0;
uint16_t t90Ls[2], t90Rs[2];
uint8_t  nT90L = 0, nT90R = 0;
int16_t  lagMs = -1;
float    trimLf = 1000, trimRf = 1000;
float    revL = 1.0, revR = 1.0;           /* backward trim = forward trim x this */
float    spinX[2]; uint8_t nSpinX = 0;     /* spin left/right mismatch per pair   */
float    kEst = 0.13;

void setStatus(uint8_t item, uint8_t s){
  newStatus = (newStatus & ~(3u << (2*item))) | ((uint16_t)s << (2*item));
}

/* Prints v with d decimals using whole numbers only: the Nano's flash is
 * nearly full in this mode and Arduino's float printing is large. */
void printF1(float v, uint8_t d){
  long scale = (d >= 2) ? 100 : (d == 1 ? 10 : 1);
  long x = (long)(v * scale + (v < 0 ? -0.5 : 0.5));
  if(x < 0){ Serial.print('-'); x = -x; }
  Serial.print(x / scale);
  if(d){
    Serial.print('.');
    long f = x % scale;
    if(d >= 2 && f < 10) Serial.print('0');
    Serial.print(f);
  }
}

/* ------------------------------------------------------------------ */
/*  1. Standing still: is there ONE line under the middle of the bar?  */
/* ------------------------------------------------------------------ */
/* rawLook() and calFromLook() are in controlLibrary.h (the mission uses them too). */

void printRaw(const uint16_t *v){
  for(uint8_t i=0;i<8;i++){ Serial.print(v[i]); Serial.print(' '); }
}

void calReport(bool saved, uint16_t seq, int addr);

void calWaitPlacement(RawLook &r){
  bool needLift = EEPROM.read(CAL_REARM) == 1;
  if(needLift){
    /* [E33] Opening the Serial Monitor restarts the Nano, and the report is
     * printed only once, at the end of the run. So print the saved one again. */
    CalRec s;
    int slot = calReadEeprom(s);
    if(slot){
      CalData keep = cal;
      cal = s.d; newStatus = s.status;
      calAborted = s.spare; curItem = s.spare - 1; calPhase = 0;   /* [E33] a run that stopped says so again */
      calReport(true, s.seq, slot - 1);
      cal = keep; newStatus = 0; calAborted = false;
    }
    Serial.println(F("\nTo calibrate again: lift the robot, then put it at the start."));
  }
  else Serial.println(F("Put the robot at the start: wheels over C1, on MID, facing east. Then step back."));
  unsigned long tOk = 0, tOff = 0, tPrint = 0;
  while(true){
    bool ok = rawLook(r);
    unsigned long now = millis();
    if(needLift){
      if(!ok){
        if(!tOff) tOff = now;
        if(now - tOff > 1000){ needLift = false; Serial.println(F("Lifted. Now put it down at the start.")); }
      }else tOff = 0;
      digitalWrite(LED_PIN, (now / 1000) & 1);
      continue;
    }
    if(ok){
      if(!tOk){ tOk = now; Serial.println(F("Line seen. Starting in 3 s: hands off.")); }
      digitalWrite(LED_PIN, 1);
      if(now - tOk > 3000) break;
    }else{
      tOk = 0;
      digitalWrite(LED_PIN, (now / 500) & 1);
      if(now - tPrint > 1000){
        tPrint = now;
        Serial.print(F("waiting: raw A0..A7: ")); printRaw(r.v);
        Serial.print(F(" biggest step ")); Serial.print(r.gap);
        Serial.println(F(" (need one line under the middle, 1-3 channels)"));
      }
    }
  }
  digitalWrite(LED_PIN, 0);
}

/* ------------------------------------------------------------------ */
/*  2. Dead band: one wheel at a time, the other braked                */
/* ------------------------------------------------------------------ */
int avgCentroid(uint8_t n){
  long s = 0; uint8_t k = 0;
  for(uint8_t j=0; j<2*n && k<n; j++){
    scanBar();
    int c = centroid256();
    if(c >= 0){ s += c; k++; }
    delay(2);
  }
  if(k == 0 || k < n/2) return -1;
  return (int)(s / k);
}

/* Raise the raw duty of one wheel until the line moves under the bar.
 * expect = which way the line should move (+1 toward A7). Returns the duty,
 * 0 if nothing moved, 255 if it moved the wrong way. */
uint8_t rampOnce(uint8_t side, int8_t dir, int8_t expect){
  int c0 = avgCentroid(24);
  if(c0 < 0) return 0;
  motorRaw(1 - side, 0, 0);
  uint8_t hits = 0;
  int8_t lastSgn = 0;
  for(uint16_t d = 6; d <= 220; d++){
    motorRaw(side, dir, (uint8_t)d);
    /* average the line position over this whole step: noise cancels out,
     * a real movement does not */
    long sum = 0; uint16_t k = 0;
    unsigned long t0 = millis();
    while(millis() - t0 < 35){
      scanBar();
      int c = centroid256();
      if(c >= 0){ sum += c; k++; }
    }
    if(k < 3){ hits = 0; continue; }
    int dc = (int)(sum / k) - c0;
    int8_t sgn = (dc > 0) ? 1 : -1;
    if((dc >= 38 || dc <= -38) && (hits == 0 || sgn == lastSgn)){   /* 0.15 of a sensor step */
      lastSgn = sgn;
      if(++hits >= 3){
        motorRaw(side, 0, 0);
        delay(300);
        return (sgn == expect) ? (uint8_t)d : 255;
      }
    }else hits = 0;
  }
  motorRaw(side, 0, 0);
  return 0;
}

/* One wheel, with one retry before blaming the wiring. */
uint8_t rampWheel(uint8_t side, int8_t dir, int8_t expect){
  uint8_t d = rampOnce(side, dir, expect);
  if(d == 255){ delay(300); d = rampOnce(side, dir, expect); }
  return d;
}

void calAbortWhy(const __FlashStringHelper *why);

void measureDeadband(){
  calPhase = F("dead band"); curItem = CI_DEADBD;
  Serial.println(F("step 2: dead band (one wheel at a time, very slowly)"));
  /* left forward turns the robot right about the braked right wheel, so the
   * line moves toward A0: expect -1. And so on. */
  const int8_t  side[4]   = { 0,  0,  1,  1 };
  const int8_t  dir[4]    = { 1, -1,  1, -1 };
  const int8_t  expect[4] = {-1,  1,  1, -1 };
  uint8_t d[4];
  for(uint8_t k=0;k<4;k++){
    d[k] = rampWheel(side[k], dir[k], expect[k]);
    Serial.print(side[k] ? F("  right ") : F("  left  "));
    Serial.print(dir[k] > 0 ? F("forward  ") : F("backward "));
    if(d[k] == 0){
      Serial.println(F("did NOT move by duty 220"));
      calAbortWhy(F("that motor did not move: motor switch, battery, STBY or wiring"));
    }
    if(d[k] == 255){
      Serial.println(F("moved the WRONG way"));
      calAbortWhy(F("that wheel turns backwards: set LEFT_ or RIGHT_REVERSED to 1"));
    }
    Serial.print(F("starts at duty ")); Serial.println(d[k]);
  }
  cal.dbLf = d[0]; cal.dbLr = d[1]; cal.dbRf = d[2]; cal.dbRr = d[3];
  bool ok = true;
  for(uint8_t k=0;k<4;k++) if(d[k] < 8 || d[k] > 180) ok = false;
  setStatus(CI_DEADBD, ok ? ST_PASS : ST_WEAK);
}

/* ------------------------------------------------------------------ */
/*  3. Sensor sweep: swing the bar over the line, keep min and max      */
/* ------------------------------------------------------------------ */
uint16_t sweepF[8];                      /* each channel smoothed over ~4 scans */
bool sweepFirst = true;

bool sweepTo(int dir, bool untilCentre, uint16_t *mn, uint16_t *mx){
  setMotor(dir * SWEEP_SP, -dir * SWEEP_SP);
  unsigned long t0 = millis(), tBlank = 0;
  int lastC = -1;
  while(millis() - t0 < 4000){
    scanBar();
    /* [E33] keep the lowest and highest of a smoothed reading: the single
     * lowest and highest raw readings are noise spikes, and on a noisy bar
     * they spread the levels so white starts to look like line */
    for(uint8_t i=0;i<8;i++){
      if(sweepFirst) sweepF[i] = rawV[i];
      else sweepF[i] = (uint16_t)((3u * sweepF[i] + rawV[i] + 2) / 4);
      if(sweepF[i] < mn[i]) mn[i] = sweepF[i];
      if(sweepF[i] > mx[i]) mx[i] = sweepF[i];
    }
    sweepFirst = false;
    int c = centroid256();
    if(untilCentre){
      if(c >= 0 && abs(c - 896) < 77){ setMotor(0, 0); return true; }
      continue;
    }
    if(c >= 0){ lastC = c; tBlank = 0; }
    else if(popcount8(lineMask) == 0){
      /* spinning right moves the line toward A0; left toward A7 */
      bool offEnd = (dir > 0) ? (lastC >= 0 && lastC < 320) : (lastC > 1472);
      /* [E33] last seen in the outer part of the bar: the two end channels
       * may be too weak for the starting levels to see the line (a bar
       * mounted a little tilted). Turn a little longer so they still get
       * their black, then stop. */
      bool outer = (dir > 0) ? (lastC >= 0 && lastC < 640) : (lastC > 1152);
      if(offEnd || outer){
        if(!tBlank) tBlank = millis();
        if(millis() - tBlank > (offEnd ? 40UL : 400UL)){ setMotor(0, 0); return true; }
      }
    }
  }
  setMotor(0, 0);
  return false;
}

void sweepLost(){ calAbortWhy(F("the sensor sweep lost the line")); }

void measureSensors(){
  calPhase = F("sensor sweep"); curItem = CI_SENSOR;
  Serial.println(F("step 3: sensor sweep (swinging the bar over the line)"));
  uint16_t mn[8], mx[8];
  for(uint8_t i=0;i<8;i++){ mn[i] = 1023; mx[i] = 0; }
  bool ok = sweepTo(+1, false, mn, mx);
  delay(200);
  ok = ok && sweepTo(-1, false, mn, mx);
  if(!ok) sweepLost();
  uint8_t weak = 0, dead = 0, deadMask = 0;
  for(uint8_t i=0;i<8;i++){
    uint16_t span = (mx[i] > mn[i]) ? mx[i] - mn[i] : 0;
    if(span < 100){ dead++; deadMask |= (0x80 >> i); }
    else if(span < 200) weak++;
    cal.lo[i] = mn[i];
    cal.hi[i] = mx[i];
  }
  if(dead >= 3) calAbortWhy(F("3 or more sensors barely see the line: check the bar height"));
  if(deadMask & 0x3C) Serial.println(F("  WARNING: a MIDDLE sensor hardly sees the line. Fix its height or wiring first."));
  cal.deadMask = deadMask;
  calApply();
  /* [E33] come back to the middle with the levels just measured: the
   * standing-still levels can be blind to a weak channel that sat on the
   * line at the start, and then the line never shows in the middle */
  delay(200);
  if(!sweepTo(+1, true, mn, mx)) sweepLost();
  setStatus(CI_SENSOR, (dead || weak) ? ST_WEAK : ST_PASS);
  if(!centreOnLine(1)) calAbortWhy(F("lost the line after the sensor sweep"));
}

/* ------------------------------------------------------------------ */
/*  4. Legs along MID, measuring the left/right match; spins at the ends */
/* ------------------------------------------------------------------ */

/* The left/right match, measured with the PID itself.
 * Between two crossings the robot starts and ends heading along the same
 * straight line, so over that cell it turned left exactly as much as right.
 * If one wheel is stronger, the PID must have given it less speed, on
 * average, to keep the robot on the line. So
 *     (total speed given to the right) / (total given to the left)
 * over one cell is exactly the correction the trims still need. It does
 * not matter where the line sat under the bar, or how the bar is mounted. */
float effL0 = 0, effR0 = 0;

void startCell(){ effL0 = odoEffL; effR0 = odoEffR; }

void endCell(){
  float dL = odoEffL - effL0, dR = odoEffR - effR0;
  if(dL < 1 || dR < 1) return;
  float r = dR / dL;                     /* < 1: the right wheel was held back */
  if(nBurst < 8) burstM[nBurst++] = r - 1;
  if(r > 2.0) r = 2.0;
  if(r < 0.5) r = 0.5;
  /* straight when trimR/trimL = (trimR/trimL now) x r; take most of the step */
  float ratio = (trimRf / trimLf) * (1 + 0.8 * (r - 1));
  if(ratio <= 1){ trimLf = 1000; trimRf = 1000 * ratio; }
  else          { trimRf = 1000; trimLf = 1000 / ratio; }
  cal.trimL = (uint16_t)(trimLf + 0.5);
  cal.trimR = (uint16_t)(trimRf + 0.5);
}

struct LegRes { uint8_t n; float uE[3]; float uStart; };   /* uE < 0: a crossing not seen */

/* Follow MID for three crossings. Records where each one ends. */
/* firstCell: also measure the cell from the start to the first crossing
 * (only when the robot starts lined up on the line, as at the very start). */
void runLeg(bool fixedSpeed, int speed, bool loose, LegRes &L, bool firstCell = false){
  L.n = 0;
  clearPid();
  sp = SP_START; maxSp = speed; tLost = 0;
  gridArmed = !checkGrid();
  L.uStart = odoU;
  startCell();
  unsigned long tStart = millis();
  float d0 = CELL_CM - SENSOR_AHEAD_CM + TAPE_W_CM / 2;
  float lastU = L.uStart, lastGap = d0;                  /* last crossing seen, and how far the next is */
  while(true){
    if(millis() - tStart > 20000) calAbortWhy(F("a leg took more than 20 s (motors switched on?)"));
    /* [E33] leg 1 steers with steerAt(), which drives straight on with no
     * line. Placed a cell off or facing west it would leave the field */
    if(fixedSpeed && millis() - tLineSeen > 1000) calAbortWhy(F("line lost: was it at the start, facing east?"));
    if(!gridArmed){
      if(!checkGrid()) gridArmed = true;
    }else if(checkGrid()){
      int s = fixedSpeed ? speed : sp;
      setMotor(s, s);                                     /* straight over it */
      uint8_t clear = 0;
      unsigned long tIn = millis();
      while(clear < 2){
        if(checkGrid()) clear = 0; else clear++;
        if(millis() - tIn > CROSS_MAX_MS) calAbortWhy(F("stuck on a crossing"));
      }
      /* CHECK: each crossing where it should be (first one: from the start pose) */
      float r = (odoU - lastU) * kEst / lastGap;
      if(r < (loose ? 0.25 : 0.45)) continue;             /* the same tape seen twice */
      bool missed = false;
      /* the first crossing of a leg includes getting up to speed from rest,
       * so it may come later on the odometer than the distance says */
      if(r > (loose ? 3.0 : (L.n == 0 ? 2.2 : 1.6))){
        Serial.print(F("  crossing ")); Serial.print(L.n + 1);
        Serial.print(F(" at ")); printF1(r, 2); Serial.println(F(" x the expected distance"));
        /* [E33] about one cell too far between two crossings: one was
         * missed (like the mission, count it and go on; that cell is not
         * measured) */
        if(loose || r > 2.6 || L.n != 1)
          calAbortWhy(F("a crossing at the wrong distance: robot not at the start, or a crossing missed"));
        L.uE[L.n++] = -1;
        missed = true;
      }
      L.uE[L.n] = odoU;
      edgeOdo = odoCm;
      if(!missed && (L.n >= 1 || firstCell)) endCell();   /* a whole cell, crossing to crossing */
      lastU = odoU; lastGap = CELL_CM;
      L.n++;
      if(L.n == 3) return;                                /* still rolling */
      startCell();
      continue;
    }
    if(fixedSpeed) steerAt(speed); else followLine();
  }
}

/* Spin 540 degrees on a crossing, timing each arm as it passes the middle
 * of the bar. Ends facing the other way, on the line. */
bool spinMeasure(int dir, uint16_t &t90){
  SpinTrack k;
  spinBegin(k, dir);
  unsigned long tA0 = 0, tPrev = 0, g1 = 0, t5 = 0;
  uint8_t steps = 0, steps5 = 0;
  bool slow = false;
  unsigned long t0 = millis();
  unsigned long tMax = (unsigned long)((dir > 0) ? cal.t90R : cal.t90L) * 6UL * 5UL;   /* slow robots too */
  spinKick(dir, TURN_SP);
  while(true){
    uint8_t m = scanBar();
    if(millis() - t0 > tMax){ stopRobot(); return false; }
    uint8_t ev = spinStep(k, dir, m);
    if(ev == SE_IN && steps >= 5 && !slow){
      slow = true;                                        /* last arm: come in slowly */
      setMotor(dir * TURN_SLOW_SP, -dir * TURN_SLOW_SP);
    }
    if(ev == SE_MIDDLE){
      unsigned long now = micros();
      if(steps == 0){ steps = 1; tA0 = now; }
      else{
        unsigned long g = now - tPrev;
        /* [E33] an arm impossibly soon after the last one is noise, not an arm */
        unsigned long guess = (unsigned long)((dir > 0) ? cal.t90R : cal.t90L) * 1000UL;
        if(g1 == 0 ? (g < guess * 35 / 100) : (g < g1 * 3 / 10)) continue;   /* (arms are 90 +- a turn-centre shift apart) */
        if(g1 == 0) g1 = g;                               /* first gap is 90 degrees */
        /* [E33] 2 (an arm missing) only if the gap is 1.75 x the average gap so
         * far. A turning centre off the crossing (the robot stopped a little
         * past it, a caster) makes the gaps short, short, long, long: judged
         * against the first, short gap alone, a long one counted 2, the spin
         * stopped one arm early and the next leg went up a column. */
        unsigned long avg = (steps > 1) ? (tPrev - tA0) / (steps - 1) : g1;
        steps += (4 * g > 7 * avg) ? 2 : 1;
      }
      tPrev = now;
      if(steps >= 5 && !t5){ t5 = now; steps5 = steps; }
      if(steps >= 6) break;
    }
  }
  stopRobot();
  if(!t5 || steps5 < 3) return false;
  t90 = (uint16_t)((t5 - tA0) / 1000UL / (steps5 - 1));
  return true;
}

/* Backward trims = forward trims x revL / revR, the stronger side scaled
 * so neither goes above 1000. */
void applyReverseTrims(){
  float l = trimLf * revL, r = trimRf * revR;
  float top = (l > r) ? l : r;
  if(top > 1000){ l = l * 1000 / top; r = r * 1000 / top; }
  cal.trimLr = (uint16_t)(l + 0.5);
  cal.trimRr = (uint16_t)(r + 0.5);
}

/* After one spin each way: a spin LEFT uses left-backward + right-forward,
 * a spin RIGHT uses left-forward + right-backward. With the forward trims
 * matched, a difference in time means the two BACKWARD speeds differ:
 *   x = 2 (T90R - T90L) / (T90L + T90R)   (x < 0: left backward is weaker)
 * Correct most of it; the next pair of spins checks the result. */
void matchSpins(){
  if(nT90L == 0 || nT90R == 0) return;
  float tl = t90Ls[nT90L - 1], tr = t90Rs[nT90R - 1];
  float x = 2.0 * (tr - tl) / (tl + tr);
  if(nSpinX < 2) spinX[nSpinX++] = x;
  if(x >  0.3) x =  0.3;
  if(x < -0.3) x = -0.3;
  revL *= (1 - 0.8 * x);
  revR *= (1 + 0.8 * x);
  float mean = (revL + revR) / 2;
  revL /= mean; revR /= mean;
  applyReverseTrims();
  Serial.print(F("  spin match: left/right difference ")); printF1(x * 100, 1);
  Serial.print(F("%  -> backward trims left ")); Serial.print(cal.trimLr);
  Serial.print(F(" right ")); Serial.println(cal.trimRr);
}

void spinAt(int dir, const __FlashStringHelper *where){
  calPhase = where; curItem = CI_TURN;
  applyReverseTrims();
  uint16_t t = 0;
  if(!spinMeasure(dir, t)) calAbortWhy(F("the spin did not see the lines it expected"));
  if(dir > 0){ if(nT90R < 2) t90Rs[nT90R++] = t; cal.t90R = t; }
  else       { if(nT90L < 2) t90Ls[nT90L++] = t; cal.t90L = t; }
  Serial.print(dir > 0 ? F("  right") : F("  left"));
  Serial.print(F(" 90 degrees = ")); Serial.print(t); Serial.println(F(" ms"));
  if(!centreOnLine(dir)) calAbortWhy(F("no line under the bar after the spin"));
  stopRobot();
  delay(300);
}

/* The middle of a few samples: their average without the highest and the
 * lowest (with 3 or more), so one bad cell does not pull it. */
float median(float *a, uint8_t n){
  if(n == 0) return 0;
  float s = 0, lo = a[0], hi = a[0];
  for(uint8_t i=0;i<n;i++){ s += a[i]; if(a[i] < lo) lo = a[i]; if(a[i] > hi) hi = a[i]; }
  if(n < 3) return s / n;
  return (s - lo - hi) / (n - 2);
}

float spreadOf(float *a, uint8_t n){
  float lo = a[0], hi = a[0];
  for(uint8_t i=1;i<n;i++){ if(a[i] < lo) lo = a[i]; if(a[i] > hi) hi = a[i]; }
  float md = median(a, n);
  return md > 0 ? (hi - lo) / md : 1;
}

void addCruise(LegRes &L){
  for(uint8_t i=1;i<3;i++){
    if(L.uE[i] < 0 || L.uE[i-1] < 0) continue;            /* a crossing was missed there */
    float du = L.uE[i] - L.uE[i-1];
    if(du > 0 && nKCruise < 6) kCruise[nKCruise++] = CELL_CM / du;
  }
  if(nKCruise == 0) return;
  float k = median(kCruise, nKCruise);
  cal.vCruise100 = (uint16_t)(k * MAX_SP * 100 + 0.5);
  kEst = k;
}

/* ------------------------------------------------------------------ */
/*  Results: status, EEPROM, report, paste block                        */
/* ------------------------------------------------------------------ */
void calEvaluate(){
  /* trim: how straight did the last drift tests come out? */
  if(nBurst >= 2){
    float r = (fabs(burstM[nBurst-1]) + fabs(burstM[nBurst-2])) / 2;
    setStatus(CI_TRIM, (nBurst >= 4 && r < 0.03) ? ST_PASS : (r < 0.08 ? ST_WEAK : ST_FAIL));
  }else if(calItem(newStatus, CI_TRIM) == ST_NONE && curItem >= CI_MOVE){
    setStatus(CI_TRIM, ST_FAIL);
  }
  if(nKCruise >= 2){
    float s = spreadOf(kCruise, nKCruise);
    setStatus(CI_CRUISE, (nKCruise >= 4 && s <= 0.08) ? ST_PASS : (s <= 0.15 ? ST_WEAK : ST_FAIL));
  }
  if(nT90L && nT90R){
    uint16_t tl = t90Ls[nT90L - 1], tr = t90Rs[nT90R - 1];
    bool sane = tl >= 150 && tl <= 4000 && tr >= 150 && tr <= 4000;
    setStatus(CI_TURN, !sane ? ST_FAIL : ((nT90L == 2 && nT90R == 2) ? ST_PASS : ST_WEAK));
    cal.t90L = tl;
    cal.t90R = tr;
  }
  if(nSpinX >= 2){
    /* still over 10% but better than before the correction: keep the better
     * values (WEAK) rather than go back to the guess */
    float r = fabs(spinX[1]);
    setStatus(CI_SPIN, r < 0.04 ? ST_PASS : ((r < 0.10 || r < fabs(spinX[0])) ? ST_WEAK : ST_FAIL));
  }else if(nSpinX == 1){
    setStatus(CI_SPIN, ST_WEAK);
  }
  /* whatever step was running when it stopped has failed */
  if(calAborted && calItem(newStatus, curItem) == ST_NONE) setStatus(curItem, ST_FAIL);
}

bool calSave(uint16_t &seqOut, int &addrOut){
  CalRec old;
  int slot = calReadEeprom(old);
  CalRec r;
  memset(&r, 0, sizeof(r));
  r.magic = CAL_MAGIC; r.version = CAL_VERSION; r.size = sizeof(CalRec);
  r.seq = slot ? (uint16_t)(old.seq + 1) : 1;
  r.status = newStatus;
  r.spare = calAborted ? curItem + 1 : 0;   /* [E33] which step stopped it, for the report printed again later */
  r.maxSp = MAX_SP; r.moveSp = MOVE_SP; r.turnSp = TURN_SP;
  r.d = cal;
  r.d.fromRun = r.seq;
  r.fletcher = fletcher16((const uint8_t*)&r, sizeof(CalRec) - 2);
  int addr = (slot == CAL_SLOT_A + 1) ? CAL_SLOT_B : CAL_SLOT_A;   /* never overwrite the newest */
  EEPROM.put(addr, r);
  CalRec chk;
  EEPROM.get(addr, chk);
  seqOut = r.seq; addrOut = addr;
  cal.fromRun = r.seq;
  return memcmp(&chk, &r, sizeof(CalRec)) == 0;
}

uint16_t rawOf(uint16_t v){ return cal.lineLow ? (uint16_t)(1023 - v) : v; }

/* Small printers: the Nano's flash is tight in this mode. */
void pStat(uint8_t item){
  Serial.println();
  printItemName(item); Serial.print(F(" [")); printItemStatus(calItem(newStatus, item)); Serial.print(F("] "));
}
void pNum(const __FlashStringHelper *k, long v){ Serial.print(k); Serial.print(v); }
void pPct(float x){ Serial.print(' '); printF1(x * 100, 1); Serial.print('%'); }
void pDef(const __FlashStringHelper *k, long v){
  Serial.print(F("#define CAL_")); Serial.print(k); Serial.print(' '); Serial.println(v);
}
void pArr(const __FlashStringHelper *k, const uint16_t *v){
  Serial.print(F("#define CAL_")); Serial.print(k); Serial.print(F(" { "));
  for(uint8_t i=0;i<8;i++){ Serial.print(rawOf(v[i])); Serial.print(i < 7 ? F(", ") : F(" }\n")); }
}

void calReport(bool saved, uint16_t seq, int addr){
  Serial.print(F("\n=== CALIBRATION REPORT run #")); Serial.print(seq);
  if(calAborted){ Serial.print(F("   STOPPED during ")); if(calPhase) Serial.print(calPhase); else printItemName(curItem); }
  Serial.print(F("\n(anything not PASS or WEAK: the mission uses the default for it)"));

  /* sensors: weakest channel, unusable ones, and a slide-style single number */
  pStat(CI_SENSOR);
  Serial.print(cal.lineLow ? F("black reads LOW") : F("black reads HIGH"));
  uint16_t minSpan = 1023;
  uint8_t weakest = 0;
  for(uint8_t i=0;i<8;i++){
    if(cal.deadMask & (0x80 >> i)){ pNum(F(", NOT USED: A"), i); continue; }
    uint16_t span = cal.hi[i] > cal.lo[i] ? cal.hi[i] - cal.lo[i] : 0;
    if(span < minSpan){ minSpan = span; weakest = i; }
  }
  pNum(F(", weakest A"), weakest); pNum(F(" contrast "), minSpan);
  Serial.print(F(" (MODE_SENSOR_CHECK: full sensor report)"));

  pStat(CI_DEADBD);
  pNum(F("duty where each wheel starts: left fwd "), cal.dbLf); pNum(F(" back "), cal.dbLr);
  pNum(F(", right fwd "), cal.dbRf); pNum(F(" back "), cal.dbRr);

  pStat(CI_TRIM);
  if(cal.trimR < cal.trimL)      pNum(F("RIGHT motor stronger by "), (long)(100.0 * cal.trimL / cal.trimR - 99.5));
  else if(cal.trimL < cal.trimR) pNum(F("LEFT motor stronger by "), (long)(100.0 * cal.trimR / cal.trimL - 99.5));
  if(cal.trimL != cal.trimR) Serial.print('%');
  Serial.print(F(" (turned down to match). left/right effort in each cell, 0 = straight:"));
  for(uint8_t i=0;i<nBurst;i++) pPct(burstM[i]);

  pStat(CI_SPIN);
  Serial.print(F("spin left vs right time, before and after matching the backward trims:"));
  for(uint8_t i=0;i<nSpinX;i++) pPct(spinX[i]);

  pStat(CI_MOVE);
  pNum(F("cm/s x100 at MOVE_SP: "), cal.vMove100); pNum(F(", start-up ms: "), cal.lagMove);
  pStat(CI_CRUISE);
  pNum(F("cm/s x100 at MAX_SP: "), cal.vCruise100); pNum(F(", cells measured: "), nKCruise);
  pStat(CI_TURN);
  Serial.print(F("ms per 90 degrees, left:"));
  for(uint8_t i=0;i<nT90L;i++) pNum(F(" "), t90Ls[i]);
  Serial.print(F("  right:"));
  for(uint8_t i=0;i<nT90R;i++) pNum(F(" "), t90Rs[i]);

  pNum(F("\n\nsaved in EEPROM at "), addr);
  Serial.println(saved ? F(", read back OK") : F(", READ BACK FAILED"));
  Serial.println(calAborted ? F("(not finished: do not paste these)")
                            : F("To freeze these values: paste over CAL VALUES in calibration.h, set CAL_USE_EEPROM 0"));
  pDef(F("VALUES_FROM_RUN"), seq);
  pDef(F("LINE_LOW"), cal.lineLow);
  pArr(F("LO"), cal.lo);
  pArr(F("HI"), cal.hi);
  pDef(F("DEAD_MASK"), cal.deadMask);
  pDef(F("DB_LF"), cal.dbLf);
  pDef(F("DB_LR"), cal.dbLr);
  pDef(F("DB_RF"), cal.dbRf);
  pDef(F("DB_RR"), cal.dbRr);
  pDef(F("TRIM_L"), cal.trimL);
  pDef(F("TRIM_R"), cal.trimR);
  pDef(F("TRIM_LR"), cal.trimLr);
  pDef(F("TRIM_RR"), cal.trimRr);
  pDef(F("V_CRUISE_X100"), cal.vCruise100);
  pDef(F("V_MOVE_X100"), cal.vMove100);
  pDef(F("LAG_MOVE_MS"), cal.lagMove);
  pDef(F("T90_L_MS"), cal.t90L);
  pDef(F("T90_R_MS"), cal.t90R);
  Serial.println(F("/* ---- end CAL VALUES ---- */"));
  Serial.println(calAborted ? F("NOT FINISHED: fix the problem (README: When the calibration stops), then calibrate again.")
                            : F("Done. Set ROBOT_MODE to MODE_MISSION in calibration.h and upload."));
}

void calFinish(){
  stopRobot();
  digitalWrite(STBY, 0);
  calEvaluate();
  uint16_t seq = 0; int addr = 0;
  bool saved = calSave(seq, addr);
  EEPROM.update(CAL_REARM, 1);            /* plugging in USB later will not start another run */
  calReport(saved, seq, addr);
  bool good = !calAborted;
  while(1){                               /* LED: long on = finished, fast blink = stopped */
    digitalWrite(LED_PIN, 1); delay(good ? 900 : 100);
    digitalWrite(LED_PIN, 0); delay(good ? 100 : 100);
  }
}

void calAbortWhy(const __FlashStringHelper *why){
  stopRobot();
  digitalWrite(STBY, 0);
  calAborted = true;
  Serial.print(F("CALIBRATION STOPPED during ")); Serial.print(calPhase); Serial.print(F(": "));
  Serial.println(why);
  calFinish();
}

void calFaultHook(uint8_t code, const __FlashStringHelper *why){
  (void)code;
  calAbortWhy(why);
}

void runCalibration(){
  faultHook = calFaultHook;
  Serial.println(F("=== AUTOMATIC CALIBRATION ==="));
  trimLf = cal.trimL; trimRf = cal.trimR;
  revL = (float)cal.trimLr / (cal.trimL ? cal.trimL : 1000);
  revR = (float)cal.trimRr / (cal.trimR ? cal.trimR : 1000);
  { float m = (revL + revR) / 2; revL /= m; revR /= m; }
  kEst = (cal.vMove100 / 100.0) / MOVE_SP;
  vScale = 1.0;
  newStatus = 0;

  RawLook r;
  calPhase = F("waiting for placement");
  digitalWrite(STBY, 0);
  calWaitPlacement(r);
  EEPROM.update(CAL_REARM, 0);
  digitalWrite(STBY, 1);
  digitalWrite(LED_PIN, 1);

  /* 1 */
  calPhase = F("sensors, standing still"); curItem = CI_SENSOR;
  calFromLook(r);                        /* each channel's own starting levels */
  Serial.print(F("step 1: black reads ")); Serial.print(r.lineLow ? F("LOW") : F("HIGH"));
  Serial.print(F(", white about ")); Serial.print(rawOf(r.white));
  Serial.print(F(", line about ")); Serial.println(rawOf(r.black));

  /* 2, 3 */
  measureDeadband();
  measureSensors();

  /* 4: legs and spins */
  LegRes L;
  calPhase = F("leg 1: C1 to C4 at MOVE_SP"); curItem = CI_MOVE;
  Serial.println(F("step 4: driving (leg 1, slow)"));
  runLeg(true, MOVE_SP, true, L, true);
  for(uint8_t i=1;i<3;i++){
    if(L.uE[i] < 0 || L.uE[i-1] < 0) continue;
    float du = L.uE[i] - L.uE[i-1];
    if(du > 0 && nKMove < 2) kMove[nKMove++] = CELL_CM / du;
  }
  if(nKMove == 2){
    float k = (kMove[0] + kMove[1]) / 2;
    float d0 = CELL_CM - SENSOR_AHEAD_CM + TAPE_W_CM / 2;
    float lagU = (L.uE[0] - L.uStart) - d0 / k;
    int lag = (int)(lagU / MOVE_SP * 1000);
    if(lag < 0) lag = 0;
    if(lag > 400) lag = 400;
    lagMs = lag;
    kEst = k;
    cal.vMove100   = (uint16_t)(k * MOVE_SP * 100 + 0.5);
    cal.vCruise100 = (uint16_t)(k * MAX_SP * 100 + 0.5);   /* until measured */
    cal.lagMove    = lag;
    float s = fabs(kMove[0] - kMove[1]) / k;
    setStatus(CI_MOVE, s <= 0.08 ? ST_PASS : (s <= 0.15 ? ST_WEAK : ST_FAIL));
  }
  rollToBarPast(SENSOR_AHEAD_CM);
  spinAt(-1, F("spin at C4 (left)"));

  calPhase = F("leg 2: C4 to C1 at MAX_SP"); curItem = CI_CRUISE;
  runLeg(false, MAX_SP, false, L);
  addCruise(L);
  rollToBarPast(SENSOR_AHEAD_CM);
  spinAt(+1, F("spin at C1 (right)"));
  matchSpins();

  calPhase = F("leg 3: C1 to C4 at MAX_SP"); curItem = CI_CRUISE;
  runLeg(false, MAX_SP, false, L);
  addCruise(L);
  rollToBarPast(SENSOR_AHEAD_CM);
  spinAt(-1, F("spin at C4 (left)"));

  calPhase = F("leg 4: C4 to C1 at MAX_SP"); curItem = CI_CRUISE;
  runLeg(false, MAX_SP, false, L);
  addCruise(L);
  rollToBarPast(SENSOR_AHEAD_CM);
  spinAt(+1, F("spin at C1 (right)"));
  matchSpins();

  calPhase = F("finished");
  calFinish();
}

/* =====================================================================
 *  MODE_METER: print what the bar sees. The robot never moves.
 * ===================================================================== */
void runMeter(){
  digitalWrite(STBY, 0);
  Serial.println(F("SENSOR METER: the robot never moves in this mode."));
  Serial.println(F("Slide the robot over white, over the line, over a crossing."));
  uint16_t mn[8], mx[8];
  for(uint8_t i=0;i<8;i++){ mn[i] = 1023; mx[i] = 0; }
  uint8_t line = 0;
  while(true){
    uint8_t m = scanBar();
    uint16_t v[8];
    for(uint8_t i=0;i<8;i++){
      v[i] = analogRead(sensorPin[i]);
      if(v[i] < mn[i]) mn[i] = v[i];
      if(v[i] > mx[i]) mx[i] = v[i];
    }
    Serial.print(F("raw ")); printRaw(v);
    Serial.print(F("| ")); printMask(m);
    int e = getErrorInput(getSensor());
    Serial.print(F(" error ")); if(e == 100) Serial.print(F("none")); else Serial.print(e);
    int c = centroid256();
    Serial.print(F(" | line at ")); if(c < 0) Serial.print(F("-")); else printF1(c / 256.0, 2);
    if(checkGrid()) Serial.print(F(" | CROSSING"));
    Serial.println();
    if(++line >= 20){
      line = 0;
      Serial.print(F("lowest so far  ")); printRaw(mn); Serial.println();
      Serial.print(F("highest so far ")); printRaw(mx); Serial.println();
    }
    delay(300);
  }
}

/* =====================================================================
 *  MODE_MOTOR_CHECK: wheels OFF the ground. Each motor in turn.
 * ===================================================================== */
void motorStep(const __FlashStringHelper *what, int uL, int uR){
  Serial.println(what);
  setMotor(uL, uR);
  delay(2000);
  setMotor(0, 0);
  delay(1000);
}

void rampRaw(uint8_t side){
  Serial.print(side ? F("RIGHT") : F("LEFT"));
  Serial.println(F(" wheel, raw duty going up: note the number where it starts turning:"));
  for(uint16_t d=0; d<=160; d+=5){
    motorRaw(side, 1, (uint8_t)d);
    Serial.print(d); Serial.print(' ');
    delay(250);
  }
  motorRaw(side, 0, 0);
  Serial.println();
  delay(1000);
}

void runMotorCheck(){
  Serial.println(F("MOTOR CHECK: LIFT THE WHEELS OFF THE GROUND. Starting in 3 s."));
  delay(3000);
  while(true){
    motorStep(F("LEFT wheel FORWARD  (as if the robot drives forward)"), MOVE_SP, 0);
    motorStep(F("LEFT wheel BACKWARD"), -MOVE_SP, 0);
    motorStep(F("RIGHT wheel FORWARD"), 0, MOVE_SP);
    motorStep(F("RIGHT wheel BACKWARD"), 0, -MOVE_SP);
    motorStep(F("BOTH FORWARD at MOVE_SP: with the trim both should turn at the SAME speed"), MOVE_SP, MOVE_SP);
    motorStep(F("BOTH FORWARD at MAX_SP"), MAX_SP, MAX_SP);
    motorStep(F("SPIN RIGHT (left forward, right backward)"), TURN_SP, -TURN_SP);
    rampRaw(0);
    rampRaw(1);
    Serial.println(F("If a wheel turned the wrong way, set LEFT_REVERSED or RIGHT_REVERSED to 1."));
    Serial.println(F("If the OTHER wheel turned (right for LEFT), swap the two motor plugs."));
    Serial.println(F("If one never turned: motor switch, battery, STBY wire (D10)."));
    Serial.println(F("Again in 5 s..."));
    delay(5000);
  }
}

#endif
