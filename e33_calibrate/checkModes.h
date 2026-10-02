/* =====================================================================
 *  checkModes.h: sensor check, gripper check and turn check   [E33]
 * =====================================================================
 *
 *  MODE_SENSOR_CHECK   the robot never moves. Four steps, each started by
 *                      pressing Enter in the Serial Monitor (or it goes on
 *                      by itself after 20 s):
 *                        1. the whole bar over WHITE
 *                        2. turn the robot left and right by hand so the
 *                           line passes under EVERY sensor
 *                        3. the line under the MIDDLE of the bar
 *                        4. the bar lying ALONG a line (black under all 8)
 *                      Then it prints a report of white and black for each
 *                      sensor. Copy it from BEGIN to END and send it.
 *
 *  MODE_GRIPPER_CHECK  the robot never drives. Type a letter + Enter in the
 *                      Serial Monitor to move the gripper and the arm one
 *                      degree at a time, to find your angles.
 *
 *  MODE_TURN_CHECK     the wheels (axle) on a MID crossing with four arms
 *                      (C2 MID or C3 MID), facing along MID. After Enter
 *                      (or 30 s) and the line steady under the bar, it
 *                      runs the mission's own turns: 4 x left 90, 4 x
 *                      right 90, 2 x left 180, 2 x right 180, twice, and
 *                      ends facing the way it started. One line per turn,
 *                      then a report. Copy it from BEGIN to END and send it.
 *
 *  Serial Monitor: 115200 baud, "New Line" (or "Both NL & CR"; with "No Line
 *  Ending" an empty Enter sends nothing).
 * ===================================================================== */
#ifndef CHECKMODES_H
#define CHECKMODES_H

/* Wait for Enter in the Serial Monitor, or for secs seconds.
 * after: 0 = a standing-still measurement follows, 1 = the sensor sweep by
 * hand, 2 = the turn check (the robot is about to turn: step back). */
void waitEnter(uint16_t secs, uint8_t after = 0){
  while(Serial.available()) Serial.read();
  Serial.print(F("   press Enter in the Serial Monitor when ready (or wait "));
  Serial.print(secs); Serial.println(F(" s)"));
  unsigned long t0 = millis();
  while(millis() - t0 < (unsigned long)secs * 1000UL){
    if(Serial.available()){ delay(30); while(Serial.available()) Serial.read(); break; }
    digitalWrite(LED_PIN, (millis() / 250) & 1);
    delay(5);
  }
  digitalWrite(LED_PIN, 0);
  Serial.println(after == 0 ? F("   measuring, keep still...") : after == 1 ? F("   go: turn it left and right now")
                             : F("   step back: it turns once the line is steady"));
}

/* ------------------------------------------------------------------ */
/*  Sensor check                                                        */
/* ------------------------------------------------------------------ */
#define SC_N 200                         /* readings per standing-still step */

struct SStat { uint16_t mn, mx; uint32_t sum, sq; };
SStat scWhite1[8], scWhite2[8], scCentre[8], scCross[8];
uint16_t scPeakHi[8], scPeakLo[8];       /* step 2: highest / lowest smoothed reading */

uint16_t scRead(uint8_t i, bool twice){
  if(twice) analogRead(sensorPin[i]);
  return analogRead(sensorPin[i]);
}

void scTake(SStat *s, bool twice){
  for(uint8_t i=0;i<8;i++){ s[i].mn = 1023; s[i].mx = 0; s[i].sum = 0; s[i].sq = 0; }
  for(uint16_t k=0;k<SC_N;k++){
    for(uint8_t i=0;i<8;i++){
      uint16_t v = scRead(i, twice);
      if(v < s[i].mn) s[i].mn = v;
      if(v > s[i].mx) s[i].mx = v;
      s[i].sum += v;
      s[i].sq  += (uint32_t)v * v;
    }
    delay(2);
  }
}

uint16_t scMean(const SStat &s){ return (uint16_t)((s.sum + SC_N / 2) / SC_N); }
uint16_t scSd(const SStat &s){
  float m = (float)s.sum / SC_N;
  float var = (float)s.sq / SC_N - m * m;
  return var > 0 ? (uint16_t)(sqrt(var) + 0.5) : 0;
}

/* Step 2: the robot is moved by hand for secs seconds. */
void scSweep(uint16_t secs){
  uint16_t f[8];
  for(uint8_t i=0;i<8;i++){ f[i] = scRead(i, true); scPeakHi[i] = f[i]; scPeakLo[i] = f[i]; }
  unsigned long t0 = millis(), tDot = t0;
  while(millis() - t0 < (unsigned long)secs * 1000UL){
    for(uint8_t i=0;i<8;i++){
      /* smoothed over ~4 readings: the peak of a noise spike does not count */
      f[i] = (uint16_t)((3u * f[i] + scRead(i, true) + 2) / 4);
      if(f[i] > scPeakHi[i]) scPeakHi[i] = f[i];
      if(f[i] < scPeakLo[i]) scPeakLo[i] = f[i];
    }
    if(millis() - tDot >= 1000){
      tDot = millis();
      Serial.print(F("   sensors that have seen the line so far: "));
      for(uint8_t i=0;i<8;i++){
        /* either way from white: the bar may read black LOW */
        int w = scMean(scWhite2[i]), hi = scPeakHi[i], lo = scPeakLo[i];
        Serial.print((hi > w + 150 || lo + 150 < w) ? '1' : '0');
      }
      Serial.println();
    }
  }
}

void scRaw(uint16_t v){ Serial.print(cal.lineLow ? 1023 - v : v); }   /* black-high scale -> raw */

void scReport(){
  /* which way is black? compare the crossing with white, over the whole bar */
  long up = 0, up2 = 0;
  for(uint8_t i=0;i<8;i++){
    long w = scMean(scWhite2[i]);
    up  += (long)scMean(scCross[i]) - w;
    up2 += (long)scPeakHi[i] + scPeakLo[i] - 2 * w;   /* step 2: which way it swung */
  }
  /* step 4 was not on black: step 2 decides, if it swung further */
  bool sameAs1 = labs(up) < 800;
  if(sameAs1 && labs(up2) > labs(up)) up = up2;
  bool low = up < 0;                     /* black reads LOW on this bar */
  uint16_t white[8], black[8];
  uint8_t weakest = 0; int minC = 2000;
  uint8_t missing = 0, dead = 0;
  for(uint8_t i=0;i<8;i++){
    white[i] = scMean(scWhite2[i]);
    uint16_t xc = scMean(scCross[i]);
    uint16_t pk = low ? scPeakLo[i] : scPeakHi[i];
    /* black: the crossing, or the darkest seen in step 2 if that was darker */
    /* step 4 is a steady reading; the step-2 peak can be a spike (noise, or
     * the bar lifted off the table), so use it only where step 4 missed */
    int c4 = low ? (int)white[i] - (int)xc : (int)xc - (int)white[i];
    int cp = low ? (int)white[i] - (int)pk : (int)pk - (int)white[i];
    black[i] = (c4 * 10 >= cp * 7) ? xc : pk;
    int c = low ? (int)white[i] - (int)black[i] : (int)black[i] - (int)white[i];
    if(c < minC){ minC = c; weakest = i; }
    /* step 2 on its own: did the line pass under this sensor? */
    int d2 = low ? (int)white[i] - (int)scPeakLo[i] : (int)scPeakHi[i] - (int)white[i];
    if(d2 <= 150) missing |= (0x80 >> i);
    if(c < 100) dead |= (0x80 >> i);
  }

  Serial.println();
  Serial.println(F("=== E33 SENSOR REPORT BEGIN (copy from here to END) ==="));
  Serial.print(F("levels in use now: "));
  if(calSource == 1){ Serial.print(F("EEPROM calibration run #")); Serial.println(cal.fromRun); }
  else if(calSource == 2) Serial.println(F("pasted into calibration.h"));
  else Serial.println(CAL_SENSORS_MEASURED ? F("the measured levels in calibration.h") : F("defaults (never calibrated)"));
  Serial.print(F("black reads ")); Serial.println(low ? F("LOW") : F("HIGH"));
  if(sameAs1)
    Serial.println(F("(steps 1 and 4 read almost the same: was step 1 on white and step 4 on the line?)"));
  if(low != (bool)cal.lineLow)
    Serial.println(F("(the levels in use now expect the other way: sees_line_now is NO until you calibrate)"));
  if(calSource == 0 && !CAL_SENSORS_MEASURED)
    Serial.println(F("(sees_line_now uses the DEFAULT levels: NO is normal before MODE_CALIBRATE; after it, all must say yes)"));
  Serial.println(F("ch,white,white_noise,white_single_read,black,contrast,line_in_middle,on_level_now,sees_line_now,verdict"));
  for(uint8_t i=0;i<8;i++){
    int c = low ? (int)white[i] - (int)black[i] : (int)black[i] - (int)white[i];
    uint16_t sd = scSd(scWhite2[i]);
    /* would the levels in use now see this sensor's black? (black-high scale) */
    uint16_t bn = cal.lineLow ? (uint16_t)(1023 - black[i]) : black[i];
    uint16_t wn = cal.lineLow ? (uint16_t)(1023 - white[i]) : white[i];
    bool sees = bn >= onLvl[i] && wn < offLvl[i] && !(cal.deadMask & (0x80 >> i));
    Serial.print('A'); Serial.print(i); Serial.print(',');
    Serial.print(white[i]); Serial.print(',');
    Serial.print(sd); Serial.print(',');
    Serial.print(scMean(scWhite1[i])); Serial.print(',');
    Serial.print(black[i]); Serial.print(',');
    Serial.print(c); Serial.print(',');
    Serial.print(scMean(scCentre[i])); Serial.print(',');
    scRaw(onLvl[i]); Serial.print(',');
    Serial.print(sees ? F("yes") : F("NO")); Serial.print(',');
    if(c < 100)               Serial.println(F("DEAD"));
    else if(c < 200)          Serial.println(F("WEAK"));
    else if(sd * 10 > (uint16_t)c) Serial.println(F("NOISY"));
    else                      Serial.println(F("OK"));
  }
  Serial.print(F("weakest: A")); Serial.print(weakest); Serial.print(F(" contrast ")); Serial.println(minC);
  /* a sensor that was on the line in step 1 has "white" = black: say so */
  uint16_t ws[8];
  for(uint8_t i=0;i<8;i++) ws[i] = low ? 1023 - white[i] : white[i];
  for(uint8_t a=1;a<8;a++){ uint16_t x = ws[a]; int8_t j = a - 1; while(j >= 0 && ws[j] > x){ ws[j+1] = ws[j]; j--; } ws[j+1] = x; }
  uint16_t wMid = ws[3];
  for(uint8_t i=0;i<8;i++){
    uint16_t w = low ? 1023 - white[i] : white[i];
    uint16_t w2 = low ? 1023 - scPeakHi[i] : scPeakLo[i];   /* the whitest seen in step 2 ... */
    uint16_t w3 = low ? 1023 - scMean(scCentre[i]) : scMean(scCentre[i]);
    if(w3 < w2) w2 = w3;                                     /* ... or in step 3 */
    if((dead & (0x80 >> i)) && w > wMid + 200){
      Serial.print(F("A")); Serial.print(i);
      if(w2 + 200 < w) Serial.println(F(" looked black in step 1: was the line under it? Do step 1 again on plain white."));
      else             Serial.println(F(" reads the same on white and black: broken, unplugged, or too far from the floor."));
    }
  }

  /* one number for all 8, like the slides' 800 / 500 */
  int lo = 0, hi = 1023, minSpan = 2000;
  for(uint8_t i=0;i<8;i++){
    if(dead & (0x80 >> i)) continue;
    int w = low ? 1023 - white[i] : white[i];
    int b = low ? 1023 - black[i] : black[i];
    if(w > lo) lo = w;
    if(b < hi) hi = b;
    if(b - w < minSpan) minSpan = b - w;
  }
  int a = lo + minSpan * 15 / 100, b = hi - minSpan * 15 / 100;
  Serial.print(F("one threshold for all 8 (like the slides' 800 / 500): "));
  if(dead != 0xFF && a < b){
    int t = (a + b) / 2;
    Serial.print(low ? 1023 - t : t); Serial.println(low ? F(" (black is BELOW it)") : F(" (black is above it)"));
  }
  else Serial.println(F("none works for all 8"));

  int adc = 0;
  for(uint8_t i=0;i<8;i++){
    int d = (int)scMean(scWhite1[i]) - (int)scMean(scWhite2[i]);
    if(abs(d) > abs(adc)) adc = d;
  }
  Serial.print(F("reading once vs twice differs by up to ")); Serial.print(adc); Serial.println(F(" counts"));

  Serial.print(F("step 2: every sensor saw the line: "));
  if(!missing) Serial.println(F("yes"));
  else{
    Serial.print(F("NO, not ")); printMask(missing);
    Serial.println(F(" (weak, or step 2 too fast / too short)"));
  }

  /* step 3 with the levels in use now */
  uint8_t m = 0;
  for(uint8_t i=0;i<8;i++){
    uint16_t v = scMean(scCentre[i]);
    uint16_t vn = cal.lineLow ? (uint16_t)(1023 - v) : v;
    if(vn >= onLvl[i] && !(cal.deadMask & (0x80 >> i))) m |= (0x80 >> i);
  }
  Serial.print(F("step 3 (line in the middle) with the levels in use now: ")); printMask(m);
  Serial.println((m == 0x18 || m == 0x10 || m == 0x08 || m == 0x38 || m == 0x1C) ? F("  good") : F("  (expected 00011000)"));

  Serial.println(F("sensor part of CAL VALUES (not needed: MODE_CALIBRATE measures them again. To use them, select the"));
  Serial.println(F("same 4 lines in calibration.h, paste over them, then run tools\\copy_settings.bat):"));
  Serial.print(F("#define CAL_LINE_LOW ")); Serial.println(low ? 1 : 0);
  Serial.print(F("#define CAL_LO { "));
  for(uint8_t i=0;i<8;i++){ Serial.print(white[i]); Serial.print(i < 7 ? F(", ") : F(" }\n")); }
  Serial.print(F("#define CAL_HI { "));
  for(uint8_t i=0;i<8;i++){ Serial.print(black[i]); Serial.print(i < 7 ? F(", ") : F(" }\n")); }
  Serial.print(F("#define CAL_DEAD_MASK 0x")); Serial.println(dead, HEX);
  Serial.println(F("=== E33 SENSOR REPORT END ==="));
}

void runSensorCheck(){
  digitalWrite(STBY, 0);                 /* motors off: this mode never moves */
  Serial.println(F("=== SENSOR CHECK: the robot never moves in this mode ==="));
  Serial.println(F("Four steps, then a report to copy and send (from BEGIN to END)."));
  while(true){
    Serial.println(F("\nSTEP 1 of 4: put the WHOLE bar over WHITE, no line under any sensor."));
    waitEnter(20);
    scTake(scWhite1, false);
    scTake(scWhite2, true);

    Serial.println(F("\nSTEP 2 of 4: with the bar over a line, turn the robot slowly LEFT and RIGHT by"));
    Serial.println(F("hand, wheels on the table (do not lift it: a lifted bar reads black), so the"));
    Serial.println(F("line passes under EVERY sensor, A0 to A7 and back. It watches for 15 s."));
    waitEnter(20, 1);
    scSweep(15);

    Serial.println(F("\nSTEP 3 of 4: put the line under the MIDDLE of the bar (between A3 and A4), keep still."));
    waitEnter(20);
    scTake(scCentre, true);

    Serial.println(F("\nSTEP 4 of 4: turn the robot so the bar lies ALONG a straight line: black under ALL 8"));
    Serial.println(F("sensors, from A0 to A7. Keep still."));
    waitEnter(20);
    scTake(scCross, true);

    scReport();
    Serial.println(F("\nPress Enter to do the check again, or go on with the next step in README.md."));
    while(!Serial.available()){ digitalWrite(LED_PIN, (millis() / 1000) & 1); delay(5); }
    delay(30);
    while(Serial.available()) Serial.read();
  }
}

/* ------------------------------------------------------------------ */
/*  Gripper check                                                       */
/* ------------------------------------------------------------------ */
void gcHelp(){
  Serial.println(F("Type a letter and press Enter:"));
  Serial.println(F("  o open (GRIP_OPEN)   r release (GRIP_RELEASE)   c closed (GRIP_CLOSED)"));
  Serial.println(F("  + gripper 1 degree tighter    - gripper 1 degree looser"));
  Serial.println(F("  d arm down (ARM_DOWN)   y carry (ARM_CARRY)   h high (ARM_HIGH)"));
  Serial.println(F("  u arm 1 degree up    j arm 1 degree down"));
  Serial.println(F("  t test: pick up, hold 2 s, put down (as in the mission)"));
  Serial.println(F("  p print the angles and the sensor bar    ? this list"));
  Serial.println(F("Sensor bar: on white, note the numbers with p. Then d, y and t with the object in the"));
  Serial.println(F("jaws: the numbers must stay near the white ones, or the arm looks like a line."));
  Serial.println(F("Finding GRIP_CLOSED: put the object in the jaws with a strip of paper between"));
  Serial.println(F("one jaw and the object. Press + until the strip is held. That angle is where the"));
  Serial.println(F("jaws touch. After closing, the mission opens GRIP_BACKOFF_DEG again to hold, so"));
  Serial.print(F("set GRIP_CLOSED ")); Serial.print(GRIP_BACKOFF_DEG + 2);
  Serial.println(F(" degrees tighter, upload, and check with t that it holds. The servo must not buzz."));
  Serial.println(F("(Several at once: +++++ and Enter = 5 degrees.)"));
}

void gcPrint(){
  Serial.print(F("gripper ")); Serial.print(gripNow);
  Serial.print(F("   arm ")); Serial.print(armNow);
  /* [E33] the number to write, so "tighter" needs no sign rule */
  Serial.print(F("   (jaws just touch here? then GRIP_CLOSED "));
  Serial.print(gripNow + ((GRIP_CLOSED < GRIP_OPEN) ? -1 : 1) * (GRIP_BACKOFF_DEG + 2));
  Serial.print(F(", now ")); Serial.print(GRIP_CLOSED); Serial.println(F(")"));
  /* [E33] the sensor bar too: the arm or a held object in front of it can
   * shine the sensors' light back and look like a line */
  Serial.print(F("   bar A0..A7:"));
  for(uint8_t i=0;i<8;i++){ Serial.print(' '); Serial.print(scRead(i, true)); }
  Serial.println();
}

void runGripperCheck(){
  digitalWrite(STBY, 0);                 /* the wheels never turn in this mode */
  Serial.println(F("=== GRIPPER CHECK: the robot never drives in this mode ==="));
  gcHelp();
  gcPrint();
  /* closing is a smaller angle (GRIP_CLOSED < GRIP_OPEN); up is a smaller angle (ARM_HIGH < ARM_DOWN) */
  int tighter = (GRIP_CLOSED < GRIP_OPEN) ? -1 : 1;
  int up      = (ARM_HIGH < ARM_DOWN) ? -1 : 1;
  while(true){
    if(!Serial.available()){ delay(5); continue; }
    char k = Serial.read();
    switch(k){
      case 'o': servoTo(servo_x, gripNow, GRIP_OPEN); break;
      case 'r': servoTo(servo_x, gripNow, GRIP_RELEASE); break;
      case 'c': servoTo(servo_x, gripNow, GRIP_CLOSED); break;
      case '+': servoTo(servo_x, gripNow, constrain(gripNow + tighter, 0, 180)); break;
      case '-': servoTo(servo_x, gripNow, constrain(gripNow - tighter, 0, 180)); break;
      case 'd': servoTo(servo_y, armNow, ARM_DOWN); break;
      case 'y': servoTo(servo_y, armNow, ARM_CARRY); break;
      case 'h': servoTo(servo_y, armNow, ARM_HIGH); break;
      case 'u': servoTo(servo_y, armNow, constrain(armNow + up, 0, 180)); break;
      case 'j': servoTo(servo_y, armNow, constrain(armNow - up, 0, 180)); break;
      case 't': keepup_object(); delay(2000); put_object(); break;
      case '?': gcHelp(); break;
      case 'p': break;
      default:                           /* line endings and anything else */
        if(k > ' ') Serial.println(F("unknown key: small letters only, ? for the list"));
        continue;
    }
    gcPrint();
  }
}

/* ------------------------------------------------------------------ */
/*  Turn check  [E33]                                                   */
/*  The mission's own turns (turnLeft90 ... turnRight180), again and    */
/*  again on one crossing, timed against what the mission expects.      */
/* ------------------------------------------------------------------ */
#if ROBOT_MODE == MODE_TURN_CHECK
#define TC_ROUNDS      2      /* the whole sequence this many times          */
#define TC_TURNS      12      /* turns in one sequence                       */
#define TC_PAUSE_MS 1000      /* standing still between two turns            */
#define TC_SLOW_PCT  130      /* longer than this % of the expected time: SLOW */
#define TC_FAST_PCT   75      /* shorter than this %: FAST                   */
#define TC_SPREAD_PCT 20      /* times more than +- this % apart: uneven     */
#define TC_AXLE_MM    15      /* axle this far off the crossing: say so      */

/* The four kinds of turn: bit 0 = right, bit 1 = 180. */
struct TcStat { uint8_t n, fail, slow, fast, extra; uint16_t mn, mx; uint32_t sum; };
TcStat tcSt[4];
int8_t  tcNow = -1;           /* kind of the turn in progress, -1 = none     */
uint8_t tcTurn = 0;           /* turns started                               */
uint8_t tcOff = 0;            /* turns that ended with the line off the middle */
uint8_t tcLines = 0;          /* lines the turn in progress should pass      */
/* Where the axle is. With the axle d cm short of the crossing, the bar
 * reaches a column arm early and MID late: the four 90s of a circle are
 * 90-a, 90+a, 90+a, 90-a degrees, with sin(a) = d / SENSOR_AHEAD_CM (a 180
 * from MID to MID is always 180). */
uint16_t tcQ[4];              /* the times of the 90s of this circle         */
long    tcAxleSum = 0;        /* a, tenths of a degree, added over circles   */
uint8_t tcAxleN = 0;          /* circles added                               */

/* One sequence: 4 x left 90, 4 x right 90, 2 x left 180, 2 x right 180.
 * Each group is a whole circle, so the robot ends facing the way it started. */
uint8_t tcKind(uint8_t i){ return (i < 4) ? 0 : (i < 8) ? 1 : (i < 10) ? 2 : 3; }

void tcName(uint8_t g){ Serial.print((g & 1) ? 'R' : 'L'); Serial.print((g & 2) ? F("180") : F("90")); }

/* the time the mission expects, the same sum as in spinTurn() */
unsigned long tcExpected(uint8_t g){
  unsigned long ms90 = (unsigned long)(((g & 1) ? cal.t90R : cal.t90L) / vScale);
  return ms90 * ((g & 2) ? 2 : 1);
}

/* One line per turn: time, lines passed, the bar afterwards, the error. */
void tcLine(uint8_t g, const __FlashStringHelper *status){
  Serial.print(F("turn ")); Serial.print(tcTurn); Serial.print('/'); Serial.print(TC_ROUNDS * TC_TURNS);
  Serial.print(' '); tcName(g);
  Serial.print(F("  ")); Serial.print(lastTurnMs); Serial.print(F(" ms (expected "));
  Serial.print(tcExpected(g)); Serial.print(F(")  lines passed ")); Serial.print(lastTurnLines);
  Serial.print(F(" (should be ")); Serial.print(tcLines); Serial.print(F(")  bar ")); printMask(lineMask);
  int e = errorOfMask(lineMask);
  Serial.print(F("  error ")); if(e == 100) Serial.print(F("none")); else Serial.print(e);
  Serial.print(F("  ")); Serial.println(status);
}

void tcSay(const __FlashStringHelper *what){ Serial.print(F("- ")); Serial.println(what); }

/* The report to send, from BEGIN to END. code / why: the fault that
 * stopped the check (code 0 = it finished). */
void tcSummary(uint8_t code, const __FlashStringHelper *why){
  Serial.println();
  Serial.println(F("=== E33 TURN CHECK BEGIN (copy from here to END) ==="));
  Serial.print(F("levels and turn times: "));
  if(calSource == 1){ Serial.print(F("EEPROM calibration run #")); Serial.println(cal.fromRun); }
  else if(calSource == 2) Serial.println(F("pasted into calibration.h"));
  else Serial.println(CAL_SENSORS_MEASURED ? F("measured levels in calibration.h, turn times guessed (never calibrated)") : F("defaults (never calibrated)"));
  Serial.print(F("TURN_SP ")); Serial.print(TURN_SP); Serial.print(F("  TURN_SLOW_SP ")); Serial.print(TURN_SLOW_SP);
  Serial.print(F("  t90 left ")); Serial.print(cal.t90L); Serial.print(F(" right ")); Serial.print(cal.t90R);
  Serial.println(F(" ms"));
  uint8_t done = 0, fails = 0, extra = 0, uneven = 0;
  for(uint8_t g=0;g<4;g++){ done += tcSt[g].n; fails += tcSt[g].fail; extra += tcSt[g].extra; }
  Serial.print(F("turns done ")); Serial.print(done); Serial.print(F(" of ")); Serial.println(TC_ROUNDS * TC_TURNS);
  /* spread: half of (longest - shortest), in % of the mean */
  Serial.println(F("turn,done,mean_ms,min_ms,max_ms,spread_pct,expected_ms,slow,fast,extra_line,failed"));
  unsigned long mean[4];
  for(uint8_t g=0;g<4;g++){
    TcStat &s = tcSt[g];
    mean[g] = s.n ? (s.sum + s.n / 2) / s.n : 0;
    uint16_t sp = mean[g] ? (uint16_t)((unsigned long)(s.mx - s.mn) * 50UL / mean[g]) : 0;
    if(s.n >= 2 && sp > TC_SPREAD_PCT) uneven++;
    tcName(g); Serial.print(','); Serial.print(s.n); Serial.print(',');
    Serial.print(mean[g]); Serial.print(','); Serial.print(s.mn); Serial.print(',');
    Serial.print(s.mx); Serial.print(','); Serial.print(sp); Serial.print(',');
    Serial.print(tcExpected(g)); Serial.print(','); Serial.print(s.slow); Serial.print(',');
    Serial.print(s.fast); Serial.print(','); Serial.print(s.extra); Serial.print(',');
    Serial.println(s.fail);
  }

  /* the axle, from the 90s: + = short of the crossing, - = past it */
  int16_t axMm = 0;
  if(tcAxleN){
    float a = (float)tcAxleSum / tcAxleN * 0.0017453f;      /* tenths of a degree to radians */
    axMm = (int16_t)(SENSOR_AHEAD_CM * 10.0 * sin(a) + (a >= 0 ? 0.5 : -0.5));
    Serial.print(F("axle off the crossing (from the 90s): about ")); logTenth(abs(axMm));
    Serial.println(axMm > 0 ? F(" cm short of it") : axMm < 0 ? F(" cm past it") : F(" cm"));
  }

  Serial.println(F("verdict:"));
  if(code){
    Serial.print(F("- STOPPED at turn ")); Serial.print(tcTurn);
    if(tcNow >= 0){ Serial.print(F(" (")); tcName(tcNow); Serial.print(')'); }
    Serial.print(F(": ")); Serial.println(why);
    if(code == 6)  tcSay(F("the turn never found the line: the robot was moved or is not on a crossing, a wheel does not turn at TURN_SP (dead band, weak battery), or a sensor misses the line. MODE_MOTOR_CHECK, then calibrate again."));
    if(code == 10 || code == 11) extra++;
  }
  if(!turnTimed())
    tcSay(F("not calibrated: the expected times are guesses, so SLOW and FAST mean little. Run MODE_CALIBRATE first."));
  /* left against right, the 90s (the 180s if no 90 finished) */
  uint8_t a = (tcSt[0].n && tcSt[1].n) ? 0 : 2;
  if(tcSt[a].n && tcSt[a + 1].n){
    if(mean[a] * 100 > mean[a + 1] * 125)
      tcSay(F("left turns much slower than right: the weak left wheel. Calibrate again (MODE_CALIBRATE, fresh battery)."));
    else if(mean[a + 1] * 100 > mean[a] * 125)
      tcSay(F("right turns much slower than left: the right wheel is slower than the calibration knows. Calibrate again."));
  }
  /* [E33] wheels on MID but not on a crossing: a 90 finds no column arm and
   * turns on to MID again, about twice as long, while a 180 is normal */
  if(tcSt[0].n && tcSt[1].n && mean[0] * 10 > tcExpected(0) * 16 && mean[1] * 10 > tcExpected(1) * 16
     && (!tcSt[2].n || mean[2] * 10 < tcExpected(2) * 13))
    tcSay(F("the 90s took about twice the expected time and the 180s did not: the wheels are on MID but NOT on a crossing. Put the WHEELS on C2 MID or C3 MID and check again."));
  bool slow = false, fast = false;
  for(uint8_t g=0;g<4;g++){
    if(!tcSt[g].n) continue;
    if(mean[g] * 100 > tcExpected(g) * TC_SLOW_PCT) slow = true;
    if(mean[g] * 100 < tcExpected(g) * TC_FAST_PCT) fast = true;
  }
  if(slow) tcSay(F("turns slower than the calibration expects: the WHEELS not on a crossing (C2 MID or C3 MID), a weak battery, or an old calibration. Check where the wheels are, then fresh battery, calibrate again."));
  if(fast) tcSay(F("turns faster than the calibration expects: a fresher battery than at the calibration? Calibrate again."));
  if(extra)
    tcSay(F("a turn passed an extra line: it spins too fast to stop on the first line. TURN_SP too high for this robot, or the dead band: calibrate again; if it stays, lower TURN_SP and TURN_SLOW_SP a little."));
  bool axle = abs(axMm) >= TC_AXLE_MM;
  if(axle){
    Serial.print(F("- the axle was about ")); logTenth(abs(axMm));
    Serial.print(axMm > 0 ? F(" cm short of the crossing (the robot too far back)") : F(" cm past the crossing (the robot too far forward)"));
    Serial.println(F(": the 90s alternate short and long. Put the WHEELS exactly over the crossing and check again."));
  }
  if(uneven)
    tcSay(F("turn times vary a lot from one turn to the next: the axle not on the crossing, or a wheel slips."));
  if(tcOff || code == 7)
    tcSay(F("off line after turns: the axle was not on the crossing (put the WHEELS exactly over it, facing along MID)."));
  if(!code && !fails && !extra && !uneven && !tcOff && !slow && !fast && !axle)
    tcSay(turnTimed() ? F("all turns OK: the turns are ready for the mission.")
                      : F("all turns OK with the default values: calibrate before the mission."));
  Serial.println(F("=== E33 TURN CHECK END ==="));
}

/* A failed turn: fault() has stopped the motors. Print the turn and the
 * report so far; fault() then prints the fault and blinks its number. */
void tcFaultHook(uint8_t code, const __FlashStringHelper *why){
  if(tcNow >= 0){
    tcSt[tcNow].fail++;
    tcLine(tcNow, (code == 7) ? F("OFF LINE") : F("FAILED"));
  }
  tcSummary(code, why);
}

/* Before a 180: the line under the middle two sensors (error -1 .. 1), so
 * spinTurn() counts it as it leaves, whichever way the robot turns. */
void tcCentre(){
  for(uint8_t k=0;k<20;k++){
    uint8_t m = scanBar();
    int e = errorOfMask(m);
    if(!m || abs(e) <= 1) return;
    int d = (e > 0) ? 1 : -1;            /* line on the right: turn right */
    setMotor(d * TURN_SLOW_SP, -d * TURN_SLOW_SP);
    delay(12);
    setMotor(0, 0);
    delay(40);
  }
}

void tcDoTurn(uint8_t g, uint8_t i){
  int dir = (g & 1) ? 1 : -1;
  tcTurn++;
  tcNow = g;
  tcLines = 0;
  if(g & 2){
    /* A 180 on a four-arm crossing sweeps over one side arm before the road
     * back. The line under the bar at the start also counts as it leaves
     * when it sits in the middle or on the side the robot turns toward (see
     * spinTurn), so skip it too: 2 lines. Otherwise 1. */
    tcCentre();
    uint8_t m = scanBar();
    skip180 = (m && dir * errorOfMask(m) >= -1) ? 2 : 1;
    tcLines = skip180;
  }
  if(g == 0)      turnLeft90();          /* the mission's own turns */
  else if(g == 1) turnRight90();
  else if(g == 2) turnLeft180();
  else            turnRight180();
  delay(TC_PAUSE_MS);
  uint8_t m = scanBar();
  uint8_t b = popcount8(m);
  int e = errorOfMask(m);
  if(b < 1 || b > 3 || !isRun(m) || abs(e) > 2)
    fault(7, F("no line under the middle of the bar after the turn"));
  if(abs(e) >= 2) tcOff++;
  TcStat &s = tcSt[g];
  uint16_t t = (uint16_t)lastTurnMs;
  if(!s.n || t < s.mn) s.mn = t;
  if(t > s.mx) s.mx = t;
  s.n++; s.sum += t;
  unsigned long x = tcExpected(g);
  const __FlashStringHelper *st = F("OK");
  if(lastTurnLines > tcLines){ s.extra++; st = F("EXTRA LINE"); }
  else if((unsigned long)t * 100 > x * TC_SLOW_PCT){ s.slow++; st = F("SLOW"); }
  else if((unsigned long)t * 100 < x * TC_FAST_PCT){ s.fast++; st = F("FAST"); }
  tcLine(g, st);
  tcNow = -1;
  if(!(g & 2)){
    tcQ[i & 3] = t;
    if((i & 3) == 3){                    /* a whole circle of 90s */
      long all = (long)tcQ[0] + tcQ[1] + tcQ[2] + tcQ[3];
      tcAxleSum += 900L * ((long)tcQ[1] + tcQ[2] - tcQ[0] - tcQ[3]) / all;
      tcAxleN++;
    }
  }
}

/* Wait until the line is steady under the middle of the bar for 1.5 s. */
void tcWaitLine(){
  Serial.println(F("waiting for the line steady under the middle of the bar..."));
  unsigned long tOk = 0, tPrint = 0;
  while(true){
    uint8_t m = scanBar();
    uint8_t b = popcount8(m);
    bool ok = (b >= 1 && b <= 3 && isRun(m) && abs(errorOfMask(m)) <= 2);
    unsigned long now = millis();
    if(ok){
      if(!tOk) tOk = now;
      digitalWrite(LED_PIN, 1);
      if(now - tOk >= 1500) break;
    }else{
      tOk = 0;
      digitalWrite(LED_PIN, (now / 500) & 1);
    }
    if(now - tPrint >= 1000){
      tPrint = now;
      Serial.print(F("  bar ")); printMask(m);
      Serial.println(ok ? F("  line OK") : (b >= 5) ? F("  on a crossing, or lifted: the WHEELS go on the crossing, not the bar")
                                                   : F("  no line in the middle: face along MID"));
    }
    delay(5);
  }
  digitalWrite(LED_PIN, 0);
}

void runTurnCheck(){
  faultHook = tcFaultHook;
  Serial.println(F("=== TURN CHECK: the robot turns on the spot, it never drives along a line ==="));
  Serial.println(F("Put the WHEELS (axle) exactly on a MID crossing with four arms (C2 MID or C3 MID),"));
  Serial.println(F("facing along MID, the line under the middle of the bar. Then step back."));
  Serial.print(F("Turns: 4 x left 90, 4 x right 90, 2 x left 180, 2 x right 180, "));
  Serial.print(TC_ROUNDS); Serial.println(F(" times. It ends facing the way it started,"));
  Serial.println(F("then prints a report to copy and send (from BEGIN to END, with the turn lines above it)."));
  if(!turnTimed()) Serial.println(F("NOT CALIBRATED: the expected times are guesses. Run MODE_CALIBRATE first."));
  bool first = true;
  while(true){
    if(first) waitEnter(30, 2);        /* (a repeat starts with the Enter below) */
    first = false;
    tcWaitLine();
    for(uint8_t g=0;g<4;g++){ tcSt[g].n = 0; tcSt[g].fail = 0; tcSt[g].slow = 0; tcSt[g].fast = 0;
                              tcSt[g].extra = 0; tcSt[g].mn = 0; tcSt[g].mx = 0; tcSt[g].sum = 0; }
    tcTurn = 0; tcOff = 0; tcAxleSum = 0; tcAxleN = 0;
    Serial.println(F("TURN CHECK: go"));
    for(uint8_t r=0;r<TC_ROUNDS;r++)
      for(uint8_t i=0;i<TC_TURNS;i++) tcDoTurn(tcKind(i), i);
    tcSummary(0, 0);
    Serial.println(F("\nPress Enter to do the check again (put the wheels back on the crossing first)."));
    while(!Serial.available()){ digitalWrite(LED_PIN, (millis() / 1000) & 1); delay(5); }
    delay(30);
    while(Serial.available()) Serial.read();
  }
}
#endif

#endif
