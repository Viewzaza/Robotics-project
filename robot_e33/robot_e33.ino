/* =====================================================================
 *  robot_e33.ino: the e33 mission, written the way the slides teach it
 * =====================================================================
 *
 *  Same shape as robot10: setup() calls beginFnc(), loop() counts crossings
 *  with countGrid() and a switch acts on the count, and the helpers
 *  turn90(), keep_item() and place_item() are the ones from robot10 pages
 *  4, 6 and 9. The library is robot11's controlLibrary.h, with every
 *  change marked [E33].
 *
 *  Your numbers are in calibration.h. That file also chooses the mode:
 *  MODE_MISSION (this route), MODE_CALIBRATE, MODE_METER, MODE_MOTOR_CHECK.
 *
 *  ---------------------------------------------------------------------
 *  THE FIELD (e33)
 *
 *          o1                                  o3
 *     -----+-------+-------+-------+-----      TOP
 *          |       |       |       |
 *     ----[R]>-----+-------+-------+-----      MID
 *          |       |       |       |
 *     -----+-------+-------+-------+-----      BOT
 *          o2      x3      x2      x1
 *          C1      C2      C3      C4
 *
 *    START: wheels (axle) over the C1 line, on MID, facing east (right).
 *    The bar is then 9.5 cm past C1. Switch on and step back; the robot
 *    starts by itself once it has seen the line steadily for 1.5 s.
 *
 *      object 3 (top of C4)     ->  x3 (bottom of C2)
 *      object 1 (top of C1)     ->  x1 (bottom of C4)
 *      object 2 (bottom of C1)  ->  x2 (bottom of C3)
 *
 *  Order 3, 1, 2: object 3 is nearest the start, and the route never
 *  drives along the bottom line, so it cannot knock over what it placed.
 *
 *  ---------------------------------------------------------------------
 *  THE COUNT  (robot07 pages 9-11, robot10)
 *
 *  countGrid() adds 1 at every crossing. Every case that does something
 *  also does numGride++ itself, which is why the case numbers skip.
 *  Numbers with no case (1, 2, 9, ...) are crossings the robot drives
 *  straight over.
 *
 *   n   where     heading       what it does                   checked
 *   1   C2 MID    east          drive over                     line ahead
 *   2   C3 MID    east          drive over                     line ahead
 *   3   C4 MID    east->north   turn90 LEFT                    line ahead
 *   5   C4 TOP    north->south  keep_item  pick up object 3    NO line ahead  (turns LEFT)
 *   7   C4 MID    south->west   turn90 RIGHT                   line ahead
 *   9   C3 MID    west          drive over                     line ahead
 *  10   C2 MID    west->south   turn90 LEFT                    line ahead
 *  12   C2 BOT    south->north  place_item object 3 on x3      NO line ahead  (turns LEFT)
 *  14   C2 MID    north->west   turn90 LEFT                    line ahead
 *  16   C1 MID    west->north   turn90 RIGHT                   line ahead
 *  18   C1 TOP    north->south  keep_item  pick up object 1    NO line ahead  (turns RIGHT)
 *  20   C1 MID    south->east   turn90 LEFT                    line ahead
 *  22   C2 MID    east          drive over                     line ahead
 *  23   C3 MID    east          drive over                     line ahead
 *  24   C4 MID    east->south   turn90 RIGHT                   line ahead
 *  26   C4 BOT    south->north  place_item object 1 on x1      NO line ahead  (turns RIGHT)
 *  28   C4 MID    north->west   turn90 LEFT                    line ahead
 *  30   C3 MID    west          drive over                     line ahead
 *  31   C2 MID    west          drive over                     line ahead
 *  32   C1 MID    west->south   turn90 LEFT                    line ahead
 *  34   C1 BOT    south->north  keep_item  pick up object 2    NO line ahead  (turns LEFT)
 *  36   C1 MID    north->east   turn90 RIGHT                   line ahead
 *  38   C2 MID    east          drive over                     line ahead
 *  39   C3 MID    east->south   turn90 RIGHT                   line ahead
 *  41   C3 BOT    stop there    place_item object 2 on x2      NO line ahead
 *  42   finished
 *
 *  AT EVERY CROSSING (inside countGrid, see controlLibrary.h) the robot
 *  checks, before it counts:
 *    - distance: not far too early (a double count), not a whole cell
 *      late (a crossing was missed - if it was a drive-over crossing it is
 *      counted, if it was a turn or an end the robot stops safely);
 *    - shape: both ends of the bar lit (a real crossing, not a branch seen
 *      while driving along the wrong line);
 *    - what is just past it: a MID crossing has line ahead, the end of a
 *      column has none. keep_item / place_item will not act where there
 *      is line ahead: they wait for the next crossing once, then stop.
 *  One line is printed on Serial (115200) at every crossing and action.
 *  If anything does not fit, the robot stops, prints why, and the LED on
 *  D13 blinks the fault number.
 * ===================================================================== */

#include "calibration.h"
#include "controlLibrary.h"
#if ROBOT_MODE != MODE_MISSION
  #include "calibrationMode.h"
#endif

int numGride = 0;

void turn90(String direction);
bool keep_item(String direction);
bool place_item(String direction);
void startMission();
void finishMission();

void setup() {
  Serial.begin(SERIAL_BAUD);
  beginFnc();
#if ROBOT_MODE == MODE_CALIBRATE
  runCalibration();           /* never comes back */
#elif ROBOT_MODE == MODE_METER
  runMeter();
#elif ROBOT_MODE == MODE_MOTOR_CHECK
  runMotorCheck();
#else
  startMission();
#endif
}

void loop() {
#if ROBOT_MODE == MODE_MISSION  /* (the other modes never get here; leaving the
                                 * route out keeps them small enough for the Nano) */
  /* [E33] after the last object (41 -> 42) there is nothing more to count:
   * with the bar resting on the BOT line, countGrid() would ride over it
   * with the gripper open and push object 2 off x2 */
  if(numGride < 42) numGride = countGrid(numGride);
  switch(numGride){

    /* ---- object 3: top of C4, goes to x3 at the bottom of C2 ---- */
    case  3: turn90("LEFT");                                    numGride++; break;
    case  5: if(keep_item("LEFT"))   numGride++; else numGride--;           break;
    case  7: turn90("RIGHT");                                   numGride++; break;
    case 10: turn90("LEFT");                                    numGride++; break;
    case 12: if(place_item("LEFT"))  numGride++; else numGride--;           break;

    /* ---- object 1: top of C1, goes to x1 at the bottom of C4 ---- */
    case 14: turn90("LEFT");                                    numGride++; break;
    case 16: turn90("RIGHT");                                   numGride++; break;
    case 18: if(keep_item("RIGHT"))  numGride++; else numGride--;           break;
    case 20: turn90("LEFT");                                    numGride++; break;
    case 24: turn90("RIGHT");                                   numGride++; break;
    case 26: if(place_item("RIGHT")) numGride++; else numGride--;           break;

    /* ---- object 2: bottom of C1, goes to x2 at the bottom of C3 ---- */
    case 28: turn90("LEFT");                                    numGride++; break;
    case 32: turn90("LEFT");                                    numGride++; break;
    case 34: if(keep_item("LEFT"))   numGride++; else numGride--;           break;
    case 36: turn90("RIGHT");                                   numGride++; break;
    case 39: turn90("RIGHT");                                   numGride++; break;
    case 41: if(place_item("STOP"))  numGride++; else numGride--;           break;

    case 42: finishMission(); break;

    default : followLine();
  }
#endif
}

/* What each count number is, and what that crossing must look like.
 * countGrid() uses this to check every crossing. It must agree with the
 * switch above.
 *
 *   K_PASS / K_TURN / K_END   what the switch does there
 *   HALF_OK_L / HALF_OK_R     a crossing at the edge of the field: the line
 *                             sticking out past C1 / C4 (the overhang) may be
 *                             too short to reach the bar, so only the end of
 *                             the bar on the FIELD side (left / right, as the
 *                             robot faces) has to light
 *   AHEAD_MAY_BLANK           heading out of the field along MID at C1 / C4:
 *                             the only line ahead is the overhang, which may
 *                             be too short to see
 */
uint8_t caseInfo(int n){
  switch(n){
    case  3: return K_TURN | AHEAD_MAY_BLANK;   /* C4 MID heading east        */
    case  5: return K_END  | HALF_OK_L;         /* C4 TOP heading north       */
    case  7: return K_TURN | HALF_OK_R;         /* C4 MID heading south       */
    case 10: return K_TURN;                     /* C2 MID heading west        */
    case 12: return K_END;                      /* C2 BOT heading south       */
    case 14: return K_TURN;                     /* C2 MID heading north       */
    case 16: return K_TURN | AHEAD_MAY_BLANK;   /* C1 MID heading west        */
    case 18: return K_END  | HALF_OK_R;         /* C1 TOP heading north       */
    case 20: return K_TURN | HALF_OK_L;         /* C1 MID heading south       */
    case 24: return K_TURN | AHEAD_MAY_BLANK;   /* C4 MID heading east        */
    case 26: return K_END  | HALF_OK_R;         /* C4 BOT heading south       */
    case 28: return K_TURN | HALF_OK_L;         /* C4 MID heading north       */
    case 32: return K_TURN | AHEAD_MAY_BLANK;   /* C1 MID heading west        */
    case 34: return K_END  | HALF_OK_L;         /* C1 BOT heading south       */
    case 36: return K_TURN | HALF_OK_R;         /* C1 MID heading north       */
    case 39: return K_TURN;                     /* C3 MID heading east        */
    case 41: return K_END;                      /* C3 BOT heading south       */
    default: return K_PASS;                     /* C2 / C3 MID, straight over */
  }
}

/* =====================================================================
 *  START  [E33]
 *  No button and no calibration motion: the robot waits, standing still,
 *  until the bar has seen the line steadily for 1.5 s. While it waits it
 *  prints what the bar sees, so "it does not move" always has a reason.
 * ===================================================================== */
void startMission(){
  Serial.println(F("MISSION: waiting for the line under the bar..."));
  unsigned long tOk = 0, tPrint = 0, tNone = millis(), tBad = 0;
  /* were the sensor levels ever measured on this robot? */
  bool sensorsKnown = (calSource == 2) ||
    (calSource == 1 && (calItem(calStatus, CI_SENSOR) == ST_PASS || calItem(calStatus, CI_SENSOR) == ST_WEAK));
  while(true){
    uint8_t m = scanBar();
    uint8_t b = popcount8(m);
    bool ok = (b >= 1 && b <= 3 && isRun(m) && abs(errorOfMask(m)) <= 5) || b >= 5;
    unsigned long now = millis();
    RawLook r;
    /* [E33] Never calibrated and nearly the whole bar lit, yet standing still
     * the bar shows ONE line: that is not a crossing. The default levels are
     * wrong for this bar (its white reads above them, or its black reads
     * LOW) and must be measured below, not driven on blind. */
    if(!sensorsKnown && b >= 5 && rawLook(r)) ok = false;
    if(ok){
      if(!tOk) tOk = now;
      tBad = 0;
      digitalWrite(LED_PIN, 1);
      if(now - tOk >= 1500) break;
    }else{
      /* [E33] with measured levels one noisy reading does not restart the
       * 1.5 s (0.1 s of no line does); with the default levels any doubt
       * restarts it, so the standing-still measurement gets its turn */
      if(!tBad) tBad = now;
      if(now - tBad > 100 || !sensorsKnown) tOk = 0;
      digitalWrite(LED_PIN, (now / 500) & 1);
      /* [E33] Never calibrated, and the default levels do not see the line
       * (a line fainter than the default threshold, or a bar whose black
       * reads LOW). Measure the levels standing still from the line under
       * the bar, with no motion, rather than wait for ever. */
      if(!sensorsKnown && now - tNone > 2000 && rawLook(r)){
        calFromLook(r);
        sensorsKnown = true;
        Serial.print(F("  default sensor levels could not see the line; measured standing still: white "));
        Serial.print(r.lineLow ? 1023 - r.white : r.white); Serial.print(F(", line "));
        Serial.println(r.lineLow ? 1023 - r.black : r.black);
        continue;                       /* look again with the new levels */
      }
    }
    if(now - tPrint >= 1000){
      tPrint = now;
      Serial.print(F("  bar ")); printMask(m);
      Serial.print(F("  raw"));
      for(uint8_t i=0;i<8;i++){ Serial.print(' '); Serial.print(cal.lineLow ? 1023 - rawV[i] : rawV[i]); }
      Serial.println(ok ? F("  line OK") : F("  no line: put the bar on MID"));
    }
  }
  digitalWrite(LED_PIN, 0);
  /* standing on a crossing: do not count it, and C2 is a whole cell away */
  gridArmed = !checkGrid();
  if(gridArmed) setLeg(CELL_CM - SENSOR_AHEAD_CM + TAPE_W_CM / 2);
  else          setLeg(CELL_CM);
  clearPid();
  sp = SP_START;
  runMark = RUN_MARK;                    /* a restart from now on is not a new start */
  Serial.println(F("MISSION: go"));
}

/* =====================================================================
 *  The three helpers, from robot10 pages 4, 6 and 9.
 *  [E33] The slides nudge with "moveFor(); delay(50);". Here the robot
 *  drives a measured distance instead, because the bar is 9.5 cm ahead of
 *  the wheels: at the moment the bar leaves a crossing, the wheels are
 *  still 8.6 cm short of it.
 * ===================================================================== */

/* robot10 page 4 */
void turn90(String direction){
  /* CHECK: every MID crossing has line straight ahead. None = the robot is
   * really at the end of a column, and turning here would lose the route.
   * (Except heading out of the field at C1 / C4, where the only line ahead
   * is the overhang.) */
  if(crossAhead == AH_BLANK && !(caseInfo(numGride) & AHEAD_MAY_BLANK))
    fault(8, F("turn90: no line ahead, this is not a MID crossing"));
  traceEvent(direction == "RIGHT" ? F("turn90 RIGHT") : F("turn90 LEFT"));
  rollToBarPast(SENSOR_AHEAD_CM);          /* wheels onto the crossing */
  if(direction == "RIGHT"){turnRight90();}
  else{turnLeft90();}
  /* next leg: up/down a column if the next action is an end, else along MID */
  float gap = (caseKind(numGride + 2) == K_END) ? ROW_CM : CELL_CM;
  setLeg(gap - SENSOR_AHEAD_CM + TAPE_W_CM / 2);
  clearPid();
}

/* [E33] CHECK at a column end: there must be no line straight ahead. If
 * there is, this was not the end (something was counted on the way): keep
 * going and treat the next crossing as this one. Only once. */
uint8_t endRetries = 0;
bool atColumnEnd(){
  if(crossAhead != AH_LINE){ endRetries = 0; return true; }
  if(++endRetries > 1) fault(9, F("still line ahead at the second try: the column end was not found"));
  nRetried ++;
  restoreLeg();
  traceEvent(F("RETRY: line ahead, not the end yet"));
  return false;
}

/* [E33] 180 at the end of a column. The wheels are ~7 cm before the end
 * line, so the spinning bar first sweeps over the end line, then finds the
 * column going back: skip one line.
 * At a CORNER (C1 or C4) the robot must turn TOWARD THE FIELD. Turning the
 * other way swings the bar over the short overhang: depending on its length
 * the bar sees the end line there or not, the count of lines is wrong, and
 * the robot stops facing along the TOP / BOT line. That is the "it goes
 * wild when it turns back" failure at object 3. So the directions in the
 * switch are: C4 TOP LEFT, C1 TOP RIGHT, C4 BOT RIGHT, C1 BOT LEFT; at
 * C2 BOT the front swings away from object 2 (LEFT). */
void turnAround(String direction){
  float s = barPast() - SENSOR_AHEAD_CM;   /* wheels past the end line (negative = before it) */
  /* the spinning bar meets the end line once on the field side as long as
   * the bar got past it; if the line is still under the bar at the start of
   * the spin, spinTurn() counts it as it leaves */
  skip180 = (barPast() > -0.5) ? 1 : 0;
  if(direction == "RIGHT"){turnRight180();}
  else{turnLeft180();}
  /* the bar is now (9.5 - s) back from the end line; MID is ROW_CM from it */
  setLeg(ROW_CM + s - SENSOR_AHEAD_CM + TAPE_W_CM / 2);
  clearPid();
}

/* robot10 page 6 */
bool keep_item(String direction){
  if(!atColumnEnd()) return false;
  traceEvent(F("keep_item: picking up"));
  rollToBarPast(SENSOR_AHEAD_CM + OBJ_BEYOND_CM - GRIP_REACH_CM);   /* jaws around the object */
  delay(500);
  keepup_object();
  turnAround(direction);
  return true;
}

/* robot10 page 9 */
bool place_item(String direction){
  if(!atColumnEnd()) return false;
  traceEvent(F("place_item: putting down"));
  rollToBarPast(SENSOR_AHEAD_CM + TGT_BEYOND_CM - GRIP_REACH_CM);   /* object over the target */
  delay(500);
  put_object();
  if(direction == "RIGHT" || direction == "LEFT"){
    arm_over_head();//ยกแขนสูง
    turnAround(direction);
    put_object();
  }
  return true;
}

void finishMission(){
  stopRobot();
  digitalWrite(STBY, 0);
  runMark = 0;
  Serial.print(F("DONE. crossings not counted (too early): ")); Serial.print(nRejected);
  Serial.print(F("  missed and counted: ")); Serial.print(nCredited);
  Serial.print(F("  end retries: ")); Serial.println(nRetried);
  while(1){ digitalWrite(LED_PIN, (millis() / 1000) & 1); }
}
