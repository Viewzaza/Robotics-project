/* =====================================================================
 *  robot_basic.ino -- the e33 mission, written the way the slides teach it
 * =====================================================================
 *
 *  Structure is exactly robot10: setup() calls beginFnc(), loop() counts
 *  crossings and switches on the count, and the three helpers turn90(),
 *  keep_item() and place_item() are the ones from robot10 pages 4, 6 and 9.
 *  The library it sits on is robot11's controlLibrary.h.
 *
 *  Only the ROUTE is different from robot10, because your field (e33) is not
 *  the field in that deck.
 *
 *  ---------------------------------------------------------------------
 *  YOUR FIELD (e33)
 *
 *          o1                                  o3
 *     -----+-------+-------+-------+-----      TOP
 *          |       |       |       |
 *     --[R]>-------+-------+-------+-----      MID
 *          |       |       |       |
 *     -----+-------+-------+-------+-----      BOT
 *          o2      x3      x2      x1
 *          C1      C2      C3      C4
 *
 *    o = object, x = where it goes, [R]> = start, on the middle line at C1
 *    facing right.
 *
 *      object 1 (top of C1)     ->  x1 (bottom of C4)
 *      object 2 (bottom of C1)  ->  x2 (bottom of C3)
 *      object 3 (top of C4)     ->  x3 (bottom of C2)
 *
 *  The worksheet lets you collect them in any order. This does 3, then 1,
 *  then 2, because the robot starts facing right so object 3 is nearest, and
 *  because the route never drives along the bottom line, so it cannot knock
 *  over something it has already placed.
 *
 *  ---------------------------------------------------------------------
 *  HOW THE COUNTING WORKS  (robot07 pages 9-11, robot08 pages 10-11)
 *
 *  countGrid() adds 1 each time the sensor bar crosses a line. The crossing
 *  the robot is sitting on when you switch it on is NOT counted, so the first
 *  crossing it meets is 1. Every case that does something also does
 *  numGride++ of its own, which is why the case numbers skip.
 *
 *   n    where        turn                what it does
 *   1    (C2,MID)     keep going east     followLine
 *   2    (C3,MID)     keep going east     followLine
 *   3    (C4,MID)     east -> north       turn90("LEFT")
 *   5    (C4,TOP)     north -> south      keep_item   pick up object 3
 *   7    (C4,MID)     south -> west       turn90("RIGHT")
 *   9    (C3,MID)     keep going west     followLine
 *  10    (C2,MID)     west -> south       turn90("LEFT")
 *  12    (C2,BOT)     south -> north      place_item  put object 3 on x3
 *  14    (C2,MID)     north -> west       turn90("LEFT")
 *  16    (C1,MID)     west -> north       turn90("RIGHT")
 *  18    (C1,TOP)     north -> south      keep_item   pick up object 1
 *  20    (C1,MID)     south -> east       turn90("LEFT")
 *  22    (C2,MID)     keep going east     followLine
 *  23    (C3,MID)     keep going east     followLine
 *  24    (C4,MID)     east -> south       turn90("RIGHT")
 *  26    (C4,BOT)     south -> north      place_item  put object 1 on x1
 *  28    (C4,MID)     north -> west       turn90("LEFT")
 *  30    (C3,MID)     keep going west     followLine
 *  31    (C2,MID)     keep going west     followLine
 *  32    (C1,MID)     west -> south       turn90("LEFT")
 *  34    (C1,BOT)     south -> north      keep_item   pick up object 2
 *  36    (C1,MID)     north -> east       turn90("RIGHT")
 *  38    (C2,MID)     keep going east     followLine
 *  39    (C3,MID)     east -> south       turn90("RIGHT")
 *  41    (C3,BOT)     stop there          place_item("STOP")  object 2 on x2
 *  42    finished     stopRobot()
 * ===================================================================== */

#include "controlLibrary.h"

int numGride = 0;

void turn90(String direction);
void keep_item(String direction);
void place_item(String direction);

void setup() {
  Serial.begin(9600);
  beginFnc();
}

void loop() {
  numGride = countGrid(numGride);
  switch(numGride){

    /* ---- object 3: top of C4, goes to x3 at the bottom of C2 ---- */
    case  3: turn90("LEFT");      numGride++; break;
    case  5: keep_item("RIGHT");  numGride++; break;
    case  7: turn90("RIGHT");     numGride++; break;
    case 10: turn90("LEFT");      numGride++; break;
    case 12: place_item("RIGHT"); numGride++; break;

    /* ---- object 1: top of C1, goes to x1 at the bottom of C4 ---- */
    case 14: turn90("LEFT");      numGride++; break;
    case 16: turn90("RIGHT");     numGride++; break;
    case 18: keep_item("RIGHT");  numGride++; break;
    case 20: turn90("LEFT");      numGride++; break;
    case 24: turn90("RIGHT");     numGride++; break;
    case 26: place_item("RIGHT"); numGride++; break;

    /* ---- object 2: bottom of C1, goes to x2 at the bottom of C3 ---- */
    case 28: turn90("LEFT");      numGride++; break;
    case 32: turn90("LEFT");      numGride++; break;
    case 34: keep_item("RIGHT");  numGride++; break;
    case 36: turn90("RIGHT");     numGride++; break;
    case 39: turn90("RIGHT");     numGride++; break;
    case 41: place_item("STOP");  numGride++; break;

    case 42: stopRobot(); break;

    default : followLine();
  }
}

/* =====================================================================
 *  The three helpers, from robot10 pages 4, 6 and 9.
 * ===================================================================== */

/* robot10 page 4 */
void turn90(String direction){
  trace(numGride, direction == "RIGHT" ? F("turn90 RIGHT") : F("turn90 LEFT"));
  moveFor();
  delay(TURN_NUDGE_MS);
  stopRobot();
  if(direction == "RIGHT"){turnRight90();}
  else{turnLeft90();}
  stopRobot();
  leaveCrossing();
  clearPid();
}

/* robot10 page 6 */
void keep_item(String direction){
  trace(numGride, F("keep_item  -- picking up"));
  moveFor();
  delay(PICK_NUDGE_MS);
  stopRobot();
  delay(500);
  keepup_object();
  if(direction == "RIGHT"){turnRight180();}
  else{turnLeft180();}
  stopRobot();
  leaveCrossing();
  clearPid();
}

/* robot10 page 9 */
void place_item(String direction){
  trace(numGride, F("place_item -- putting down"));
  moveFor();
  delay(PLACE_NUDGE_MS);
  stopRobot();
  delay(500);
  put_object();
  if(direction == "RIGHT"){
    arm_over_head();//ยกแขนสูง
    turnRight180();
  }else if(direction == "LEFT"){
    arm_over_head();//ยกแขนสูง
    turnLeft180();
  }
  stopRobot();
  put_object();
  leaveCrossing();
  clearPid();
}
