/* =====================================================================
 *  robot_countGride.ino      Robot mission (worksheet e33)
 * =====================================================================
 *
 *  Field  (o = object, x = drop position, [R]> = robot start, facing right)
 *
 *          o1                                  o3
 *     ─────┬───────┬───────┬───────┬─────      <- TOP
 *          │       │       │       │
 *     ──[R]>───────┼───────┼───────┼─────      <- MID
 *          │       │       │       │
 *     ─────┴───────┴───────┴───────┴─────      <- BOT
 *          o2      x3      x2      x1
 *          C1      C2      C3      C4
 *
 *  Object                     ->  must be placed at
 *    o1 = top of C1           ->  x1 = bottom of C4
 *    o2 = bottom of C1        ->  x2 = bottom of C3
 *    o3 = top of C4           ->  x3 = bottom of C2
 *
 *  The worksheet allows starting with any object number. This program runs
 *  the order  3 -> 1 -> 2, because the robot starts facing right and o3 is
 *  the right-most object. The route never travels along the bottom line, so
 *  it cannot disturb objects it has already placed.
 *
 * ---------------------------------------------------------------------
 *  How numGride is counted (same convention as slides robot08/09/10)
 *  - countGrid() adds 1 every time a crossing line is detected
 *  - the intersection the robot starts on, (C1,MID), is NOT counted;
 *    the first crossing it meets is 1
 *  - every case that performs an action also does numGride++, so the
 *    case numbers skip
 *
 *  Route table
 *  n    Point      Heading         Action
 *  ---  ---------  --------------  ------------------------------------
 *   1   (C2,MID)   E               followLine
 *   2   (C3,MID)   E               followLine
 *   3   (C4,MID)   E -> N          turn90("LEFT")
 *   5   (C4,TOP)   N -> S          keep_item    pick object 3
 *   7   (C4,MID)   S -> W          turn90("RIGHT")
 *   9   (C3,MID)   W               followLine
 *  10   (C2,MID)   W -> S          turn90("LEFT")
 *  12   (C2,BOT)   S -> N          place_item   drop object 3 at x3
 *  14   (C2,MID)   N -> W          turn90("LEFT")
 *  16   (C1,MID)   W -> N          turn90("RIGHT")
 *  18   (C1,TOP)   N -> S          keep_item    pick object 1
 *  20   (C1,MID)   S -> E          turn90("LEFT")
 *  22   (C2,MID)   E               followLine
 *  23   (C3,MID)   E               followLine
 *  24   (C4,MID)   E -> S          turn90("RIGHT")
 *  26   (C4,BOT)   S -> N          place_item   drop object 1 at x1
 *  28   (C4,MID)   N -> W          turn90("LEFT")
 *  30   (C3,MID)   W               followLine
 *  31   (C2,MID)   W               followLine
 *  32   (C1,MID)   W -> S          turn90("LEFT")
 *  34   (C1,BOT)   S -> N          keep_item    pick object 2
 *  36   (C1,MID)   N -> E          turn90("RIGHT")
 *  38   (C2,MID)   E               followLine
 *  39   (C3,MID)   E -> S          turn90("RIGHT")
 *  41   (C3,BOT)   S               place_item("STOP")  drop object 2 at x2
 *  42   mission complete -> stopRobot()
 * ===================================================================== */

#include "controlLibrary.h"

/* ---- field tuning ---------------------------------------------------
 * If the robot turns too early or too late at an intersection, or fails to
 * grip the object, these three values are what to change.
 */
#define NUDGE_TURN    50   // ms driven past an intersection before turning
#define NUDGE_PICK    50   // ms driven toward the object before gripping
#define NUDGE_PLACE   30   // ms driven forward before releasing the object

#define END_GRIDE     42   // reaching this count means the mission is done

int numGride = 0;

void turn90(String direction);
void keep_item(String direction);
void place_item(String direction);
void leaveGride();

void setup() {
  Serial.begin(9600);
  beginFnc();
}

void loop() {
  // Mission finished: hold still. Without this, countGrid() could keep
  // counting and the default case would drive the robot off the field.
  if (numGride >= END_GRIDE) { stopRobot(); return; }

  numGride = countGrid(numGride);

  switch (numGride) {

    /* ---------- object 3: top of C4  ->  drop at x3, bottom of C2 ------- */
    case  3: turn90("LEFT");       numGride++; break;  // (C4,MID) face north
    case  5: keep_item("RIGHT");   numGride++; break;  // (C4,TOP) pick object 3
    case  7: turn90("RIGHT");      numGride++; break;  // (C4,MID) face west
    case 10: turn90("LEFT");       numGride++; break;  // (C2,MID) face south
    case 12: place_item("RIGHT");  numGride++; break;  // (C2,BOT) drop object 3

    /* ---------- object 1: top of C1  ->  drop at x1, bottom of C4 ------- */
    case 14: turn90("LEFT");       numGride++; break;  // (C2,MID) face west
    case 16: turn90("RIGHT");      numGride++; break;  // (C1,MID) face north
    case 18: keep_item("RIGHT");   numGride++; break;  // (C1,TOP) pick object 1
    case 20: turn90("LEFT");       numGride++; break;  // (C1,MID) face east
    case 24: turn90("RIGHT");      numGride++; break;  // (C4,MID) face south
    case 26: place_item("RIGHT");  numGride++; break;  // (C4,BOT) drop object 1

    /* ---------- object 2: bottom of C1  ->  drop at x2, bottom of C3 ---- */
    case 28: turn90("LEFT");       numGride++; break;  // (C4,MID) face west
    case 32: turn90("LEFT");       numGride++; break;  // (C1,MID) face south
    case 34: keep_item("RIGHT");   numGride++; break;  // (C1,BOT) pick object 2
    case 36: turn90("RIGHT");      numGride++; break;  // (C1,MID) face east
    case 39: turn90("RIGHT");      numGride++; break;  // (C3,MID) face south
    case 41: place_item("STOP");   numGride++; break;  // (C3,BOT) drop object 2

    default: followLine();
  }
}

/* =====================================================================
 *  Helper wrappers (from slide robot10)
 * ===================================================================== */

// Turn 90 degrees at an intersection. direction = "RIGHT" or "LEFT".
void turn90(String direction) {
  moveFor();
  delay(NUDGE_TURN);
  stopRobot();
  if (direction == "RIGHT") { turnRight90(); }
  else                      { turnLeft90();  }
  stopRobot();
  leaveGride();
  clearPid();
}

// Pick up the object at the end of a line, then turn 180 degrees to head back.
void keep_item(String direction) {
  moveFor();
  delay(NUDGE_PICK);
  stopRobot();
  delay(500);
  keepup_object();
  if (direction == "RIGHT") { turnRight180(); }
  else                      { turnLeft180();  }
  stopRobot();
  leaveGride();
  clearPid();
}

// Place the object at the end of a line.
//   "RIGHT" / "LEFT" = place it, raise the arm, then turn 180 degrees
//   "STOP"           = place it and stay put (used for the last drop)
void place_item(String direction) {
  moveFor();
  delay(NUDGE_PLACE);
  stopRobot();
  delay(500);
  put_object();
  if (direction == "RIGHT") {
    arm_over_head();  // raise the arm so it clears the object just placed
    turnRight180();
  } else if (direction == "LEFT") {
    arm_over_head();  // raise the arm
    turnLeft180();
  }
  stopRobot();
  put_object();       // return the arm to its resting pose (down and open)
  if (direction != "STOP") { leaveGride(); }
  clearPid();
}

/* After a turn completes the robot can still be straddling the intersection
 * it just used. Left alone, countGrid() counts that same crossing twice, and
 * its internal while(checkGrid()) never exits because stopRobot() has already
 * braked the wheels, so the robot freezes in the middle of the field. Creep
 * forward slowly until the crossing is clear, then resume line following.
 */
void leaveGride() {
  unsigned long t0 = millis();
  sp = 60;
  while (checkGrid() && millis() - t0 < 800) { moveFor(); }
  stopRobot();
}
