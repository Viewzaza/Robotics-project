#include "controlLibrary.h"

/* robot_e33_emergency: the student's robot_original.ino with the route put in,
 * the way the slides do it (robot10: turn90, keep_item, place_item and one
 * "case N: helper(...); numGride++; break;" line per action).
 * Route 312: object 3 (top of C4) to x3 (bottom of C2), object 1 (top of C1)
 * to x1 (bottom of C4), object 2 (bottom of C1) to x2 (bottom of C3).
 * Start: the wheels over C1 on the MID line, facing east (toward C4).
 *
 * The NUDGE times are how long the robot drives straight on (moveFor) after
 * a crossing has been counted. The slides use 50 / 50 / 30 ms. This robot's
 * sensor bar sits 9.5 cm ahead of the wheels, so when a turn starts the
 * wheels are still about 6 to 8 cm short of the crossing (even at 90 ms).
 * The turn still finds the new line, often stopping early, and followLine()
 * then straightens the robot. In the simulator 50 to 100 ms work about the
 * same; 90 was a little better with a tired battery. A longer time (120 ms
 * and more) gets worse. PICK 30 and PLACE 30 put the gripper right over the
 * object and the drop spot. Details and numbers: README.md. */
#define NUDGE_TURN    90   // ms driven past an intersection before turning (slides: 50)
#define NUDGE_PICK    30   // ms driven toward the object before gripping (slides: 50)
#define NUDGE_PLACE   30   // ms driven forward before releasing the object (slides: 30)

#define END_GRIDE     42



void setup() {
  Serial.begin(9600);
  beginFnc();

}

int numGride = 0;

//robot10 p4: drive onto the crossing, then turn 90 degrees
void turn90(String direction){
        moveFor();
        delay(NUDGE_TURN);
        stopRobot();
        if(direction == "RIGHT"){turnRight90();}
        else{turnLeft90();}
        stopRobot();
        clearPid();
}

//robot10 p6: drive up to the object, pick it up, then turn 180 degrees
void keep_item(String direction){
        moveFor();
        delay(NUDGE_PICK);
        stopRobot();
        delay(500);
        keepup_object();
        if(direction == "RIGHT"){turnRight180();}
        else{turnLeft180();}
        stopRobot();
        clearPid();
}

//robot10 p9: drive onto the drop spot, put the object down, then turn
//180 degrees with the arm high ("STOP": no turn, the last object)
void place_item(String direction){
        moveFor();
        delay(NUDGE_PLACE);
        stopRobot();
        delay(500);
        put_object();
        if(direction == "RIGHT"){
          arm_over_head();//raise the arm high (ยกแขนสูง)
          turnRight180();
        }else if(direction == "LEFT"){
          arm_over_head();//raise the arm high (ยกแขนสูง)
          turnLeft180();
        }
        stopRobot();
        put_object();
        clearPid();
}

void loop() {
  //upSpeed();
  //moveFor();
  //Serial.println(getSensor());
  numGride = countGrid(numGride);
  switch(numGride){

    //object 3: top of C4, goes to x3 at the bottom of C2
    case 3: turn90("LEFT"); numGride++; break;        //C4 MID: turn north
    case 5: keep_item("LEFT"); numGride++; break;     //C4 TOP: pick object 3
    case 7: turn90("RIGHT"); numGride++; break;       //C4 MID: turn west
    case 10: turn90("LEFT"); numGride++; break;       //C2 MID: turn south
    case 12: place_item("LEFT"); numGride++; break;   //C2 BOT: place on x3

    //object 1: top of C1, goes to x1 at the bottom of C4
    case 14: turn90("LEFT"); numGride++; break;       //C2 MID: turn west
    case 16: turn90("RIGHT"); numGride++; break;      //C1 MID: turn north
    case 18: keep_item("RIGHT"); numGride++; break;   //C1 TOP: pick object 1
    case 20: turn90("LEFT"); numGride++; break;       //C1 MID: turn east
    case 24: turn90("RIGHT"); numGride++; break;      //C4 MID: turn south
    case 26: place_item("RIGHT"); numGride++; break;  //C4 BOT: place on x1

    //object 2: bottom of C1, goes to x2 at the bottom of C3
    case 28: turn90("LEFT"); numGride++; break;       //C4 MID: turn west
    case 32: turn90("LEFT"); numGride++; break;       //C1 MID: turn south
    case 34: keep_item("LEFT"); numGride++; break;    //C1 BOT: pick object 2
    case 36: turn90("RIGHT"); numGride++; break;      //C1 MID: turn east
    case 39: turn90("RIGHT"); numGride++; break;      //C3 MID: turn south
    case 41: place_item("STOP"); numGride++; break;   //C3 BOT: place on x2

    case END_GRIDE: stopRobot(); break;               //done: stay stopped

    default: followLine(); break;
  }

}
