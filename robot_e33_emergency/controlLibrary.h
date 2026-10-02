/* controlLibrary.h
 * Transcribed from slide deck robot11 ("circuit and code summary"),
 * pages 3-20. The Thai comments from the slides are kept in brackets
 * after the English so this file can still be diffed against the source.
 *
 * [E33 EMERGENCY] This is the student's own controlLibrary.h with only three
 * changes, all marked [E33 EMERGENCY]:
 *   1. turnRight90() spins the right motor BACKWARD, as the slides do
 *      (robot08 p3, robot11 p14). The student's copy drove it forward, so
 *      the robot rolled forward in an arc instead of turning on the spot.
 *   2. The servo angles are the ones measured on THIS gripper (the
 *      GRIP_ and ARM_ defines below). The old values are in the comments.
 *   3. followLine(): a robot that is standing still since stopRobot() and
 *      sees a pattern that getErrorInput() does not know (often ONE middle
 *      sensor on the line, e.g. 00100000) now drives straight on until it
 *      sees one it knows. Before, followLine() did nothing there, so the
 *      braked robot stood still for ever (the simulator found this after
 *      the 180 degree turn at the first drop spot).
 */

#include <Servo.h>
#include "pidLibrary.h"

/* [E33 EMERGENCY] servo angles measured on the student's own gripper.
 * The slides use 140/65/149 and 105/90/40; this file used 160/100 and
 * 100/80/60. */
#define GRIP_OPEN     140   //gripper open at the start   (was 160)
#define GRIP_CLOSED    75   //gripper closed on an object (was 100)
#define GRIP_RELEASE  149   //gripper opened to let go    (was 160)
#define ARM_DOWN      103   //arm down at the floor       (was 100)
#define ARM_CARRY      70   //arm lifted with the object  (was 80)
#define ARM_HIGH       50   //arm high, over the head     (was 60)

#define sp_L 6
#define F_L 12
#define B_L 11

#define sp_R 3
#define F_R 7
#define B_R 8
#define STBY 10

#define x_pin 5
#define y_pin 4
Servo servo_x;
Servo servo_y;

int sensorPin[8] = {A0,A1,A2,A3,A4,A5,A6,A7};
String detectLine = "00000000";

int sp = 50;
int maxSp = 128;
unsigned long tUpSp = 0;

void beginFnc();
String getSensor();
int getErrorInput(String L);
void followLine();
bool checkGrid();
int countGrid(int n);
void stopRobot();
void upSpeed();
void moveFor();
void turnRight90();   //turn right 90 degrees   (เลี้ยวขวา 90 องศา)
void turnLeft90();    //turn left 90 degrees    (เลี้ยวซ้าย 90 องศา)
void turnRight180();  //turn right 180 degrees  (เลี้ยวขวา 180 องศา)
void turnLeft180();   //turn left 180 degrees   (เลี้ยวซ้าย 180 องศา)
void keepup_object();
void put_object();
void arm_over_head(); //raise the arm high      (ยกแขนสูง)

void beginFnc(){
  pinMode(sp_L,OUTPUT); pinMode(F_L,OUTPUT); pinMode(B_L,OUTPUT);
  pinMode(sp_R,OUTPUT); pinMode(F_R,OUTPUT); pinMode(B_R,OUTPUT);
  pinMode(STBY,OUTPUT);
  delay(1000);

  servo_x.attach(x_pin);//attach signal pin      (เชื่อมต่อขาสัญญาณ)
  servo_y.attach(y_pin);//attach signal pin      (เชื่อมต่อขาสัญญาณ)
  servo_x.write(GRIP_OPEN);//open the gripper   (คลายแขนจับ) [was 160]
  servo_y.write(ARM_DOWN); //arm down           (ยกแขนลง)   [was 100]

  digitalWrite(STBY,1);
  clearPid();
}

String getSensor(){
  String x = "";
  for(int i=0;i<8;i++){
    if(analogRead(sensorPin[i]) >= 500){
      x += "1";
    }else{
      x += "0";
    }
  }
  return(x);
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
  else{e = 100;}          //line lost
  return(e);
}

void followLine(){
  detectLine = getSensor();
  int errorInput = getErrorInput(detectLine);
  if(errorInput != 100){
    float pidOut = pidFNC(errorInput,0,1,0,0.7);
    upSpeed();

    int spOut = map(round(pidOut),-7,7,-sp,sp);
    int speedL = sp - spOut;
    int speedR = sp + spOut;
    if(speedL>sp) {speedL = sp;}
    if(speedL<0) {speedL = 0;}
    if(speedR>sp) {speedR = sp;}
    if(speedR<0) {speedR = 0;}
    digitalWrite(F_L,1); digitalWrite(B_L,0); analogWrite(sp_L,speedL+10);
    digitalWrite(F_R,1); digitalWrite(B_R,0); analogWrite(sp_R,speedR);
  }else if(sp == 50){
    //[E33 EMERGENCY] still standing since stopRobot() (sp is back at 50)
    //and the line pattern is not in the table: drive straight on until it is
    upSpeed();
    moveFor();
  }
}

//true when at least half the sensor bar sees the line, i.e. a crossing line
bool checkGrid(){
  String ch = getSensor();
  if(ch == "00001111" || ch == "00011111" || ch == "00111111"
  || ch == "01111111" || ch == "11111111" || ch == "11111110"
  || ch == "11111100" || ch == "11111000" || ch == "11110000"){
    return (true);
  }else{
    return (false);
  }
}

int countGrid(int n){
  if(checkGrid()){
    delay(10);            //debounce
    if(checkGrid()){
      n ++;
      while(checkGrid());  //wait until the crossing has been driven over
    }
  }
  return (n);
}

void stopRobot(){
  digitalWrite(F_L,1); digitalWrite(B_L,1);
  digitalWrite(F_R,1); digitalWrite(B_R,1);
  sp = 50;                //reset to the starting speed
}

//ramp the speed up by 2 every 10 ms, capped at maxSp
void upSpeed(){
  if(millis()-tUpSp >= 10){
    sp += 2;
    tUpSp = millis();
  }
  if(sp>maxSp){sp=maxSp;}
}

void moveFor(){
  digitalWrite(F_L,1); digitalWrite(B_L,0); analogWrite(sp_L,sp+10);
  digitalWrite(F_R,1); digitalWrite(B_R,0); analogWrite(sp_R,sp);
}

//spin right until the line reaches the right-hand end of the sensor bar
//[E33 EMERGENCY] the right motor now runs BACKWARD (F_R 0, B_R 1) as on the
//slides; it was F_R 1, B_R 0 (forward), which is not a spin
void turnRight90(){
  String ch;
  while(true){
    upSpeed();
    digitalWrite(F_L,1); digitalWrite(B_L,0); analogWrite(sp_L,sp+10);
    digitalWrite(F_R,0); digitalWrite(B_R,1); analogWrite(sp_R,sp);
    ch = getSensor();
    if(ch == "00000011" || ch == "00000111" || ch == "00000110"){
      break;
    }
  }
}

//spin left until the line reaches the left-hand end of the sensor bar
void turnLeft90(){
  String ch;
  while(true){
    upSpeed();
    digitalWrite(F_L,0); digitalWrite(B_L,1); analogWrite(sp_L,sp+10);
    digitalWrite(F_R,1); digitalWrite(B_R,0); analogWrite(sp_R,sp);
    ch = getSensor();
    if(ch == "11000000" || ch == "11100000" || ch == "01100000"){
      break;
    }
  }
}

//two right 90s back to back, waiting in between until the line is left behind
void turnRight180(){
  turnRight90();
  String ch;
  while(true){
    ch = getSensor();
    if(ch == "11000000" || ch == "10000000" || ch == "00000000"){
      break;
    }
  }
  turnRight90();
}

//two left 90s back to back, waiting in between until the line is left behind
void turnLeft180(){
  turnLeft90();
  String ch;
  while(true){
    ch = getSensor();
    if(ch == "00000011" || ch == "00000001" || ch == "00000000"){
      break;
    }
  }
  turnLeft90();
}

void keepup_object(){
  servo_x.write(GRIP_CLOSED);//grip the object    (หนีบของ)   [was 100]
  delay(500);
  servo_y.write(ARM_CARRY);//lift the object      (ยกของ)     [was 80]
  delay(500);
}

void put_object(){
  servo_y.write(ARM_DOWN);//lower the object down (วางของลง)  [was 100]
  delay(500);
  servo_x.write(GRIP_RELEASE);//open the gripper  (คลายแขนหนีบ) [was 160]
  delay(500);
}

void arm_over_head(){
  servo_y.write(ARM_HIGH);//raise the arm high    (ยกแขนสูง)  [was 60]
  delay(500);
}