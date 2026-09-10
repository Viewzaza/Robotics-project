/* controlLibrary.h
 * ถอดมาจากสไลด์ robot11 (สรุปวงจรและโค้ด) หน้า 3 - 20
 */

#include <Servo.h>
#include "pidLibrary.h"

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
int maxSp = 255;
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
void turnRight90();   //เลี้ยวขวา 90 องศา
void turnLeft90();    //เลี้ยวซ้าย 90 องศา
void turnRight180();  //เลี้ยวขวา 180 องศา
void turnLeft180();   //เลี้ยวซ้าย 180 องศา
void keepup_object();
void put_object();
void arm_over_head(); //ยกแขนสูง

void beginFnc(){
  pinMode(sp_L,OUTPUT); pinMode(F_L,OUTPUT); pinMode(B_L,OUTPUT);
  pinMode(sp_R,OUTPUT); pinMode(F_R,OUTPUT); pinMode(B_R,OUTPUT);
  pinMode(STBY,OUTPUT);
  delay(1000);

  servo_x.attach(x_pin);//เชื่อมต่อขาสัญญาณ
  servo_y.attach(y_pin);//เชื่อมต่อขาสัญญาณ
  servo_x.write(140);//คลายแขนจับ
  servo_y.write(105);//ยกแขนลง

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
  else{e = 100;}
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
    digitalWrite(F_L,1); digitalWrite(B_L,0); analogWrite(sp_L,speedL);
    digitalWrite(F_R,1); digitalWrite(B_R,0); analogWrite(sp_R,speedR);
  }
}

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
    delay(10);
    if(checkGrid()){
      n ++;
      while(checkGrid());
    }
  }
  return (n);
}

void stopRobot(){
  digitalWrite(F_L,1); digitalWrite(B_L,1);
  digitalWrite(F_R,1); digitalWrite(B_R,1);
  sp = 50;
}

void upSpeed(){
  if(millis()-tUpSp >= 10){
    sp += 2;
    tUpSp = millis();
  }
  if(sp>maxSp){sp=maxSp;}
}

void moveFor(){
  digitalWrite(F_L,1); digitalWrite(B_L,0); analogWrite(sp_L,sp);
  digitalWrite(F_R,1); digitalWrite(B_R,0); analogWrite(sp_R,sp);
}

void turnRight90(){
  String ch;
  while(true){
    upSpeed();
    digitalWrite(F_L,1); digitalWrite(B_L,0); analogWrite(sp_L,sp);
    digitalWrite(F_R,0); digitalWrite(B_R,1); analogWrite(sp_R,sp);
    ch = getSensor();
    if(ch == "00000011" || ch == "00000111" || ch == "00000110"){
      break;
    }
  }
}

void turnLeft90(){
  String ch;
  while(true){
    upSpeed();
    digitalWrite(F_L,0); digitalWrite(B_L,1); analogWrite(sp_L,sp);
    digitalWrite(F_R,1); digitalWrite(B_R,0); analogWrite(sp_R,sp);
    ch = getSensor();
    if(ch == "11000000" || ch == "11100000" || ch == "01100000"){
      break;
    }
  }
}

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
  servo_x.write(65);//หนีบของ
  delay(500);
  servo_y.write(90);//ยกของ
  delay(500);
}

void put_object(){
  servo_y.write(105);//วางของลง
  delay(500);
  servo_x.write(149);//คลายแขนหนีบ
  delay(500);
}

void arm_over_head(){
  servo_y.write(40);
  delay(500);
}
