/* controlLibrary.h
 *
 * The course library, transcribed from the slides. robot11 pages 3-20 show this
 * file top to bottom; robot06 and robot08 are where followLine() and upSpeed()
 * come from, and robot07 is where the grid counting comes from.
 *
 * This is the teacher's code. It is deliberately NOT modernised, NOT
 * bug-fixed and NOT reorganised. If you want the reworked version, that is in
 * robot_countGride/.
 *
 * The only things below that are yours rather than the slides' are the six
 * servo angles, which you measured on your own arm, and maxSp. Both are marked.
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

/* YOURS, not the slides'. robot06 page 11 uses 255. Lower it if the robot is
 * too fast to count crossings reliably or overshoots its turns.
 * Note sp starts at 50 and climbs by 2 every 10 ms, so it reaches maxSp in
 * well under a second -- this really is the walking speed, not just a ceiling. */
int maxSp = 100;

unsigned long tUpSp = 0;

/* ---- YOUR measured servo angles ------------------------------------------
 * The slides use 140/65/149 and 105/90/40. These are the ones you measured on
 * your own arm. */
#define GRIP_OPEN     140
#define GRIP_CLOSED    75
#define GRIP_RELEASE  149
#define ARM_DOWN      103
#define ARM_CARRY      70
#define ARM_HIGH       50

/* ---- reading the line ----------------------------------------------------
 * THE SLIDES GIVE TWO DIFFERENT THRESHOLDS.
 *
 *   robot04 p11 and p12, the lesson that actually teaches sensing:
 *       if(analogRead(sensorPin[i]) >= 800)
 *   robot11 p06, the summary:
 *       if(analogRead(sensorPin[i]) >= 500)
 *
 * robot04 p10 shows what the teacher's own bar reads:
 *       over white  (พื้นที่สีขาว):  245, 246, 245 ...
 *       over black  (พื้นที่สีดำ):   979, 978, 979 ...
 * With that much contrast either number works. This file uses 500, the lower of
 * the two, ON PURPOSE: a threshold that is too HIGH is the worse failure. If no
 * sensor ever reads past it, getSensor() returns "00000000", getErrorInput()
 * returns 100, and followLine() writes nothing to the motors at all - the robot
 * just sits there looking dead. A threshold that is too low at least moves.
 *
 * On YOUR bar these numbers may be nothing like 245/979. Read your own with
 * STARTUP_REPORT below and set this from them: halfway between what a channel
 * reads over white and what it reads over the tape.
 *
 * 1 means the sensor is over the BLACK LINE, 0 means over the white surface
 * (robot04 p12 states this explicitly).
 *
 * SENSOR_ACTIVE_LOW is for a bar wired the other way round, where black reads
 * LOW. The teacher's bar reads black HIGH, so this is 0. If your raw numbers go
 * DOWN when a sensor moves onto the tape, set this to 1; otherwise every
 * pattern in getErrorInput and checkGrid is inverted and no threshold helps. */
#define LINE_THRESHOLD     500
#define SENSOR_ACTIVE_LOW    0

/* ---- why is it not moving? -----------------------------------------------
 * followLine() only writes to the motors inside `if(errorInput != 100)`, and
 * there is no else. getErrorInput() returns 100 for any pattern that is not one
 * of its fifteen. So if no sensor reads past LINE_THRESHOLD, the pattern is
 * "00000000", the error is 100, nothing is written, and THE ROBOT SITS STILL
 * with its motors braked. That is the slides' behaviour, not a fault - but it
 * looks identical to a dead robot.
 *
 * STARTUP_REPORT prints every sensor's raw value for a few seconds before the
 * mission starts, so you can read your own numbers off the serial monitor and
 * set LINE_THRESHOLD from them. It does not move the robot and does not change
 * anything: it just tells you what the sensors see.
 *
 * NO_LINE_WARN_MS then watches while it drives: if the pattern stays
 * unrecognised for this long it prints why it is not moving, rather than
 * leaving you guessing. 0 turns it off. */
#define STARTUP_REPORT       1
#define STARTUP_REPORT_MS 3000
#define NO_LINE_WARN_MS   1000

/* ---- turning ---------------------------------------------------------------
 * THE TURNS RAMP UP WHILE THEY SPIN, AND THAT IS WHY THEY GO WRONG.
 *
 * The slides' turnRight90() calls upSpeed() inside its own while(true) loop, so
 * `sp` keeps climbing for as long as the turn lasts. A 180 is two of those back
 * to back, so by the end the robot is spinning at full speed and the bar can
 * sweep straight past the pattern it was waiting for between two reads. It then
 * keeps going and stops on a LATER pattern, facing somewhere else entirely.
 *
 * At the top of C4 that is very easy to do: the bar sweeps over the TOP line
 * AND the C4 column, so a missed exit leaves the robot running west along the
 * top line. It then counts C3, C2, C1 as it goes, the count runs ahead of the
 * route, and it reaches `case 12: place_item` -- which opens the gripper. That
 * is the "it grabs it and immediately drops it" you saw.
 *
 * So the turns now spin at a FIXED slow speed instead of the ramping one, and
 * the middle of the 180 drives the motors itself rather than coasting on
 * whatever the previous 90 left behind. Lower TURN_SPEED if turns still
 * overshoot; raise it if the robot is too weak to turn at all. */
#define TURN_SPEED          70
#define TURN_TIMEOUT_MS   4000   /* give up rather than spin forever; 0 = never */

/* ---- reaching the object ---------------------------------------------------
 * The column line STOPS at the outer line, so once the bar is over that
 * crossing there is no line ahead any more, getSensor() returns "00000000" and
 * followLine() stops driving. From that point the only thing that moves the
 * robot toward the object is the nudge below. The slides use 50 ms, which on a
 * slow robot is a couple of centimetres. If it stops short of the object, raise
 * PICK_NUDGE_MS. If it shoves the object away, lower it. */
#define TURN_NUDGE_MS       50   /* robot10 p04 */
#define PICK_NUDGE_MS       50   /* robot10 p06 */
#define PLACE_NUDGE_MS      30   /* robot10 p09 */

/* ---- after an action, get clear of the crossing ----------------------------
 * countGrid() guards against counting the same crossing twice with
 * `while(checkGrid());` -- but that only works while the robot is still
 * rolling. Every action ends with stopRobot(), which BRAKES. So if a turn
 * finishes with the bar still over a crossing, the very next countGrid() counts
 * it again, and again, and the count runs away from the route. Six spare counts
 * is all it takes to jump from `case 5: keep_item` to `case 12: place_item`,
 * which opens the gripper -- the object gets picked up and dropped on the spot.
 *
 * leaveCrossing() creeps forward until the bar is clear before driving on. If
 * it cannot get clear it gives up rather than sitting there. */
#define LEAVE_CROSSING_MS  700   /* 0 to switch this off */

/* ---- watching the route ----------------------------------------------------
 * Prints the crossing count and what the sensors saw every time a case fires.
 * This is how you find out WHERE it goes wrong: if the robot grabs the object
 * and then drops it, this shows you whether the count jumped. */
#define TRACE_ROUTE          1

/* Set to 1 to print the sensor pattern while it drives, so you can watch what
 * it is actually seeing. Slows the loop down; turn it off for a real run. */
#define SHOW_SENSORS         0

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


#if STARTUP_REPORT
/* Read every channel out loud for a few seconds before the mission begins. */
void sensorReport(){
  Serial.println();
  Serial.println(F("--- sensors, before starting ---"));
  Serial.print(F("threshold is ")); Serial.println(LINE_THRESHOLD);
  Serial.println(F("slide the robot on and off the line and watch"));
  Serial.println(F("want: LOW over white, HIGH over black, threshold in between"));
  unsigned long t0 = millis();
  while(millis() - t0 < STARTUP_REPORT_MS){
    for(int i=0;i<8;i++){
      int v = analogRead(sensorPin[i]);
      if(v<100) Serial.print(F(" "));
      if(v<10)  Serial.print(F(" "));
      Serial.print(v); Serial.print(F(" "));
    }
    Serial.print(F("  -> ")); Serial.println(getSensor());
    delay(250);
  }
  Serial.println(F("--------------------------------"));
}
#endif

#if LEAVE_CROSSING_MS
/* Roll forward until the bar is off the crossing, so it is not counted twice. */
void leaveCrossing(){
  unsigned long t0 = millis();
  while(checkGrid()){
    digitalWrite(F_L,1); digitalWrite(B_L,0); analogWrite(sp_L,TURN_SPEED);
    digitalWrite(F_R,1); digitalWrite(B_R,0); analogWrite(sp_R,TURN_SPEED);
    if(millis() - t0 > LEAVE_CROSSING_MS){
      Serial.println(F("leaveCrossing: still on a crossing, carrying on anyway"));
      break;
    }
  }
  stopRobot();
}
#else
void leaveCrossing(){}
#endif

#if TRACE_ROUTE
void trace(int n, const __FlashStringHelper *what){
  Serial.print(F("n=")); Serial.print(n);
  Serial.print(F("  ")); Serial.print(what);
  Serial.print(F("  saw ")); Serial.println(getSensor());
}
#else
void trace(int, const __FlashStringHelper*){}
#endif

void beginFnc(){
  pinMode(sp_L,OUTPUT); pinMode(F_L,OUTPUT); pinMode(B_L,OUTPUT);
  pinMode(sp_R,OUTPUT); pinMode(F_R,OUTPUT); pinMode(B_R,OUTPUT);
  pinMode(STBY,OUTPUT);
  delay(1000);

  servo_x.attach(x_pin);//เชื่อมต่อขาสัญญาณ
  servo_y.attach(y_pin);//เชื่อมต่อขาสัญญาณ
  servo_x.write(GRIP_OPEN);//คลายแขนจับ
  servo_y.write(ARM_DOWN);//ยกแขนลง

  digitalWrite(STBY,1);

#if STARTUP_REPORT
  sensorReport();
#endif

  clearPid();
  tUpSp = millis();
}

String getSensor(){
  String x = "";
  for(int i=0;i<8;i++){
    int v = analogRead(sensorPin[i]);
#if SENSOR_ACTIVE_LOW
    bool onLine = (v <= LINE_THRESHOLD);
#else
    bool onLine = (v >= LINE_THRESHOLD);
#endif
    if(onLine){
      x += "1";
    }else{
      x += "0";
    }
  }
  return(x);
}

#if SHOW_SENSORS
/* Prints the pattern and the raw values, no more than 5 times a second so it
 * does not swamp the control loop. */
unsigned long tShow = 0;
void showSensors(){
  if(millis() - tShow < 200) return;
  tShow = millis();
  Serial.print(getSensor());
  Serial.print("  ");
  for(int i=0;i<8;i++){ Serial.print(analogRead(sensorPin[i])); Serial.print(' '); }
  Serial.println();
}
#endif

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

unsigned long tNoLine = 0;

void followLine(){
#if SHOW_SENSORS
  showSensors();
#endif
  detectLine = getSensor();
  int errorInput = getErrorInput(detectLine);

#if NO_LINE_WARN_MS
  /* Say so, rather than just sitting there looking broken. */
  if(errorInput == 100){
    if(tNoLine == 0) tNoLine = millis();
    else if(millis() - tNoLine > NO_LINE_WARN_MS){
      tNoLine = millis();
      Serial.print(F("no line: pattern ")); Serial.print(detectLine);
      Serial.print(F("  raw "));
      for(int i=0;i<8;i++){ Serial.print(analogRead(sensorPin[i])); Serial.print(F(" ")); }
      Serial.print(F(" vs threshold ")); Serial.println(LINE_THRESHOLD);
    }
  }else{
    tNoLine = 0;
  }
#endif

  if(errorInput != 100){
    float pidOut = pidFNC(errorInput,0,1,0,0.7);
    upSpeed();

    int spOut = map(round(pidOut),-7,7,-sp,sp);
    int speedL = sp - spOut;
    int speedR = sp + spOut;
    if(speedL>sp){speedL=sp;}
    if(speedL<0){speedL=0;}
    if(speedR>sp){speedR=sp;}
    if(speedR<0){speedR=0;}
    digitalWrite(F_L,1); digitalWrite(B_L,0); analogWrite(sp_L,speedL);
    digitalWrite(F_R,1); digitalWrite(B_R,0); analogWrite(sp_R,speedR);
    /* robot11 p08 ends followLine() with this line live. robot07 p05 shows it
     * commented out and robot08 p04 leaves it out entirely. Kept commented,
     * like robot07: at 9600 baud a line like this takes about 15 ms to send
     * once the serial buffer fills, which is far longer than the control loop,
     * so leaving it live makes the robot follow the line visibly worse.
     * Uncomment it if you want to watch the wheel speeds. */
    //Serial.println(String(speedL) + "," + String(speedR));
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
  unsigned long t0 = millis();
  while(true){
    digitalWrite(F_L,1); digitalWrite(B_L,0); analogWrite(sp_L,TURN_SPEED);
    digitalWrite(F_R,0); digitalWrite(B_R,1); analogWrite(sp_R,TURN_SPEED);
    ch = getSensor();
    if(ch == "00000011" || ch == "00000111" || ch == "00000110"){
      break;
    }
#if TURN_TIMEOUT_MS
    if(millis() - t0 > TURN_TIMEOUT_MS){
      Serial.println(F("turnRight90: gave up, never saw the line"));
      break;
    }
#endif
  }
}

void turnLeft90(){
  String ch;
  unsigned long t0 = millis();
  while(true){
    digitalWrite(F_L,0); digitalWrite(B_L,1); analogWrite(sp_L,TURN_SPEED);
    digitalWrite(F_R,1); digitalWrite(B_R,0); analogWrite(sp_R,TURN_SPEED);
    ch = getSensor();
    if(ch == "11000000" || ch == "11100000" || ch == "01100000"){
      break;
    }
#if TURN_TIMEOUT_MS
    if(millis() - t0 > TURN_TIMEOUT_MS){
      Serial.println(F("turnLeft90: gave up, never saw the line"));
      break;
    }
#endif
  }
}

void turnRight180(){
  turnRight90();
  String ch;
  unsigned long t0 = millis();
  while(true){
    /* Keep driving. The slides leave this loop with no motor command at all,
     * so it coasts on whatever the 90 left behind. */
    digitalWrite(F_L,1); digitalWrite(B_L,0); analogWrite(sp_L,TURN_SPEED);
    digitalWrite(F_R,0); digitalWrite(B_R,1); analogWrite(sp_R,TURN_SPEED);
    ch = getSensor();
    if(ch == "11000000" || ch == "10000000" || ch == "00000000"){
      break;
    }
#if TURN_TIMEOUT_MS
    if(millis() - t0 > TURN_TIMEOUT_MS) break;
#endif
  }
  turnRight90();
}

void turnLeft180(){
  turnLeft90();
  String ch;
  unsigned long t0 = millis();
  while(true){
    digitalWrite(F_L,0); digitalWrite(B_L,1); analogWrite(sp_L,TURN_SPEED);
    digitalWrite(F_R,1); digitalWrite(B_R,0); analogWrite(sp_R,TURN_SPEED);
    ch = getSensor();
    if(ch == "00000011" || ch == "00000001" || ch == "00000000"){
      break;
    }
#if TURN_TIMEOUT_MS
    if(millis() - t0 > TURN_TIMEOUT_MS) break;
#endif
  }
  turnLeft90();
}

void keepup_object(){
  servo_x.write(GRIP_CLOSED);//หนีบของ
  delay(500);
  servo_y.write(ARM_CARRY);//ยกของ
  delay(500);
}

void put_object(){
  servo_y.write(ARM_DOWN);//วางของลง
  delay(500);
  servo_x.write(GRIP_RELEASE);//คลายแขนหนีบ
  delay(500);
}

void arm_over_head(){
  servo_y.write(ARM_HIGH);
  delay(500);
}
