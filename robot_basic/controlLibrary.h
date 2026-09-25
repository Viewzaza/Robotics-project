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
 * robot04 p10 shows what the teacher's own bar reads, and it is why 800 is the
 * better default:
 *       over white  (พื้นที่สีขาว):  245, 246, 245 ...
 *       over black  (พื้นที่สีดำ):   979, 978, 979 ...
 * With that much contrast either number works, but 800 sits well clear of the
 * white readings, so a bit of stray light cannot push a white patch over the
 * line. 500 is only 255 counts above white.
 *
 * On YOUR bar these numbers may be nothing like 245/979, which is the whole
 * problem with a fixed threshold - see AUTO_CALIBRATE below, which measures
 * them instead of assuming.
 *
 * 1 means the sensor is over the BLACK LINE, 0 means over the white surface
 * (robot04 p12 states this explicitly).
 *
 * SENSOR_ACTIVE_LOW is for a bar wired the other way round, where black reads
 * LOW. The teacher's bar reads black HIGH, so this is 0. If yours is inverted
 * every pattern in getErrorInput and checkGrid is inverted with it and no
 * threshold value can help - AUTO_CALIBRATE detects this for you. */
#define LINE_THRESHOLD     800
#define SENSOR_ACTIVE_LOW    0

/* ---- AUTOMATIC -----------------------------------------------------------
 * With this on, the two settings above are only fallbacks. At switch-on the
 * robot works them out for itself: it rocks left and right a few times so the
 * bar sweeps across the line, watches what every channel does, and sets a
 * threshold per channel from what it actually saw.
 *
 * It works out the polarity on its own too. The line is thin and the mat is
 * wide, so during the sweep each sensor spends most of its time over the mat.
 * Whichever side of the middle a channel sits on MOST of the time is therefore
 * the mat, and the line is the other side. No need to know whether your bar
 * reads high or low over black.
 *
 * The rocking is symmetric -- equal time each way -- so the robot ends up
 * pointing where it started, and it then squares itself back up on the line
 * before the mission begins.
 *
 * Set to 0 to go back to the fixed numbers above. */
#define AUTO_CALIBRATE       1
#define CAL_DUTY            70   /* how hard it rocks; slow is fine        */
#define CAL_MS             500   /* per quarter of the rocking sequence    */
#define CAL_MIN_SWING      120   /* below this a channel is called useless */

/* Set to 1 to print the sensor pattern while it drives, so you can watch what
 * it is actually seeing. Slows the loop down; turn it off for a real run. */
#define SHOW_SENSORS         0

int  senseThr[8];                /* per-channel threshold, filled at startup */
bool senseActiveLow = SENSOR_ACTIVE_LOW;
bool senseCalibrated = false;

void autoCalibrate();
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

/* Rock the robot on the spot so the sensor bar sweeps across the line. */
void calSpin(int dir,int duty){
  if(dir > 0){
    digitalWrite(F_L,1); digitalWrite(B_L,0); analogWrite(sp_L,duty);
    digitalWrite(F_R,0); digitalWrite(B_R,1); analogWrite(sp_R,duty);
  }else{
    digitalWrite(F_L,0); digitalWrite(B_L,1); analogWrite(sp_L,duty);
    digitalWrite(F_R,1); digitalWrite(B_R,0); analogWrite(sp_R,duty);
  }
}

void autoCalibrate(){
  int lo[8], hi[8];
  unsigned long sum[8];
  unsigned long n = 0;
  for(int i=0;i<8;i++){ lo[i]=1023; hi[i]=0; sum[i]=0; }

  /* Left, right, right, left: equal time each way, so the robot finishes
   * pointing where it started. Four passes means the bar crosses the line
   * several times even if it began sitting squarely on it. */
  int seq[4] = {-1, 1, 1, -1};
  for(int k=0;k<4;k++){
    calSpin(seq[k], CAL_DUTY);
    unsigned long t0 = millis();
    while(millis() - t0 < CAL_MS){
      for(int i=0;i<8;i++){
        int v = analogRead(sensorPin[i]);
        if(v < lo[i]) lo[i] = v;
        if(v > hi[i]) hi[i] = v;
        sum[i] += (unsigned long)v;
      }
      n++;
    }
  }
  stopRobot();

  /* Threshold halfway between the two surfaces this channel actually saw. */
  int usable = 0;
  for(int i=0;i<8;i++){
    senseThr[i] = (lo[i] + hi[i]) / 2;
    if(hi[i] - lo[i] >= CAL_MIN_SWING) usable++;
  }

  /* Polarity. The tape is thin and the mat is wide, so each sensor spent most
   * of the sweep over the mat -- which means the AVERAGE reading sits on the
   * mat's side of the middle. If the average is above the middle, the mat is
   * the bright side and the line is the dark one, so the line is ACTIVE LOW.
   * Every channel votes; the majority wins. */
  int votesLow = 0;
  for(int i=0;i<8;i++){
    if(hi[i] - lo[i] < CAL_MIN_SWING) continue;     /* a flat channel gets no vote */
    unsigned long mean = n ? (sum[i] / n) : 0;
    if((int)mean > senseThr[i]) votesLow++;
  }
  senseActiveLow = (votesLow * 2 > usable);
  senseCalibrated = (usable >= 3);

  Serial.println();
  Serial.println(F("--- auto calibration ---"));
  for(int i=0;i<8;i++){
    Serial.print(F("  A")); Serial.print(i);
    Serial.print(F("  low ")); Serial.print(lo[i]);
    Serial.print(F("  high ")); Serial.print(hi[i]);
    Serial.print(F("  swing ")); Serial.print(hi[i]-lo[i]);
    Serial.print(F("  threshold ")); Serial.print(senseThr[i]);
    if(hi[i]-lo[i] < CAL_MIN_SWING) Serial.print(F("   <-- too flat to use"));
    Serial.println();
  }
  Serial.print(F("  line reads ")); Serial.println(senseActiveLow ? F("DARKER than the mat") : F("BRIGHTER than the mat"));
  Serial.print(F("  usable channels: ")); Serial.println(usable);
  if(!senseCalibrated){
    Serial.println(F("  NOT ENOUGH CONTRAST -- falling back to the fixed threshold."));
    Serial.println(F("  Start the robot ON a line, and check the bar is 5-10 mm off the surface."));
  }
  Serial.println(F("------------------------"));
}

/* Square up on the line after calibrating, so the mission starts straight. */
void centreOnLine(){
  unsigned long t0 = millis();
  while(millis() - t0 < 2500){
    String ch = getSensor();
    if(ch.charAt(3) == '1' || ch.charAt(4) == '1') break;   /* middle sensors */
    /* turn toward whichever side can see it */
    int seen = -1;
    for(int i=0;i<8;i++){ if(ch.charAt(i) == '1'){ seen = i; break; } }
    if(seen < 0){ calSpin(1, CAL_DUTY); }                   /* nothing: just look */
    else if(seen < 3){ calSpin(-1, CAL_DUTY); }             /* line is to the left */
    else { calSpin(1, CAL_DUTY); }
  }
  stopRobot();
}

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

#if AUTO_CALIBRATE
  autoCalibrate();
  centreOnLine();
  delay(300);
#endif

  clearPid();
  tUpSp = millis();
}

String getSensor(){
  String x = "";
  for(int i=0;i<8;i++){
    int v = analogRead(sensorPin[i]);
    int thr = senseCalibrated ? senseThr[i] : LINE_THRESHOLD;
    bool onLine = senseActiveLow ? (v <= thr) : (v >= thr);
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

void followLine(){
#if SHOW_SENSORS
  showSensors();
#endif
  detectLine = getSensor();
  int errorInput = getErrorInput(detectLine);
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
