/* pidLibrary.h
 *
 * The course PID library, unchanged except for two guards marked [E33].
 * The gains are still whatever the caller passes: followLine() calls
 * pidFNC(errorInput,0,1,0,0.7) exactly as robot06 p10-p12 teaches.
 * Neither guard changes kp, ki or kd, and at normal loop speed neither
 * changes the wheel commands.
 */
#ifndef PIDLIBRARY_H
#define PIDLIBRARY_H

float integrateError = 0;
unsigned long lastTime = 0;
float lastError =0;
float pidFNC(float input,float setPoint,float kp,float ki,float kd);
void clearPid();

float pidFNC(float input,float setPoint,float kp,float ki,float kd){
  float error = setPoint - input;
  unsigned long t = millis();
  float dt = (float)(t - lastTime)/1000.0;

  /* [E33] guard 1: clearPid() sets lastTime to now, so the next call can land
   * in the same millisecond. dt is then 0 and the line below divides by zero,
   * giving inf or NaN, and round(NaN) in followLine() is undefined. */
  if(dt < 0.001) dt = 0.001;

  integrateError += error*dt;
  if(integrateError >= 100) integrateError = 100;
  if(integrateError <= -100) integrateError = -100;

  float dr = (error-lastError)/dt;

  float out = kp*error+ ki*integrateError + kd*dr;

  /* [E33] guard 2: followLine() passes this to map(out,-7,7,-sp,sp). At
   * dt = 1 ms the D term can be several thousand, map() does not clamp, and
   * the result overflows an int and steers the wrong way. The slides' own
   * clamps would turn anything past +-7 into "one wheel stopped" anyway, so
   * clamping here gives the same wheel commands without the overflow. */
  if(out >  7) out =  7;
  if(out < -7) out = -7;

  lastTime = t;
  lastError = error;

  return out;
}

void clearPid(){
  lastTime = millis();
  lastError = 0;
  integrateError = 0;
}

#endif
