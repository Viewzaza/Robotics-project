float integrateError = 0;
unsigned long lastTime = 0;
float lastError =0;
float pidFNC(float input,float setPoint,float kp,float ki,float kd);
void clearPid();

float pidFNC(float input,float setPoint,float kp,float ki,float kd){
  float error = setPoint - input;
  unsigned long t = millis();
  float dt = (float)(t - lastTime)/1000.0;

  integrateError += error*dt;
  if(integrateError >= 100) integrateError = 100;
  if(integrateError <= -100) integrateError = -100;

  float dr = (error-lastError)/dt;

  float out = kp*error+ ki*integrateError + kd*dr;
  lastTime = t;
  lastError = error;

  return out;
}

void clearPid(){
  lastTime = millis();
  lastError = 0;
  integrateError = 0;
}
