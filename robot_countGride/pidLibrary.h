/* pidLibrary.h
 *
 * The PID helper supplied with the course, with three guards added. The
 * algorithm and the pidFNC/clearPid API are unchanged, so it still matches the
 * robot06 slides.
 */
#ifndef PIDLIBRARY_H
#define PIDLIBRARY_H

float integrateError = 0;
unsigned long lastTime = 0;
float lastError = 0;
bool  pidFirst = true;

float pidFNC(float input, float setPoint, float kp, float ki, float kd);
void clearPid();

float pidFNC(float input, float setPoint, float kp, float ki, float kd) {
  float error = setPoint - input;
  unsigned long t = millis();
  float dt = (float)(t - lastTime) / 1000.0f;

  /* GUARD 1: the control loop runs faster than 1 kHz whenever the sensor read
   * is short, so two calls can land in the same millisecond and dt becomes 0.
   * The derivative term then divides by zero, producing inf/NaN, which
   * propagates into the motor duty and makes the robot lurch off the line. */
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.25f)  dt = 0.25f;   /* after a long blocking action, do not let
                                  * one huge dt dump into the integral */

  integrateError += error * dt;
  if (integrateError >= 100) integrateError = 100;
  if (integrateError <= -100) integrateError = -100;

  /* GUARD 2: on the very first call after clearPid(), lastError is stale, so
   * the derivative sees a step change that never physically happened. */
  float dr = pidFirst ? 0.0f : (error - lastError) / dt;
  pidFirst = false;

  float out = kp * error + ki * integrateError + kd * dr;

  /* GUARD 3: the caller feeds this to map(out, -7, 7, -sp, sp), and Arduino's
   * map() does NOT clamp -- an out of +40 becomes a duty far past 255, which
   * wraps when cast. Clamp at the source. */
  if (out > 7.0f) out = 7.0f;
  if (out < -7.0f) out = -7.0f;

  lastTime = t;
  lastError = error;
  return out;
}

void clearPid() {
  lastTime = millis();
  lastError = 0;
  integrateError = 0;
  pidFirst = true;
}

#endif
