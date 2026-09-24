#pragma once
#include "Arduino.h"
// Mock Servo. Records the commanded angle so the world model can read it.
void sim_servo_write(int pin, int angle);
class Servo {
public:
  int pin = -1;
  int angle = 90;
  void attach(int p) { pin = p; }
  void detach() { pin = -1; }
  void write(int a) { angle = a; sim_servo_write(pin, a); }
  int read() const { return angle; }
};
