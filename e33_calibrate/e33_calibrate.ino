/* =====================================================================
 *  e33_calibrate.ino: the checks and the calibration for e33_mission
 * =====================================================================
 *
 *  Choose the STEP below, upload, and open the Serial Monitor at 115200
 *  baud, line ending "New Line". Lift the robot off the field BEFORE you
 *  upload: the step starts as soon as the upload ends. The order:
 *  SENSOR_CHECK, GRIPPER_CHECK, MOTOR_CHECK, CALIBRATE, TURN_CHECK.
 *
 *    MODE_SENSOR_CHECK   the robot never moves. White and black of every
 *                        sensor, as a report to copy and send. It ends
 *                        with the sensor part of the CAL VALUES (pasting
 *                        it into calibration.h is not needed:
 *                        MODE_CALIBRATE measures the levels again).
 *    MODE_GRIPPER_CHECK  the robot never drives: move the gripper and the
 *                        arm one degree at a time to find your angles.
 *    MODE_MOTOR_CHECK    WHEELS OFF THE GROUND: each wheel in turn (it
 *                        waits until the robot is lifted).
 *    MODE_CALIBRATE      put the robot at the mission start (wheels over
 *                        C1, on MID, facing east) and step back: it drives
 *                        by itself for about a minute, measures the sensors,
 *                        the motors and the turns, and saves them in
 *                        EEPROM. e33_mission then uses them by itself.
 *    MODE_TURN_CHECK     after MODE_CALIBRATE: the mission's turns, 24
 *                        times on one MID crossing, with a report.
 *    MODE_METER          (any time) prints what the sensors see, live.
 *
 *  The mission is the other program, e33_mission: it never calibrates.
 *  The Nano keeps this program until you upload e33_mission.
 *  calibration.h here must be the same as in e33_mission: after every
 *  change, double-click tools\copy_settings.bat.
 * ===================================================================== */
#ifndef ROBOT_MODE                  /* (the simulator can set it from outside) */
#define STEP   MODE_SENSOR_CHECK    /* <- choose the step here */
#define ROBOT_MODE STEP
#endif

#include "calibration.h"
#if !(ROBOT_MODE >= MODE_CALIBRATE && ROBOT_MODE <= MODE_TURN_CHECK)
  #error "STEP must be one of the steps listed above (MODE_CALIBRATE, MODE_SENSOR_CHECK, ...). The mission is the other program, e33_mission."
#endif
#include "controlLibrary.h"
#include "calibrationMode.h"
#include "checkModes.h"

/* (the library's crossing counter asks the route what each count is: the
 * checks and the calibration have no route) */
int numGride = 0;
uint8_t caseInfo(int n){ (void)n; return K_PASS; }

void setup() {
  Serial.begin(SERIAL_BAUD);
  beginFnc();
#if ROBOT_MODE == MODE_CALIBRATE
  runCalibration();           /* never comes back */
#elif ROBOT_MODE == MODE_METER
  runMeter();
#elif ROBOT_MODE == MODE_MOTOR_CHECK
  runMotorCheck();
#elif ROBOT_MODE == MODE_SENSOR_CHECK
  runSensorCheck();
#elif ROBOT_MODE == MODE_GRIPPER_CHECK
  runGripperCheck();
#elif ROBOT_MODE == MODE_TURN_CHECK
  runTurnCheck();
#endif
}

void loop() {
}
