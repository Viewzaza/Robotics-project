# e33_calibrate: the checks and the calibration

This is one of the two programs. The other one, `../e33_mission`, is the real run, and its README is the
full manual: read "Quick start" and "Getting it running" there.

Choose the step at the top of `e33_calibrate.ino`, upload, and open the Serial Monitor (115200 baud, line
ending "New Line"). **Lift the robot off the field before you upload** (hold it, or stand it on a box with
the wheels in the air): the step starts as soon as the upload ends.

Do the steps in this order:

| order | `#define STEP` | what happens | then |
|---|---|---|---|
| 1 | `MODE_SENSOR_CHECK` | the robot never moves: white and black of every sensor, as a report | every sensor `OK`; send the report if you want |
| 2 | `MODE_GRIPPER_CHECK` | the robot never drives: move the gripper and the arm one degree at a time | set `GRIP_CLOSED` and the arm angles in `calibration.h` |
| 3 | `MODE_MOTOR_CHECK` | **wheels OFF the ground**: each wheel forward and backward, for ever (it waits until the robot is lifted) | `LEFT_REVERSED` / `RIGHT_REVERSED` if one runs the wrong way |
| 4 | `MODE_CALIBRATE` | at the mission start (wheels over C1, on MID, facing east), it drives by itself for about a minute, measures sensors, motors and turns, and saves them in EEPROM | every item `PASS` or `WEAK`, then upload `e33_mission`: it uses them by itself |
| 5 | `MODE_TURN_CHECK` | after `MODE_CALIBRATE`: the mission's turns, 24 times on one MID crossing, with a report | `all turns OK` (you can skip this step) |
| (any time) | `MODE_METER` | prints what the sensors see, live | |

The Nano keeps this program until you upload `e33_mission`: **upload `e33_mission` before a real run.**
Check the window title of the Arduino IDE (`e33_calibrate` or `e33_mission`) before you press Upload.

`calibration.h` here must be the same as in `e33_mission`. **After every change, save and double-click
`tools\copy_settings.bat`** (the `tools` folder next to this one): it copies the one you changed last over
the other. Change only one copy between two runs of it. If Windows will not run it, copy the file by hand:
copy `calibration.h` from the folder you changed into the other folder and choose "Replace".
