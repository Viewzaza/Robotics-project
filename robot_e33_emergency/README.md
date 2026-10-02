# robot_e33_emergency

The last, emergency version: the student's own course files
(`robot_original.ino`, `controlLibrary.h`, `pidLibrary.h`) with the route put
in the way the slides do it, and nothing else that is not needed.
No calibration, no recovery, no buttons. `Serial.begin(9600)` stays.

## How to use

1. Keep the three files together in one folder called `robot_e33_emergency`.
2. Open `robot_e33_emergency.ino` in the Arduino IDE, board Arduino Nano, upload.
3. Put the robot on the field with the **wheels over C1 on the MID line**,
   facing east (toward C4), gripper open, arm down.
4. Switch on. After about 1 s (the `delay(1000)` in `beginFnc()`) it drives.

Route 312: object 3 (top of C4) to x3 (bottom of C2), object 1 (top of C1) to
x1 (bottom of C4), object 2 (bottom of C1) to x2 (bottom of C3).

## The route (the switch in loop())

Same count table as the working `robot_e33_v2` (lines 190 to 216). Every
case was also walked on the field by hand.

| case | helper | where |
|---|---|---|
| 3 | turn90 LEFT | C4 MID, turn north |
| 5 | keep_item LEFT | C4 TOP, pick object 3, turn round |
| 7 | turn90 RIGHT | C4 MID, turn west |
| 10 | turn90 LEFT | C2 MID, turn south |
| 12 | place_item LEFT | C2 BOT, object 3 on x3, turn round |
| 14 | turn90 LEFT | C2 MID, turn west |
| 16 | turn90 RIGHT | C1 MID, turn north |
| 18 | keep_item RIGHT | C1 TOP, pick object 1, turn round |
| 20 | turn90 LEFT | C1 MID, turn east |
| 24 | turn90 RIGHT | C4 MID, turn south |
| 26 | place_item RIGHT | C4 BOT, object 1 on x1, turn round |
| 28 | turn90 LEFT | C4 MID, turn west |
| 32 | turn90 LEFT | C1 MID, turn south |
| 34 | keep_item LEFT | C1 BOT, pick object 2, turn round |
| 36 | turn90 RIGHT | C1 MID, turn east |
| 39 | turn90 RIGHT | C3 MID, turn south |
| 41 | place_item STOP | C3 BOT, object 2 on x2, no turn |
| 42 (END_GRIDE) | stopRobot | done |

`turn90`, `keep_item` and `place_item` are copied from robot10 p4, p6 and p9.
The only change in them is `delay(NUDGE_TURN)`, `delay(NUDGE_PICK)` and
`delay(NUDGE_PLACE)` in place of the slides' `delay(50)`, `delay(50)` and
`delay(30)`.

## What was changed in the student's files, and why

Only three things, each marked `[E33 EMERGENCY]` in `controlLibrary.h`.
`pidLibrary.h` is not changed at all.

1. **turnRight90()**: the right motor now runs backward (`F_R 0, B_R 1`) as on
   robot08 p3 and robot11 p14. The student's copy drove it forward, so the
   robot did not spin. In the simulator that copy scores 0/3 on all three
   presets.
2. **Servo angles**: the ones measured on this gripper, as defines at the top:
   GRIP_OPEN 140 (was 160), GRIP_CLOSED 75 (was 100), GRIP_RELEASE 149
   (was 160), ARM_DOWN 103 (was 100), ARM_CARRY 70 (was 80), ARM_HIGH 50
   (was 60).
3. **followLine() when standing still**: if the robot has not moved since
   `stopRobot()` (sp is still 50) and the bar shows a pattern that
   `getErrorInput()` does not know, it now drives straight on (`upSpeed();
   moveFor();`) until it sees one it knows. Why: the slides' table has no
   entry for ONE middle sensor on the line (for example `00100000`). After the
   180 degree turn at the first drop spot the robot often stops with the line
   under one sensor; `followLine()` then did nothing, and the braked robot
   stood there for ever. Without this, at the slides' NUDGE values, the
   simulator's slow preset completed 0 of 50 runs and the fast preset 6 of 50;
   with it, 37 and 49 of 50. (Adding the six single
   sensor patterns to the table was tried too: it made the line following
   weave more and was worse, 30 of 60 against 54 of 60.)

## Every other difference from the slides (robot11 p3 to p20): kept

| function | slides | student's file | kept because |
|---|---|---|---|
| globals | `maxSp = 255` | `maxSp = 128` | 255 in the simulator: 40 of 60 full runs (fast preset 0 of 15) |
| followLine, moveFor, turnRight90, turnLeft90 | left motor at `sp` / `speedL` | `+10` on the left motor | the left motor is weaker; without +10: 41 of 60 (slow preset 1 of 15) |
| followLine | `map(pidOut, ...)` | `map(round(pidOut), ...)` | round() is what robot06 p8 to robot08 p4 teach; harmless |
| followLine | `Serial.println(speedL,speedR)` at the end | no print | at 9600 baud a print in every loop slows the robot's steering; robot08 p4 has no print either |
| beginFnc | 140 / 105 | 160 / 100 | changed to the measured angles (above) |
| keepup_object, put_object, arm_over_head | 65, 90 / 105, 149 / 40 | 100, 80 / 100, 160 / 60 | changed to the measured angles (above) |
| getSensor (threshold 500), getErrorInput, checkGrid, countGrid, stopRobot, upSpeed, turnRight180, turnLeft180 | | same as the slides | nothing to change |
| pidLibrary.h | not shown on the slides (only a download link) | | kept as it is |

The 500 threshold stays. 650 scored a little better in the simulator (190 of
200 full runs against 178 of 200) but the run works at 500, so it was not
changed. If A0 reads about 640 on white again (the loose wire seen in the
real log), **no threshold saves the run** (500, 650, 750 and 800 complete
at best 2 of 40 runs in the simulator): fix the A0 wire. To check the bar, upload once with the line
`//Serial.println(getSensor());` in `loop()` turned back on and look at the
Serial Monitor over white and over the line.

## The NUDGE values

| | slides | here |
|---|---|---|
| NUDGE_TURN | 50 | 90 |
| NUDGE_PICK | 50 | 30 |
| NUDGE_PLACE | 30 | 30 |

* NUDGE_TURN: the bar is 9.5 cm ahead of the wheels, so when a turn starts the
  wheels are still 6 to 8 cm short of the crossing, even at 90 ms. The turn
  still finds the new line (it often stops early and followLine() straightens
  the robot). Full runs out of 200 (4 presets x 5 starts x 10 seeds): 50 ms 179,
  80 ms 182, 90 ms 178, 100 ms 181, 110 ms 166. With 5 seeds (out of 100):
  30 ms 55, 130 ms 71.
  50 to 100 is one flat plateau; 90 was a little better with a tired battery
  (slow preset 42 of 50 against 37 at 50 ms) and equal with the "start lift"
  effect of the real log (`--lift=0.5`: 81 of 100).
* NUDGE_PICK / NUDGE_PLACE: 0 to 100 ms all work; 30 puts the gripper right on
  the object (jaws 29.4 to 30.7 cm from MID for an object at 30 cm) and on the
  drop spot (29.4 to 30.6 cm for a spot at 30 cm). 150 ms is too far (the gripper closes in front of the object).

## How often it completes the route (simulator)

Final version, 10 seeds each (full runs out of 10, objects out of 30).
REALM is the robot fitted to the real Serial log.

| start x (cm) | --student | --student-slow | --student-fast | REALM |
|---|---|---|---|---|
| -4 | 9 (28) | 9 (28) | 8 (28) | 9 (29) |
| 0 | 10 (30) | 7 (23) | 10 (30) | 9 (29) |
| 2 | 9 (29) | 8 (26) | 9 (29) | 9 (27) |
| 5 | 10 (30) | 9 (29) | 9 (29) | 10 (30) |
| 8 | 8 (26) | 9 (28) | 7 (27) | 10 (30) |
| all | 46/50 | 42/50 | 43/50 | 47/50 |

In all: 178 of 200 full runs, 565 of 600 objects (94%). With the start lift
effect: 81 of 100. On the emulated Nano (the real compiled program): 3/3 for
student, slow, fast and REALM from x = 0, and for student from x = -4, 2, 8;
0/3 from x = 5.

Where it fails (the slides' code has no recovery, so some failure is expected):

* **A count off by one crossing** (most failures). Either the bar sees one
  crossing twice (the crossing pattern flickers as the bar leaves it and
  `countGrid()` only waits 10 ms), or a 90 degree turn with the wheels short
  of the crossing meets the new line in the middle of the bar, does not stop,
  and ends on the wrong line. The object is then put down one column off.
* **A pick missed** (4 of 200 runs).
* **An object put down a little outside its spot** (6.3 cm and 8.8 cm, the
  simulator allows 6 cm), 2 of 200 runs.
* **A Nano restart** (the 9 V battery dips when a servo moves): the program
  starts again from the beginning, opens the gripper (drops the object) and
  counts from 0, so the run is lost. Use a fresh battery, or USB power, and
  if it restarts, put the robot back at the start.

## Nano build

Flash 9990 bytes of 30720 (the original: 8080), RAM 522 of 2048 bytes,
no warnings (`-Wall -Wextra`, the same as the original).
