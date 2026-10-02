# e33: the e33 mission, built on the course slides

Two programs (two Arduino sketches):

| program | what it does | when |
|---|---|---|
| `e33_calibrate/` | the checks and the calibration: choose the `STEP` at the top of `e33_calibrate.ino` | before the mission, and again when something changed |
| `e33_mission/` | the real run: nothing else, and it never calibrates | for every run |

`calibration.h` (your robot and field numbers) is in both folders, and both programs must use the same
numbers. **Edit either copy, then double-click `tools\copy_settings.bat`** (the `tools` folder next to
`e33_mission`): it copies the one you changed last over the other. (Change only one copy between two runs
of it.)

`calibration.h` already holds the sensor levels you measured (white about 70 to 200, black about 965 to
975), so `e33_mission` sees your line without a calibration, and it goes as soon as the line has been under
the middle of the bar for 0.2 s. The motors and turns still use the starting guesses until you run
`e33_calibrate` with `STEP MODE_CALIBRATE` once. Do the steps below in this order.

## Quick start: the steps in order

Each step is explained in "Getting it running" below. **After every change to `calibration.h`, save it and
double-click `tools\copy_settings.bat`.**

| # | what | the robot | the result you want |
|---|---|---|---|
| 0 | Arduino IDE: Tools > Board **Arduino Nano**, Tools > Port (the COM port that appears when you plug in the Nano), Tools > Processor **ATmega328P** (if the upload ends with `not in sync`, choose **ATmega328P (Old Bootloader)**) | USB in | the upload ends with no red error |
| 1 | a ruler: the sizes in `calibration.h` | | |
| 2 | `e33_calibrate`, `STEP MODE_SENSOR_CHECK` | never moves | every sensor `OK` |
| 3 | `e33_calibrate`, `STEP MODE_GRIPPER_CHECK` | never drives | `GRIP_CLOSED` and the arm angles in `calibration.h` |
| 4 | `e33_calibrate`, `STEP MODE_MOTOR_CHECK` | **wheels in the air** | each wheel turns the right way |
| 5 | `e33_calibrate`, `STEP MODE_CALIBRATE` | at the start, drives by itself for about a minute | every item `PASS` or `WEAK` |
| 6 | `e33_calibrate`, `STEP MODE_TURN_CHECK` (you can skip it) | wheels on C2 MID | `all turns OK` |
| 7 | **`e33_mission`** | at the start | the real run |

Each program is a folder, and the Arduino IDE opens each one in its own window. **Look at the window
title (`e33_mission` or `e33_calibrate`) before you press Upload.** The Nano keeps the program you uploaded
last: after the checks it still has `e33_calibrate` until you upload `e33_mission`.
Unzip the download first (right-click, "Extract All"), and open the sketches from the unzipped folder,
never from inside the ZIP.

The first version is kept unchanged in `../robot_e33/` (and in git; the version before this one, one
sketch with a mode switch, is `robot_e33_v2` in git). This version has:
- **MODE_SENSOR_CHECK**: white and black of every sensor, printed as a report you can copy and send;
- **a run log**: every run is recorded in EEPROM and printed at the next start, so a run without the
  USB cable can be read afterwards;
- **MODE_GRIPPER_CHECK**: move the gripper and the arm one degree at a time from the Serial Monitor to
  find your angles;
- **MODE_TURN_CHECK**: the mission's own turns, 24 times on one crossing, each timed, with a report;
- **two route orders** to choose from (objects 3, 1, 2 or 1, 3, 2) and a **practice** setting that
  stops after the first object, or the first two (see "Two routes, and practice");
- **a log reader**: `../tools/e33_log_reader.html` explains what the Serial Monitor printed;
- **the supply voltage**: the Nano measures its own 5 V at the start and during every pick and place,
  keeps it in the run log and warns about a weak battery before it causes a brown-out (see "The battery").

## Why it did not start on your field (fixed)

Your printout with the line under the middle showed `bar 11100111`: the six WHITE sensors lit and the two on
the line (876, 898) dark, so the robot thought it stood on a crossing and waited for ever. It had swapped
black and white. With no calibration, the robot measured the levels itself while standing still, and at
one moment the bar was partly on a black area (5 sensors on black, 3 on white): the old code took those 3
white sensors for a white line on a black field ("black reads LOW"). Now:
- which way black reads is a setting, `CAL_LINE_LOW` (0: black reads HIGH, as on your bar), never guessed;
- the standing-still measurement only accepts a narrow BLACK line near the middle of the bar;
- a saved calibration with the other polarity is not used for the sensors;
- your measured levels are in `calibration.h` (`CAL_SENSORS_MEASURED 1`), so the mission does not need
  the standing-still measurement at all.
In the simulator, with your eight white and black readings: put down with a black area under 5 sensors,
then moved onto the line, the old code shows exactly your `11100111` and never starts; now it starts and
places all 3 objects (both routes, ideal and realistic motors).

This is the firmware for the e33 worksheet (3 objects, 3 targets, 4 columns x 3 lines).
It keeps the slides' structure (robot04 to robot11): `beginFnc()`, `followLine()` with
`pidFNC(errorInput,0,1,0,0.7)`, `countGrid()`, the `switch(numGride)` from robot10, and the
helpers `turn90()`, `keep_item()` and `place_item()`. Every line that differs from the slides
is marked `[E33]` in the code, with the reason.

## The files

| file | what it is | edit it? |
|---|---|---|
| `calibration.h` | your robot and field numbers, the CAL VALUES (in both folders: `tools\copy_settings.bat` copies the one you changed last over the other) | **yes** |
| `e33_mission/e33_mission.ino` | the mission: the route (robot10 style) and the three helpers | no |
| `e33_calibrate/e33_calibrate.ino` | the checks and the calibration: choose the `STEP` at its top | only `STEP` |
| `controlLibrary.h` | the slides' library, with the [E33] changes | no |
| `pidLibrary.h` | the slides' PID, gains unchanged, two safety guards | no |
| `calibrationMode.h` | automatic calibration, sensor meter, motor check (in `e33_calibrate`) | no |
| `checkModes.h` | sensor check, gripper check and turn check (in `e33_calibrate`) | no |
| `runLog.h` | the record of the last run, kept in EEPROM | no |
| `../tools/e33_log_reader.html` | a page that reads what the Serial Monitor printed (open it in Chrome or Edge) | no |

Open the Serial Monitor at **115200** baud to see what the robot is doing (the Arduino IDE's default is
9600: at 9600 you only see strange characters). The first two lines at every start are
`reset cause: ...` (`unknown (the bootloader cleared it)` is normal on many Nanos) and `CAL: ...`
(which calibration is in use). Before your first `MODE_CALIBRATE` it must say
`CAL: sensor levels measured (calibration.h) ...`. If it already says `CAL: EEPROM run #...`, an older
program saved a calibration on this Nano: `MODE_CALIBRATE` replaces it.

To copy from the Serial Monitor: switch off "Toggle Autoscroll" (a button at the top right of the Serial Monitor), select the lines
with the mouse, then Ctrl+C. Leave "Toggle Timestamp" (the clock button) off.

## Sending me data (copy the block between BEGIN and END)

Open the Serial Monitor at **115200** baud, line ending **New Line** (the box next to the baud rate).
If Enter does nothing, that box says "No Line Ending": choose New Line (or wait: every step also goes on by
itself).

1. **Sensor check** (`e33_calibrate`, `STEP MODE_SENSOR_CHECK`). The robot never moves. Four steps, each started with
   Enter (or it goes on by itself after 20 s):
   1. the whole bar over **white**;
   2. with the bar over a line, turn the robot slowly **left and right** by hand, wheels on the table
      (a lifted bar reads black), so the line passes under **every** sensor
      (it prints which sensors have seen the line so far);
   3. the line under the **middle** of the bar;
   4. the bar lying **along a straight line**, so **all 8** sensors are on black.

   It prints `=== E33 SENSOR REPORT BEGIN ===` ... `END`: white, noise, black and contrast of every
   sensor, whether the levels in use now see the line, one threshold that would work for all 8 (like the
   slides' 800 / 500), and the sensor part of the CAL VALUES.
2. **Calibration report** (`e33_calibrate`, `STEP MODE_CALIBRATE`, after a calibration run): plug in USB and open the Serial
   Monitor; it prints `=== CALIBRATION REPORT` and the `#define CAL_...` block. Copy from
   `=== CALIBRATION REPORT` down to `end CAL VALUES` (with the Serial Monitor's timestamps switched off).
   This copy, printed again after the restart, has every result but not the lists measured on the way
   (`left/right effort in each cell`, the single turn times, `cells measured`): those are printed only at
   the end of the run itself, with the USB cable in.
3. **Run log** (`e33_mission`): after a run (with or without the cable), pick the robot up off the lines,
   plug in USB and open the Serial Monitor. It prints `=== E33 RUN LOG BEGIN ===` ... `END`: every crossing (distance measured and
   expected, sensors, line ahead), every turn (time, lines passed), each pick and place, a fault if there
   was one, and how the run ended (normally, a fault, the power switched off, or the Nano restarting in
   the middle: a brown-out). `START cal=1 run#1` says which calibration was in use (0 the defaults, 1 the
   EEPROM calibration, 2 values pasted into `calibration.h`); `light=` is how far the white under the bar
   at the start was from the calibrated white (near 0 is good; see "Competition day"). `DONE ... speed=x0.93`
   is the speed the run measured against the calibration (the battery). The `POWER` lines are the supply
   voltage at the start, at each pick and place, and the lowest of the run (see "The battery"). The log is kept until the next run has really moved (its first turn), so
   switching on to read it does not wipe it. After a run that did not finish, the next start waits 5 s
   (instead of 0.2 s) before driving: lift the robot, or switch the motors off, if you only want to read
   the log.
4. **Gripper check** (`e33_calibrate`, `STEP MODE_GRIPPER_CHECK`): type `o` `c` `r` (open, closed, release), `+` `-` (gripper one
   degree tighter / looser), `d` `y` `h` (arm down, carry, high), `u` `j` (arm one degree up / down),
   `t` (a test pick). To find `GRIP_CLOSED`: object in the jaws, a strip of paper between one jaw and the
   object, press `+` until the strip is held. That angle is where the jaws touch. After closing, the
   mission opens `GRIP_BACKOFF_DEG` (3) again to hold, so set `GRIP_CLOSED` `GRIP_BACKOFF_DEG` + 2 = 5 degrees
   tighter, and check with `t` that the object does not slip. The servo must not buzz while holding.
   Every line also prints the sensor bar (`bar A0..A7:`). With the robot on white, compare the numbers
   with the arm down, carrying and high, with the object in the jaws: they must stay near the white
   numbers. If one moves towards black, the arm or the object is in front of the sensors (or shines their
   light back) and the robot can see a line that is not there.
5. **Turn check** (`e33_calibrate`, `STEP MODE_TURN_CHECK`, after `MODE_CALIBRATE`: it uses the saved calibration like the
   mission). Put the **wheels (axle) exactly on the C2 MID or C3 MID crossing** (four arms), facing along
   MID, so the line is under the middle of the bar. Switch on, open the Serial Monitor, press Enter (or
   wait 30 s) and step back. Once the line is steady under the bar it turns on the spot with the
   mission's own turns: 4 x left 90, 4 x right 90, 2 x left 180, 2 x right 180, and all of it once more
   (24 turns, about a minute). It ends facing the way it started. After every turn one line: the time
   against what the mission expects, the lines passed, the bar and the error afterwards, and `OK`,
   `SLOW`, `FAST`, `EXTRA LINE` or `OFF LINE`. Then `=== E33 TURN CHECK BEGIN ===` ... `END`: for each
   turn the mean time, shortest and longest, failures, how far the axle was off the crossing (the 90s
   alternate short and long when it is), and a verdict in plain words. If a turn fails, the robot stops,
   prints the report so far and blinks the fault number. Send the turn lines and the report. Press Enter
   to run it again (wheels back on the crossing first).

**Reading it yourself.** Double-click `tools/e33_log_reader.html` (in the repository, next to this folder; it
works offline in Chrome or Edge), paste everything the Serial Monitor printed and press **Read it**. It
reads the run log, the live trace, the sensor report, the calibration report, the gripper check and the
turn check. It
says how the run ended, what a fault number means and what to look at. It compares every measured
distance and turn time with the expected one, MID legs and column legs separately, so a wrong `CELL_CM`
or `ROW_CM` shows up as a number of cm. It also lists any line it did not recognise. Serial Monitor
timestamps are fine. **Copy summary** copies a short text version: send it together with the full paste.

## Getting it running (about 20 minutes)

For every check and for the calibration: open `e33_calibrate`, set the `STEP` line at the top of
`e33_calibrate.ino`, for example `#define STEP   MODE_SENSOR_CHECK`, upload, and open the Serial Monitor
(115200 baud, New Line). The program starts as soon as the upload ends, so **lift the robot off the field
before you upload** (hold it, or stand it on a box with the wheels in the air): on the field the mission
drives off 0.2 s after it sees the line, and the calibration 3 s after it sees it, with the USB cable still
in (the motor check only turns the wheels once the robot is lifted). Switching only the motors off is not enough: `e33_mission` then starts, cannot move, and stops
with FAULT 12 (12 blinks); switch off and on to clear it. (The run log of the last run is kept: a
FAULT 12 before the first crossing does not replace it.)

1. **Measure with a ruler** and put the numbers in `calibration.h`:
   `CELL_CM` (C1 to C4 along MID, divided by 3), `ROW_CM` (MID to TOP), `TAPE_W_CM`,
   `GRIP_REACH_CM` (wheel axle to the centre of an object held in the gripper),
   `OBJ_BEYOND_CM` and `TGT_BEYOND_CM` (outer line centre to object / target centre).
   `SENSOR_AHEAD_CM` is your 9.5 cm. Save, then double-click `tools\copy_settings.bat`.
   How exact each one must be, and the order to measure them: "How exact must the ruler numbers be".
2. **`STEP MODE_SENSOR_CHECK`** (in `e33_calibrate.ino`, upload). The robot never moves. Follow the
   four steps; every sensor should say `OK`, and the report should say `step 2: every sensor saw the line: yes`.
   `sees_line_now` tells whether the levels in use now see the line. The levels in `calibration.h` are
   the ones you measured, so every sensor should already say `yes` (with `CAL_SENSORS_MEASURED 0`, guesses,
   `NO` is normal before step 5). After step 5 every sensor must say `yes`.
   `WEAK` or `DEAD`: check the bar height and that sensor's wire. `NOISY`: check the wiring, keep sunlight off.
   To watch the sensors while you move the robot by hand, use `STEP MODE_METER`: it prints 10 lines a second,
   and every 3 s the lowest and highest each sensor has read, so a line that passes quickly still shows. (The
   mission's waiting screen prints only once a second: a line slid past between two prints is not seen.)
   (`MODE_METER` prints the raw values live.) Pasting the report's 4 sensor lines into `calibration.h` is
   not needed (the calibration measures the levels again). If you do: select the old `CAL_LINE_LOW`,
   `CAL_LO`, `CAL_HI` and `CAL_DEAD_MASK` lines first, so the paste replaces them (pasted above the old
   lines, the upload stops with `CAL VALUES twice in calibration.h`: delete the old lines), then run
   `copy_settings.bat`.
3. **`STEP MODE_GRIPPER_CHECK`**: find the gripper angle. Close the gripper slowly on a real object and note
   the angle where the jaws just touch it. `GRIP_CLOSED` should be `GRIP_BACKOFF_DEG` + 2 = 5 degrees
   tighter than that (after closing, the mission opens `GRIP_BACKOFF_DEG` again, so it holds 2 degrees past
   the touch), not the "empty and fully closed" angle: each printed line gives the number to write. A servo
   squeezing much tighter than the object pulls a lot of current from the 9 V battery. Put the angles in
   `calibration.h`, then `copy_settings.bat`.
4. **`STEP MODE_MOTOR_CHECK`** with the **wheels off the ground**. Each wheel runs forward and backward in turn.
   It only runs while the robot is lifted: then nothing reflects the sensors' light and they read black. On the
   floor it waits (`waiting: lift the robot`) instead of driving off.
   LEFT and RIGHT are the robot's own left and right, seen from behind it, looking the way it drives.
   If one runs the wrong way, set `LEFT_REVERSED` or `RIGHT_REVERSED` to 1 (then `copy_settings.bat`). If the
   OTHER wheel turns (the right one when it says LEFT), swap the two motor plugs. If a wheel never turns:
   motor switch, battery, the STBY wire (D10). The duty where each wheel starts to turn is only for you to
   see: nothing to write, the calibration measures it. It goes on for ever: upload the next step when you
   have seen it.
5. **`STEP MODE_CALIBRATE`** with a **fresh battery**. Put the robot at the start (wheels over C1, on MID,
   facing east), switch on and step back. 3 s after it sees the line (LED on) it calibrates itself for
   about a minute and stops back at the start. The LED stays on with short gaps when it finished, or blinks
   fast if it stopped with a problem.
   If a calibration stops part way, what the last good calibration measured is kept (a stopped run is not
   trusted), so a bad run does not wipe a good one (when both were made at the same `MAX_SP`, `MOVE_SP` and
   `TURN_SP`). A finished run replaces what it measured well. If the report says `FAST ROBOT`, your motors are strong (above about
   30 cm/s the robot loses the line): lower `MAX_SP`, `MOVE_SP`, `TURN_SP`, `TURN_SLOW_SP` and `SP_START` to the
   percentage it gives, run `copy_settings.bat`, and calibrate again.
   Then plug in USB and open the Serial Monitor: the Nano restarts and prints the saved report and the
   CAL VALUES block again. If the first line says `STOPPED during ...`, see "When the calibration stops"
   below. To calibrate again, lift the robot and put it down at the start. (If this Nano was calibrated
   before, also with `robot_e33` or `robot_e33_v2`, switching on at the start does nothing, LED on 1 s / off 1 s, until you
   lift it and put it down.) A calibration that STOPPED is saved too: the mission then uses the items the
   last good calibration had, the items the stopped run finished where there were none, and the guesses in
   `calibration.h` for the rest. After a STOPPED report, fix the cause and calibrate again until it finishes.
   Pasting the `#define CAL_...` block into `calibration.h` is not needed: the mission reads the calibration
   from EEPROM by itself. (To freeze it: select the whole CAL VALUES block of `calibration.h`, from
   `#define CAL_VALUES_FROM_RUN` down to `end CAL VALUES`, paste the printed block over it, set
   `CAL_USE_EEPROM` to 0, and run `copy_settings.bat`. From then on a new calibration is **ignored** until
   you set `CAL_USE_EEPROM` back to 1.)
6. **`STEP MODE_TURN_CHECK`** (you can skip it): see "Sending me data" above, point 5. It uses the same
   calibration as the mission.
7. **Upload `e33_mission`**. Put the robot at the start and switch on. Put the wheels exactly over C1 (the
   simulator's limits are in "Two routes, and practice": the 1-3-2 route needs it more exactly). It waits,
   standing still, until the line has been under the middle of the bar for 0.2 s (`START_WAIT_MS`), then
   goes. It only starts with the bar on a plain line (not on a crossing, and never while held in the air,
   where every sensor reads black). No buttons. It never makes a calibration movement.
   It uses, in this order: what `STEP MODE_CALIBRATE` saved in EEPROM, else the CAL VALUES in
   `calibration.h`. The sensor levels there are the ones you measured (`CAL_SENSORS_MEASURED 1`). With
   `CAL_SENSORS_MEASURED 0` (guesses) and guesses that cannot see the line, it measures the levels from a
   black line under the middle of the bar while standing still, and says so. It never swaps black and
   white: `CAL_LINE_LOW` says which way black reads.
   After `MODE_CALIBRATE`, the `CAL:` line in the Serial Monitor must say `EEPROM run #1` (or the number of
   your last calibration) with every item `PASS` or `WEAK`.

## Two routes, and practice

`ROUTE_ORDER` in `calibration.h` chooses the order of the objects:

| | `312` (the default): objects 3, 1, 2 | `132`: objects 1, 3, 2 |
|---|---|---|
| turns | 11 of 90 degrees and five 180s | 9 of 90 degrees (the first one at the start) and five 180s |
| crossings counted | 25 | 19 |
| time (simulator, calibrated, `--student`) | about 63 s (54 s fresh battery to 96 s tired: `--student-fast` / `--student-slow`) | about 54 s (47 to 85 s) |
| object 3 | reached after a turn at C4 MID | reached driving straight up C4 from x1 |
| where to put it down (simulator) | wheels from 12 cm behind C1 to 9 cm past it (it waits while the bar itself is on C1) | wheels within 7 cm of C1; further off, the first turn can end crooked and object 1 is missed (it goes on with the other two) |

Both did every condition of the simulator tests: the standard list, 12 runs with realistic motors, the
battery at 50% to 120% of the calibration speed, no overhang or a 3 cm one, `CELL_CM` 3 cm too big or too
small, noisy sensors, the robot 1.5 cm to the side of MID or 5 degrees crooked. The 1-3-2 route starts
with a turn where it stands, so it needs the wheels over C1 more exactly. 3-1-2 is the one tested the
most; use 1-3-2 if you want the shorter run and put the robot down carefully. The run log's START line
says `route=132` when that route ran. In the simulator: `DEFS="-DROUTE_ORDER=132" ./sim/e33_test.sh`.

`PRACTICE` in `calibration.h`: `1` stops the mission once the first object is put down (object 3 in
route 312, object 1 in route 132), `2` after the second. It stops as at the end (LED on 1 s, off 1 s) and
the run log ends with DONE. Use it to practise the first object and its turns back again and again.
**Set it back to 0 for the competition.**

## Competition day

- **Upload `e33_mission`** before you go (not `e33_calibrate`), with `PRACTICE 0` and the `ROUTE_ORDER`
  you tested. With USB in, the Serial Monitor says `MISSION: waiting for the line under the bar...`.
- **Calibrate in the room of the competition** if you can (a practice slot on the field). Sunlight from a
  window or strong lamps change what the sensors read. At every start the robot compares the white under
  the bar with the white of the calibration. More than a quarter of the way to black and it prints
  `WARNING: not the light of the calibration: calibrate here`, the LED flickers fast for 1 s before it
  drives, and the run log keeps it as `light=`.
  If you cannot calibrate there, at least run `MODE_SENSOR_CHECK` on their field.
- **Battery.** A fresh one for the calibration and for the run. A 9 V PP3 is weak for motors and servos:
  when the gripper moves it can drop so far that the Nano restarts (a brown-out, see above). 6 x AA or a
  2-cell Li-ion pack (7.4 V) holds up much better. A 470 to 1000 uF capacitor across the servo supply
  helps too. Bring spares. If the LED blinks long-short-short-short (twice) before it drives or after a
  pick or a place, the supply is low: change the battery before the next run (see "The battery").
- **Check the program on the Nano**: plug in USB before the run, with the robot **lifted off the field**
  (held in the air it never starts; on the start line it drives off 0.2 s after the Nano restarts, with
  the cable still in). The Serial Monitor must say
  `MISSION: waiting for the line under the bar...`. If it says `=== AUTOMATIC CALIBRATION ===`,
  `=== TURN CHECK` or another check, the Nano still has `e33_calibrate`: upload `e33_mission`.
- **Walk the field first**: gaps, loose tape and dirt near the crossings are what worn tape breaks
  most (see "Worn tape"). Repair them before your run. Look at the white too, and at where the objects
  sit (see "Setting up the field").
- **Put it down** with the wheels exactly over C1, on MID, facing east. It goes 0.2 s after it sees the
  line steadily (LED on), or 5 s after a run that did not finish.
- **To start again, use the power switch** (off, then on). A reset or plugging in USB during a run looks
  like a brown-out: the robot stops and blinks fast. Press reset once more (or switch off and on).
- **The LED**: on = line OK, it is about to go. On 0.5 s / off 0.5 s = no good line under the bar. On 1 s /
  off 1 s = DONE. Short blinks and a pause = a fault, count them (see "When it stops by itself" below). Fast = restarted mid-run.
  Long-short-short-short, twice, and it goes on = low supply: weak battery.
- **Bring the USB cable** and a laptop with the Arduino IDE. After a bad run, pick the robot up off the
  lines, plug in and copy the run log (`=== E33 RUN LOG BEGIN` ... `END`): it says what happened.

## Setting up the field

The robot never sees the objects or the targets. It counts crossings, turns where the route says, and
closes the jaws at a fixed place. So how well it does depends on how the field is laid out and on what
the sensors see on it. This section says how much room there is for each thing and what to do about it.
The numbers come from the simulator with your robot (`--student`, `--student-slow`, `--student-fast`,
both routes, with and without calibration).

### Objects and targets

- **Put each object on the end of its line, in the middle.** Up to 2.5 cm to the side still worked in
  every run; at 3 cm about one run in nine missed it. Along the line, put it where the worksheet says: the
  jaws close at the same place every run.
- **Calibrate before the run.** With calibration the jaws came down within about 1.3 cm of where they
  should; without it up to about 2.3 cm (a turn a little long or short moves the jaws sideways). That
  difference is most of the room you have.
- **Measure how wide your gripper takes an object.** The simulator takes an object when the jaws are
  within 3.5 cm of it. For yours: open the jaws, then (jaw gap minus object width) divided by 2. That is how
  far off the middle an object may be and still end up between the jaws. If it is only 2 to 2.5 cm, the
  robot still takes an object within about 1 to 1.5 cm of the line (with or without calibration), and
  misses some past that. Open the jaws wider if you can.
- **Targets are easy**: up to 3 cm off the line was still scored in every run.

### The tape

- **Matt black tape on a white field.** The robot needs about half of the usual contrast: black that reads
  500 (normally about 970, white about 100) still worked in every run. Below that (450) it works only after
  the calibration has measured that tape. Grey tape, shiny tape (it reflects the sensors' light back and
  reads grey) and a bar mounted too high all lower the contrast in the same way.
- **One weak sensor** in the middle of the bar (black reads 400) needs the calibration too. A weak one at
  either end of the bar does not matter.
- Check with `MODE_SENSOR_CHECK`: every sensor should say `OK`.

### A white field that is not the same everywhere

A darker patch on the white (a shadow, a stain, a seam, pencil, a grey sheet under part of the field)
makes the white read higher there. The robot copes with a patch where the white reads up to about 300
higher than where it was calibrated (about a third of the way to black). At 350 it stops with `FAULT 2`
(a crossing that never ends), `FAULT 6` or `FAULT 7` (a turn that does not find the line). Nothing in the
program can help there: such a patch looks like the edge of a line to the sensors.

(Before, the limit was about 250. A sensor that is on stays on until it reads below a lower level, so a
line edge does not flicker. That level was 30% of the way from white to black: on a patch above it, a
sensor that had just left the line stayed on. It is now 38%, `SENS_OFF_PCT` in `calibration.h`. 34% and
36% were tried too: 36 kept only about 60% of the gain on darker patches, 34 about a third. The one case
found where 38 does worse than 30: a hole in the tape, 5 cm across, right on the corner where C1 meets
TOP. Route 312 then follows the stub past the corner instead of stopping at the end of C1, and misses
object 1. So keep the corners of the field well taped.)

So:
- **Look at the field under the light of the run.** With `MODE_SENSOR_CHECK` running, move the robot by
  hand over the whole field and watch the white readings. Where they are more than about 150 above the
  rest, fix the field (clean it, cover the seam, move the lamp) or calibrate there.
- **Keep your own shadow off the field**, and do not stand between a window or a lamp and the field.

### Light that changes during the run

- **The white reading higher** (a lamp switched on, sunlight, someone standing over the robot): up to 300
  worked, whether it came slowly or all at once. At 350 most runs stop.
- **The black reading lower** (reflections on the tape): fine while black stays above about 570 (from 970).
  At 470 the robot no longer sees the line and waits.
- At the start the robot compares the white under the bar with the white of the calibration. More than a
  quarter of the way to black and it prints `WARNING: not the light of the calibration: calibrate here` and
  flickers the LED fast for 1 s. That warning comes well before the level where runs fail: **if you see it,
  calibrate again in that light.**

### Putting the robot down

- **Wheels over C1, on MID, facing east, with the line under the middle two sensors** (`bar 00011000` in
  the Serial Monitor).
- It is forgiving: turned up to 15 degrees, or up to 3 cm to the side, still worked. What counts is where
  the bar is, 9.5 cm in front of the wheels, so turned and moved to the same side add up: 10 degrees and
  2 cm to the same side is too much. 20 degrees is too much on its own.
- If the line is not in the middle of the bar the robot does not drive off. It waits, blinks the LED 0.5 s
  on / 0.5 s off, and prints `no line in the middle: put the bar on MID`. Put it down again.

## How exact must the ruler numbers be

The robot uses the seven ruler numbers in `calibration.h` to know where it is between two crossings:
when to slow down, where to turn, where the jaws are. The calibration measures speeds and turns, but it
does not correct these lengths (it even uses `CELL_CM` itself, see below). So how far may each one be off?

This was tried in the simulator: one number wrong at a time, on your robot (`--student`, and the tired and
fresh battery ends `--student-slow` / `--student-fast`), both routes, with and without calibration (the
calibration done on the same wrong field). A run counts as good only with 3/3 and every count at the right
crossing (the route check). The tolerance below is the worst of those 12 cases.

"Too big" means the number in `calibration.h` is bigger than the real length.

| number | how to measure | may be off by | what goes wrong past that |
|---|---|---|---|
| `SENSOR_AHEAD_CM` | Put a tape line exactly under the middle of the sensor windows. Measure along the floor from that line back to the point under the wheel axle (the middle of the wheel hub). | 1 cm too big, 2.5 cm too small | Too big: the robot rolls too far before a turn and comes out of it beside the line (FAULT 7 or 11). Too small: it turns before the wheels are on the crossing, and the jaws stop short (route 132 missed object 1). |
| `GRIP_REACH_CM` | Arm down, jaws closed on an object. From the point under the axle to the centre of the object. | 3 cm too big, 2 cm too small | Too big: the jaws close short of the object. Too small: the jaws go past it. Either way the gripper misses it. |
| `OBJ_BEYOND_CM` | From the centre of the TOP line to the centre of an object (and BOT: use the average). | 2 cm too big, 3 cm too small | Too big: the jaws go past the object. Too small: they close short of it. |
| `ROW_CM` | From the centre of MID to the centre of TOP, and MID to BOT, at C1 and at C4. Use the average. | 4.5 cm too small, 7.5 cm too big | Too small: a turn or an end is missed (FAULT 4). Too big: the robot turns too early and finds no line (FAULT 7 or 8). |
| `CELL_CM` | From the centre of the C1 line to the centre of the C4 line, along MID, divided by 3. | 3.5 cm too small, 6 cm too big | Too small (calibrated): the calibration measures the 90 degree turn with the wheels off the crossing, and turns then fail (FAULT 6 or 11). Too big: a crossing is judged missed (FAULT 4). |
| `TGT_BEYOND_CM` | From the centre of the BOT line to the centre of a target mark. | 3 cm too big, 5.5 cm too small | Too big: the robot drives too far past BOT, and the 180 after putting the object down ends off the line. Too small: the object goes down short of the target. |
| `TAPE_W_CM` | The width of the black tape, in a few places. | 1 cm either way | The robot works out where the middle of a line is from its edge: half this error goes into every stop. |

**The order to measure them.** Most exact first: `SENSOR_AHEAD_CM`, then `GRIP_REACH_CM` and
`OBJ_BEYOND_CM`, then `ROW_CM`, `CELL_CM`, `TGT_BEYOND_CM`, `TAPE_W_CM`. Aim for 0.5 cm on every one: with
every number off by 0.3 to 1 cm at once (all one way, or all the other way), all runs were good.

**Three of them add up.** Where the jaws stop is `SENSOR_AHEAD_CM + OBJ_BEYOND_CM - GRIP_REACH_CM` past the
TOP or BOT line. Their errors add: `GRIP_REACH_CM` 1 cm too small, `OBJ_BEYOND_CM` 1 cm too big and
`SENSOR_AHEAD_CM` 0.5 cm too big put the jaws 2.5 cm past the object, and then a few runs missed it (the
same errors the other way, 2.5 cm short, were all good). Keep the three together within about 2 cm.

**The tape itself.** Tape narrower than about 1.1 cm fits between two sensors: the robot does not see MID at
the start. Up to 3 cm wide worked. That is the tape, not the number: measure it and write it in.

**A 30 cm field with the 25 cm numbers** (a common field size): all runs but two were good (route 312,
calibrated, fresh or tired battery: FAULT 11). If your field is not 25 cm, write in its size.

These are simulator results for your robot's model, one seed each (the combinations: three seeds). The real
robot will have its own small errors on top, so treat them as the most you can get away with, not a target.

### Route 132 (changed)

Route 132 was the weak one: with a fresh battery and no calibration it failed with `ROW_CM` only 2.5 cm
too small. It learned its speed from the short leg back to MID after the first pick, a few cm that hold
mostly the error of the pick and the 180. Now a leg shorter than half a cell does not set the speed, and the
few cm driven past a crossing are corrected when the speed is. The table above is with that change.

## What calibration measures, and why it matters

| item | how | used for |
|---|---|---|
| sensors | swings the bar over the line, keeps each channel's white and black | a threshold for **each** channel between its own white and black (the slides use one number for all 8; 800 was too high for your bar, which is why the robot did not move) |
| dead band | turns one wheel at a time very slowly until the line moves under the bar | every speed from the slides really moves both wheels ("speed 50" did not move your left wheel) |
| left/right match | adds up how much speed the PID gave each wheel between two crossings. To drive straight the stronger wheel must have been given less. It does not matter where the line sits under the bar | your left wheel is about 75% of the right. The stronger motor is turned down so the same number gives the same speed |
| backward match | compares the time of a spin left (left wheel backward) with a spin right (right wheel backward) | spins turn about the middle of the axle instead of about the weak wheel |
| speed | time between crossings | the exact distances before turns and grips |
| turn time | a 540 degree spin on C4 and on C1 | time limits in the turns |

## Why it went wild when turning back, and what changed

The 180 at the end of a column is where the old code failed. There were five reasons:

1. **The spin speed was too low for the weak left wheel.** At the old fixed turn duty the left wheel,
   especially going backward, barely moved. The robot then turned about the left wheel instead of about
   its middle, so the bar swept a much bigger circle and the lines came in at the wrong moments.
   Now the dead band and the backward trim make both wheels really turn, a spin starts with a short
   kick, and if a turn is slower than expected it adds speed step by step.
2. **The turn waited for one exact sensor pattern** (like `00000011`). While spinning, a line crossed at
   a slant lights 4 to 6 sensors, which is not in the table, and a fast spin can skip the pattern.
   Now the turn follows each line as it sweeps under the bar (in at one end, across the middle, out at
   the other end), slows down when the right line comes in, and stops when it is in the middle.
3. **It turned the wrong way at the corners.** At C4 TOP (object 3) and C1 BOT it turned so the bar
   swung over the short line sticking out past the column. Depending on its length the bar saw a line
   there or not, so the robot passed the column and stopped along the TOP line. Now it always turns
   toward the field at a corner (C4 TOP left, C1 TOP right, C4 BOT right, C1 BOT left), where the line
   is always there.
4. **The turns sped up during the spin** (`upSpeed()` was called inside the turn). Turns now use fixed speeds.
5. **Reading the sensors while turning.** Each channel is now read twice and the first reading thrown away:
   read at once, a channel shows a little of its neighbour, which blurs a line as it sweeps across the bar.
   A line that disappears in the middle of the bar during a spin (a weak sensor) is waited for instead of
   being counted as passed.

The slides' turn names (`turnRight90`, `turnLeft180`, ...) are kept; only their insides changed.

## The battery

A 9 V battery gets weaker during the day, so the robot slows down and the speed saved by the calibration
is no longer right. Every run therefore measures its own speed: at the first crossing (its distance from
the start is known) and again at every crossing it drives through at speed. Distances before turns and
grips, the "a crossing was missed" check and the time limits of the turns all use this live speed. In the
simulator the mission still works at anywhere from 45% to 125% of the speed it was calibrated at.
The run log's `DONE` line shows it: `speed=x0.72 (battery weaker than at calibration)` below x0.80.

### The supply voltage (the POWER lines)

The Nano also measures its own 5 V supply (VCC), with no extra wire: the chip compares its built-in
1.1 V reference with VCC. It measures only while the robot stands still, so the line following is never
disturbed: before it drives, and all through the servo moves of every pick and place. The run log keeps it:

```
t=0.0 n=0 START cal=1 run#1 light=0
t=0.0 n=0 POWER start rest=5.00V
...
t=9.8 n=5 PICK bar-past-line=2.6 target=2.5
t=9.8 n=5 POWER pick rest=5.00V min=4.94V
...
t=92.6 n=42 DONE rejected=0 missed-counted=0 retries=0 speed=x1.03
t=92.6 n=42 POWER end rest=5.00V lowest=4.94V
```

- `rest=` is VCC standing still with the servos not moving. `min=` is the lowest while the servos of that
  pick or place moved. The `end` line, the last one of the log, has the lowest of the whole run (the arm
  moves after a place included). A POWER line has the time of the line before it, the one it belongs to.
- The regulator on the Nano needs about 6.2 V from the battery to give 5 V. While the battery can give
  that, VCC stays at 5 V. When the battery sags under the servo current (a PP3 does), VCC follows it
  down: `min=` more than about 0.25 V below `rest=` means the battery is at its limit. Near 2.7 V the Nano
  restarts (a brown-out, see "When it stops by itself"), and long before that the sensors and the servos
  suffer.
- The chip's 1.1 V is only about 10 % exact, so the volts are approximate: compare them with each other
  (`rest=` against `min=`, one run against the next). To make them exact, see `BANDGAP_MV` in
  `calibration.h`. `?` means the chip gave no usable reading. With only the USB cable (no battery), VCC
  is the USB 5 V after a diode, about 4.4 to 4.8 V, so a low start then is normal.
- Below `VCC_WARN_MV` (4.5 V) the line ends in `LOW`, it prints
  `WARNING: low supply: weak battery (README: The battery)` and the LED blinks long, short, short,
  short, twice (Morse "B" for battery, about 3 s). Then it goes on: this warning never stops the mission.
  At the start it comes before it drives.
- What to do: a fresh battery. Better, 6 x AA (also 9 V, and much stronger than a PP3) or a 2-cell Li-ion
  pack (7.4 V). A stronger battery also makes the motors faster: calibrate again after changing it, and if
  the report says `FAST ROBOT`, lower the speeds as it says. A 470 to 1000 uF capacitor across the servo supply, close to the servos, covers the short
  current peaks of a servo starting to move.
- After a restart in the middle of a run, the POWER lines before `RESTARTED here` show how low the supply
  had got. A low one there means the restart was a brown-out. But a tired battery can read 5 V at rest
  and still collapse at the first servo move: if `RESTARTED here` comes right after a `PICK` or `PLACE`
  line, it restarted while the servos moved, which is a brown-out too (its POWER line is written only
  after the move). The log reader says so.

## Checks at every crossing

Before a crossing is counted, the robot checks it:

- **distance**: a crossing far too early is a double count and is ignored. One a whole cell late means a
  crossing was missed: if it was one to drive straight over, it is counted; if it was a turn or an end,
  the robot stops safely.
- **shape**: a real crossing lights both ends of the bar. Only one end lit means the robot is driving along
  the wrong line. (At the edge of the field only the field side has to light.)
- **what is just past it**: a MID crossing has line straight ahead, the end of a column has none. The robot
  will not pick up or put down where there is line ahead: it goes on to the next crossing once, then stops.
- after every turn: a narrow line must be under the middle of the bar.

One line is printed for every crossing and every turn, for example
`n=5 keep_item: picking up d=16.6/16.4 m=11111111 ahead=BLANK v=0.98`
(count 5, measured 16.6 cm where 16.4 was expected, all 8 sensors lit, no line ahead, speed correction 0.98).

## Worn tape

Real fields get gaps, scuffs and stray marks. What the robot does about them:

- **a gap in the line**: when the line vanishes from the MIDDLE of the bar (drifting off a line, it leaves
  through one end instead), the robot drives straight on over it, up to 5 cm, before it searches to the
  sides. A gap just past a MID crossing is not taken for the end of a column: it looks 3 cm further first.
  Near a column end it waits 2.5 cm before deciding that the line ran out, so a T just past a short gap is
  still counted normally.
- **a gap where a turn looks for the new road** (the bar crosses every road 9.5 to 10.4 cm from the
  wheels): if the road comes in and vanishes, or shows only at the trailing end of the bar, the robot
  drives 5 cm on and turns back to find it beyond the gap (the trace says `turn: new road not seen (a
  gap?)`). A road that first shows in the middle of the bar after a long blank also counts. It never
  decides this on time alone: a slow or crooked turn on a real floor takes far longer than expected.
- **a faded arm** of a crossing (only one end of the bar lights): still counted when the crossing came
  where it was expected and what is past it fits (line after a MID crossing, none after a column end;
  either one heading out of the field at C1 or C4, where the line ahead is only the short overhang).
  The first crossing after a stop (a turn, a 180, the start) reads a few cm long on the odometer (the
  wheels need a moment to get up to speed), so there "where it was expected" reaches further.
- **a black mark beside the line** (dirt, a piece of tape): a "crossing" that did not light both outer
  sensors, well before the next crossing is due, is not counted (`not counted: a mark beside the line?`).
  Once per run, a one-ended "crossing" a little before the next crossing is due, which would otherwise
  stop the robot with FAULT 3, is not counted either (the log shows `RECOVER-MARK`).
  After a turn the narrow spot nearest the middle is taken as the line, and if a mark sits right beside
  the line where the robot stopped, it drives 3 cm on and looks again.

In the simulator (both routes, calibrated, ideal and realistic motors), damage was put at thousands of
places along every kind of leg, at and around the crossings. Share of runs that failed:

| damage | before | now |
|---|---|---|
| gap of 1 cm | 4% | 1% |
| gap of 2 cm | 12% | 4% |
| gap of 3 cm | 30% | 7% |
| gap of 4 cm | 43% | 10% |
| black mark 1 x 1 cm | 0.7% | 0.1% |
| black mark 2 x 1 cm | 2.5% | 0.3% |
| black mark 3 x 3 cm | 13% | 5% |
| one arm missing next to a crossing, 1 to 4.5 cm | 50% | 4% |
| one arm missing next to a crossing, 1 to 9 cm | 65% | 44% |

With noisy sensors on top the gains are the same. On an undamaged field nothing changed: the calibrated
runs are byte-identical in the simulator. What still goes wrong most: a longer missing piece right next to
a crossing, a gap of 3 cm or more exactly where a turn looks for the road, and big marks near a crossing.
So before a run, walk the field: **repair gaps and loose tape, above all at the crossings and within 10 cm
of them, and wipe off dirt and stray pieces of tape.**

## When it stops by itself (the LED on D13 blinks the number)

| blinks | meaning | look at |
|---|---|---|
| 1 | line lost and not found again | sensor height, lighting, `SENS_ON_PCT` |
| 2 | on a "crossing" for more than 1 s (driving along a line), or the whole bar sees black for too long | the turn before it; sensor levels |
| 3 | half crossing: on the wrong line | the 180 before it |
| 4 | a turn or an end was missed | `CELL_CM`, `ROW_CM`, battery |
| 5 | no line after a MID crossing | the count is off |
| 6 | a turn never found its line | dead band, battery, `TURN_SP` |
| 7 | no line under the bar after a turn (it first tries once more with longer nudges: `RECOVER-TURN` in the log) | turn speed, calibration, battery |
| 8 | no line ahead where a MID crossing should be | the count is off |
| 9 | no column end found, even at the next crossing | `ROW_CM`, the count |
| 10 | a 90 degree turn passed a line first | turn speed too high |
| 11 | a 180 took far too long or too short | calibration |
| 12 | no crossing for far too long: the wheels are not really turning | motor switch, battery, only USB power (before the first crossing it keeps the last run's log) |

A **brown-out** (the battery could not supply the servos and motors) makes the Nano restart. The old code
then opened the gripper and drove off as if at the start. Now the robot recognises a restart in the middle of
a run, prints `RESTARTED IN THE MIDDLE OF A RUN`, blinks fast and does not drive. (The usual Nano bootloader
wipes the chip's own reset record, so the robot also keeps a mark in memory while the mission runs;
switching the power off clears it, a restart does not.) Fit a fresh battery, put the robot at the start and
press reset. A 470 to 1000 uF capacitor across the servo supply also helps. The same happens if you press
reset or plug in USB during a run: press reset once more to start again.

## When the calibration stops

The report's first line says `STOPPED during` and the step (with the USB cable in, the line before it
also says why). Fix the cause, then lift the robot and put it down at the start to calibrate again.

| STOPPED during | why | what to do |
|---|---|---|
| deadband | a wheel did not move by duty 220, or moved the wrong way | `MODE_MOTOR_CHECK`; motor switch, battery, STBY wire; `LEFT_REVERSED` / `RIGHT_REVERSED` |
| sensors | the sweep lost the line, or 3 or more sensors barely see it | `MODE_SENSOR_CHECK`; bar height |
| move (the first, slow leg) | line lost, a crossing at the wrong distance, or more than 20 s | place it again: wheels over C1, on MID, facing east; check `CELL_CM` |
| turn | a spin did not see the lines, or no line under the bar after it | fresh battery; `TURN_SP` |
| cruise (legs 2 to 4) | line lost, stuck on a crossing, a crossing at the wrong distance | fresh battery; `CELL_CM` |

## What the LED on D13 means

| LED | mode | meaning |
|---|---|---|
| on 0.5 s, off 0.5 s | mission, calibrate | waiting: no good line under the middle of the bar |
| on | mission | line OK: it goes after 0.2 s (5 s after a run that did not finish) |
| flickers fast for 1 s, then it drives | mission | the light is not the light of the calibration: calibrate there next time |
| on | calibrate | line seen: starts in 3 s, then stays on while it calibrates |
| on 0.9 s, off 0.1 s | calibrate | finished and saved: plug in USB to read the report |
| fast, 5 times a second | calibrate | stopped with a problem: plug in USB to read the report |
| on 1 s, off 1 s | calibrate | calibrated before: lift it and put it down to calibrate again, or upload `e33_mission` |
| on 1 s, off 1 s | mission | DONE |
| 1 to 12 short blinks, then a pause | mission, turn check | a fault: see "When it stops by itself" |
| fast, 5 times a second | mission | restarted in the middle of a run (brown-out, above) |
| long, short, short, short, twice (about 3 s), then it goes on | mission | low supply: weak battery (see "The battery") |
| 2 times a second | motor check | waiting: lift the robot (it only turns the wheels while lifted) |
| 2 times a second | sensor check | waiting for Enter (or 20 s) |
| on 1 s, off 1 s | sensor check | the report is printed; Enter runs the check again |
| 2 times a second | turn check | waiting for Enter (or 30 s) |
| on 0.5 s, off 0.5 s | turn check | waiting: no good line under the middle of the bar |
| on 1 s, off 1 s | turn check | finished: the report is printed, Enter runs it again |

## Testing without the robot

`sim/` runs these exact programs in a simulated field (`sim/e33_test.sh` builds the mission from
`e33_mission` and the calibration from `e33_calibrate`):

```
./sim/e33_test.sh
./sim/e33_test.sh "--trimL=0.75 --trimLr=0.6" "--overhang=3"
VERBOSE=1 ./sim/e33_test.sh "--trimL=0.75"
```

Each condition runs the mission without calibration, then the calibration followed by the mission.
By default the simulator is ideal (no slipping, no momentum, a perfect battery). `--real` adds motors
that take time to speed up and slow down, rolling after a brake, tyre scrub in spins, a turning point that
wanders, and the battery sagging while the servos work (see `./sim/sim_e33.exe --help`). Both show that the logic
is right, not that the real robot will be.

The simulator also has a simple supply model for the POWER lines: a battery (`--vbat`, `--rint`), the
regulator's dropout, and the currents of the Nano, the motors and the servos. By default it is a fresh
PP3 and VCC stays at 5.00 V; `--real` is a half-used one (8.2 V, 2.5 ohm): VCC dips to about 4.94 V
while a servo moves. `--vbat=7.6 --rint=3` gives the warnings (and the calibrated mission still completes);
`--vbat=6.0 --rint=4` browns out at the first pick: the run ends there, and the restart build
(`-DSIM_RUNMARK=0xE33A55C3UL`) shows what the Nano does next.

**Route check.** `PATHCHECK=1 ./sim/e33_test.sh` (and the same with `DEFS="-DROUTE_ORDER=132"`) also
checks every count of each calibrated mission against the route with `sim/path_check.py`: where the bar
was at each count, the heading after each turn, each pick and place. A run can put all three objects in
place for the wrong reason (for example a 180 turned the wrong way round); this catches it.

**Worn tape and ruler errors.** `--erase=X:Y:R` (a round gap), `--wipe=X:Y:HW:HH` (a clean cut),
`--patch=X:Y:HW:HH` (a black mark) and `--row=CM` (the row pitch apart from `--cell`), with X and Y in
cm, C1 MID at (0, 0), east and north positive.

**Your own printouts.** `bash sim/replay_test.sh` plays the Serial Monitor lines you sent (kept in
`sim/student_frames.txt`) into the real programs, the robot standing still, and prints PASS or FAIL
for each part: with the line under the middle the mission must start after 0.2 s; on white, on black,
lifted or on a crossing it must never start; MODE_METER and MODE_SENSOR_CHECK must read each part right.
It also checks that there is no light warning at the start, with the levels in calibration.h and after
a simulated MODE_CALIBRATE on your whitest white. Your printouts showed why that matters: with the line
under the middle, the white beside it reads 55 to 230 above your plain white (A0 329 against 75) in the
same light, so the warning now waits for a quarter of the way to black (it was a fifth, which that pose
reached at every start after a calibration).
To add new printouts, copy the lines from the Serial Monitor (the mission's waiting screen or MODE_METER:
any line with `raw` and 8 numbers) into that file under a new header line. Its first word says where
the bar was: `[white ...]`, `[black ...]`, `[middle ...]`, `[lifted ...]` or `[crossing ...]`, for
example `[white under the window]`. Then run it again; `MARGINS=1 bash sim/replay_test.sh` adds a table
of how far each sensor is from its ON and OFF levels. With your printouts, a sensor on the line reads
at least 399 above its ON level and a sensor on white at least 290 below its OFF level; the closest is
A5 just beside the line (307, which is 124 below its OFF level of 431). Lifted, all 8 read 676 to 906,
like black, so a lifted robot never starts. (The part you sent as a crossing reads white on all 8:
please send one with the bar on a crossing, for example C2 on MID.)

**Field set-up.** `--objoff=NAME:DX:DY` (move object o1..o3 or target x1..x3 by DX, DY cm),
`--gripr=CM` (how close the jaws must be to take an object, 3.5 by default), `--wzone=X:Y:HW:HH:DW[:MASK]`
(a darker patch: inside it the white reads DW higher), `--wshift=DW:T0:T1[:MASK]` and `--bshift=...` (the
white or the black level moving by DW during the run, from T0 to T1 seconds). MASK picks the sensors (hex,
default ff = all eight). "Setting up the field" was measured with these. All are off by default.

## How far it was tested (simulator)

With calibration, the mission completed in every condition tried, including:
- the left wheel at 45% of the right (yours is about 75%), or the right wheel at 55% of the left
- one wheel weaker going backward than forward
- the battery giving anywhere from 45% to 125% of the speed it had during calibration
- dead bands from 15 to 70
- no line sticking out past C1 / C4, gripper reach 1 to 2 cm off, the robot placed up to 12 cm behind the start or 9 cm past it with route 312, and within 7 cm with route 132
- very noisy, weak, uneven, stuck or partly dead sensors (except a dead sensor in the MIDDLE of the bar)
- the light changed after the calibration: white reading 100 at calibration (black 900) and anywhere from
  20 to 400 in the run. From 420 it failed (it was 350 before `SENS_OFF_PCT` went from 30 to 38); the
  start warning appears from 300. With your sensors, see "Setting up the field".
- `ROW_CM` 4.5 cm too small or 7.5 cm too big, and the other ruler numbers: see "How exact must the ruler numbers be"
- worn tape: see "Worn tape" for what it tolerates
- objects off the line, grey or shiny tape, a darker patch on the field, crooked starts: see "Setting up the field"
- every count of every calibrated run checked against the route (`PATHCHECK=1`), both routes

Without calibration it only works when the guesses in `calibration.h` are close to your robot
(left wheel about 75% of the right). Calibrate once.

## What is not known yet

- None of this has run on the real robot yet. The numbers in `calibration.h` marked as guesses need the ruler and the calibration.
- The simulator's field has lines sticking out 12 cm past C1 and C4 and objects 5 cm past the outer line. Measure yours.
- The PWM frequency fix (D3 made to run at the same 976 Hz as D6) explains a few percent of the left/right difference at most; the calibration handles the rest.
