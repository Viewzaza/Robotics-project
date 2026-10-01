# robot_e33_v3: the e33 mission, built on the course slides

**v3 is robot_e33_v2 that does its best.** A fault (a lost line, a half crossing, a missed turn...) no
longer stops the robot and blinks the LED: it prints the fault on Serial, notes it in the run log and goes
on with the route (see "When it stops by itself" below). Everything else is the same as v2.

### What v3 does differently (from your 19 real runs)

- **A restart in the middle of a run no longer stops the robot.** Your Nano restarted at picks, at
  places, and in the turn or the 180 right after them (the 9 V battery dips when a servo or a motor
  starts). v2 then printed `Not driving ... press reset` and blinked fast. v3 keeps, in memory that a
  restart does not clear, the count (`numGride`), the servo angles, whether an object is held, the speed
  it measured and the sensor levels of this run. After the restart it sends the servos the same angles
  before they are attached (a held object stays in the jaws), prints
  `MISSION: carrying on at n=...` and carries on: a pick or a place is done again from where the servos
  are, a turn or a 180 is finished, and on the way to a crossing it follows on and counts the next one.
- **When it does not carry on:** switching on (that memory is lost when the power is off, and a check
  sum tells), the reset button or plugging in USB (a new start on purpose), and a robot that is lifted
  when it restarts (the bar sees black everywhere, or no line on the way to a crossing): it prints
  `Lifted: a new start.` and waits for the normal start. Note: on your Nano a battery dip reports
  `reset cause: power-on`, exactly like switching on (all your restarts did), so the memory and its check
  sum, not that flag, tell the two apart.
- **No false crossing just after a standing start.** While it speeds up from standing (the start, every
  turn, every 180) your bar read darker for a moment, and it saw a "crossing" 1.4 to 4 cm on, almost every
  time. After the 180 at C4 TOP the leg back to MID is only 9.7 cm, and v2 counted one at 2.0 cm as C4
  MID: it turned 7.7 cm early ("it turns 180 again"). v3 ignores a crossing closer than 3.5 cm, or than
  half the leg, after a standing start (`START_BLIND_CM`, `START_BLIND_FRAC` in `controlLibrary.h`).
- **No waiting on the LED before it drives:** the light warning (v2 flickered 1 s) and the low supply
  warning (v2 blinked about 3 s, at the start and after every pick and place) are printed on Serial
  only, and the robot goes on at once.
- Kept from v2: the same route, messages and run log, the standing-still measurement of the sensor
  levels when the default levels miss the line (it is what copes with your loose A0 wire), and your own
  numbers in the CAL VALUES of `calibration.h`.

Version 2. The first version is kept unchanged in `../robot_e33/` (and in git). Version 2 adds:
- **MODE_SENSOR_CHECK**: white and black of every sensor, printed as a report you can copy and send;
- **a run log**: every run is recorded in EEPROM and printed at the next start, so a run without the
  USB cable can be read afterwards;
- **MODE_GRIPPER_CHECK**: move the gripper and the arm one degree at a time from the Serial Monitor to
  find your angles;
- **MODE_TURN_CHECK**: the mission's own turns, 24 times on one crossing, each timed, with a report;
- **two route orders** to choose from (objects 3, 1, 2 or 1, 3, 2) and a **practice** setting that
  stops after the first object (see "Two routes, and practice");
- **a log reader**: `../tools/e33_log_reader.html` explains what the Serial Monitor printed;
- **the supply voltage**: the Nano measures its own 5 V at the start and during every pick and place,
  keeps it in the run log and warns about a weak battery before it causes a brown-out (see "The battery").

This is the firmware for the e33 worksheet (3 objects, 3 targets, 4 columns x 3 lines).
It keeps the slides' structure (robot04 to robot11): `beginFnc()`, `followLine()` with
`pidFNC(errorInput,0,1,0,0.7)`, `countGrid()`, the `switch(numGride)` from robot10, and the
helpers `turn90()`, `keep_item()` and `place_item()`. Every line that differs from the slides
is marked `[E33]` in the code, with the reason.

## The files

| file | what it is | edit it? |
|---|---|---|
| `calibration.h` | your robot and field numbers, the mode switch, the CAL VALUES | **yes, only this one** |
| `robot_e33_v3.ino` | the route (robot10 style) and the three helpers | no |
| `controlLibrary.h` | the slides' library, with the [E33] changes | no |
| `pidLibrary.h` | the slides' PID, gains unchanged, two safety guards | no |
| `calibrationMode.h` | automatic calibration, sensor meter, motor check | no |
| `checkModes.h` | sensor check, gripper check and turn check | no |
| `runLog.h` | the record of the last run, kept in EEPROM | no |
| `../tools/e33_log_reader.html` | a page that reads what the Serial Monitor printed (open it in Chrome or Edge) | no |

Open the Serial Monitor at **115200** baud to see what the robot is doing (the Arduino IDE's default is
9600: at 9600 you only see strange characters). The first two lines at every start are
`reset cause: ...` (`unknown (the bootloader cleared it)` is normal on many Nanos) and `CAL: ...`
(which calibration is in use).

## Sending me data (copy the block between BEGIN and END)

Open the Serial Monitor at **115200** baud, line ending "Newline".

1. **Sensor check** (`ROBOT_MODE MODE_SENSOR_CHECK`). The robot never moves. Four steps, each started with
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
2. **Calibration report** (`MODE_CALIBRATE`, after a calibration run): plug in USB and open the Serial
   Monitor; it prints `=== CALIBRATION REPORT` and the `#define CAL_...` block. Copy from
   `=== CALIBRATION REPORT` down to `end CAL VALUES` (with the Serial Monitor's timestamps switched off).
3. **Run log** (`MODE_MISSION`): after a run (with or without the cable), pick the robot up off the lines,
   plug in USB and open the Serial Monitor. It prints `=== E33 RUN LOG BEGIN ===` ... `END`: every crossing (distance measured and
   expected, sensors, line ahead), every turn (time, lines passed), each pick and place, a fault if there
   was one, and how the run ended (normally, a fault, the power switched off, or the Nano restarting in
   the middle: a brown-out). `START cal=1 run#1` says which calibration was in use (0 the defaults, 1 the
   EEPROM calibration, 2 values pasted into `calibration.h`); `light=` is how far the white under the bar
   at the start was from the calibrated white (near 0 is good; see "Competition day"). `DONE ... speed=x0.93`
   is the speed the run measured against the calibration (the battery). The `POWER` lines are the supply
   voltage at the start, at each pick and place, and the lowest of the run (see "The battery"). The log is kept until the next run has really moved (its first turn), so
   switching on to read it does not wipe it. After a run that did not finish, the next start waits 5 s
   (instead of 1.5 s) before driving: lift the robot, or switch the motors off, if you only want to read
   the log.
4. **Gripper check** (`MODE_GRIPPER_CHECK`): type `o` `c` `r` (open, closed, release), `+` `-` (gripper one
   degree tighter / looser), `d` `y` `h` (arm down, carry, high), `u` `j` (arm one degree up / down),
   `t` (a test pick). To find `GRIP_CLOSED`: object in the jaws, a strip of paper between one jaw and the
   object, press `+` until the strip is held. That angle is where the jaws touch. After closing, the
   mission opens `GRIP_BACKOFF_DEG` (3) again to hold, so set `GRIP_CLOSED` `GRIP_BACKOFF_DEG` + 2 = 5 degrees
   tighter, and check with `t` that the object does not slip. The servo must not buzz while holding.
   Every line also prints the sensor bar (`bar A0..A7:`). With the robot on white, compare the numbers
   with the arm down, carrying and high, with the object in the jaws: they must stay near the white
   numbers. If one moves towards black, the arm or the object is in front of the sensors (or shines their
   light back) and the robot can see a line that is not there.
5. **Turn check** (`MODE_TURN_CHECK`, after `MODE_CALIBRATE`: it uses the saved calibration like the
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
reads the run log, the live trace, the sensor report, the calibration report and the gripper check. It
says how the run ended, what a fault number means and what to look at. It compares every measured
distance and turn time with the expected one, MID legs and column legs separately, so a wrong `CELL_CM`
or `ROW_CM` shows up as a number of cm. It also lists any line it did not recognise. Serial Monitor
timestamps are fine. **Copy summary** copies a short text version: send it together with the full paste.

## Getting it running (about 20 minutes)

For every step with a mode: change the mode line in `calibration.h`, for example
`#define ROBOT_MODE   MODE_SENSOR_CHECK`, upload, and open the Serial Monitor (115200 baud, "Newline").
The new mode starts as soon as the upload ends, so **lift the robot or switch the motors off before you
upload**: on the field the mission drives off 1.5 s after it sees the line, and the motor check turns the
wheels after 3 s, with the USB cable still in.

1. **Measure with a ruler** and put the numbers in `calibration.h`:
   `CELL_CM` (C1 to C4 along MID, divided by 3), `ROW_CM` (MID to TOP), `TAPE_W_CM`,
   `GRIP_REACH_CM` (wheel axle to the centre of an object held in the gripper),
   `OBJ_BEYOND_CM` and `TGT_BEYOND_CM` (outer line centre to object / target centre).
   `SENSOR_AHEAD_CM` is your 9.5 cm.
2. **Check the gripper angle.** Close the gripper slowly on a real object and note the angle where the
   jaws just touch it (`MODE_GRIPPER_CHECK` helps). `GRIP_CLOSED` should be `GRIP_BACKOFF_DEG` + 2 = 5 degrees
   tighter than that (after closing, the mission opens `GRIP_BACKOFF_DEG` again, so it holds 2 degrees past
   the touch), not the "empty and fully closed" angle. A servo squeezing much tighter than the object pulls a
   lot of current from the 9 V battery.
3. **`MODE_SENSOR_CHECK`** (set `ROBOT_MODE` in `calibration.h`, upload). The robot never moves. Follow the
   four steps; every sensor should say `OK`, and the report should say `step 2: every sensor saw the line: yes`.
   `sees_line_now` tells whether the levels in use now see the line. Before step 5 these are the default
   guesses, so `NO` is normal then. Do this check again after step 5: then every sensor must say `yes`.
   `WEAK` or `DEAD`: check the bar height and that sensor's wire. `NOISY`: check the wiring, keep sunlight off.
   (`MODE_METER` prints the raw values live.)
4. **`MODE_MOTOR_CHECK`** with the **wheels off the ground**. Each wheel runs forward and backward in turn.
   If one runs the wrong way, set `LEFT_REVERSED` or `RIGHT_REVERSED` to 1. If the OTHER wheel turns
   (the right one when it says LEFT), swap the two motor plugs. If a wheel never turns: motor switch,
   battery, the STBY wire (D10).
5. **`MODE_CALIBRATE`** with a **fresh battery**. Put the robot at the start (wheels over C1, on MID,
   facing east), switch on and step back. 3 s after it sees the line (LED on) it calibrates itself for
   about 90 s and stops back at the start. The LED stays on with short gaps when it finished, or blinks
   fast if it stopped with a problem.
   Then plug in USB and open the Serial Monitor: the Nano restarts and prints the saved report and the
   CAL VALUES block again. If the first line says `STOPPED during ...`, see "When the calibration stops"
   below. To calibrate again, lift the robot and put it down at the start. (If this Nano was calibrated
   before, also with `robot_e33`, switching on at the start does nothing, LED on 1 s / off 1 s, until you
   lift it and put it down.)
6. **`MODE_MISSION`**. Put the robot at the start and switch on. Put the wheels exactly over C1 (the
   simulator's limits are in "Two routes, and practice": the 1-3-2 route needs it more exactly). It waits, standing still, until it has seen
   the line steadily for 1.5 s, then goes. It only starts with the bar on a plain line (not on a crossing,
   and never while held in the air, where every sensor reads black). No buttons. It never makes a calibration movement at the start.
   (One exception, without any movement: if the sensors were never calibrated and the default levels cannot
   see the line properly, for example a bar whose black reads low, whose white reads above the default
   level, or which reads black LOW, it measures the levels from the line under the bar while standing
   still, and says so.)

## Two routes, and practice

`ROUTE_ORDER` in `calibration.h` chooses the order of the objects:

| | `312` (the default): objects 3, 1, 2 | `132`: objects 1, 3, 2 |
|---|---|---|
| turns | 11 of 90 degrees and five 180s | 9 of 90 degrees (the first one at the start) and five 180s |
| crossings counted | 25 | 19 |
| time (simulator) | about 93 s | about 77 s |
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

- **Upload `MODE_MISSION`** before you go (not a check mode), with `PRACTICE 0` and the `ROUTE_ORDER`
  you tested. With USB in, the Serial Monitor says `MISSION: waiting for the line under the bar...`.
- **Calibrate in the room of the competition** if you can (a practice slot on the field). Sunlight from a
  window or strong lamps change what the sensors read. At every start the robot compares the white under
  the bar with the white of the calibration. More than a fifth of the way to black and it prints
  `WARNING: not the light of the calibration: calibrate here` (v3: it does not wait for it), and the run
  log keeps it as `light=`.
  If you cannot calibrate there, at least run `MODE_SENSOR_CHECK` on their field.
- **Battery.** A fresh one for the calibration and for the run. A 9 V PP3 is weak for motors and servos:
  when the gripper moves it can drop so far that the Nano restarts (a brown-out, see above). 6 x AA or a
  2-cell Li-ion pack (7.4 V) holds up much better. A 470 to 1000 uF capacitor across the servo supply
  helps too. Bring spares. If the Serial Monitor or the run log shows `WARNING: low supply` or a POWER
  line ending in `LOW`, change the battery before the next run (see "The battery").
- **Walk the field first**: gaps, loose tape and dirt near the crossings are what worn tape breaks
  most (see "Worn tape"). Repair them before your run.
- **Put it down** with the wheels exactly over C1, on MID, facing east. It goes 1.5 s after it sees the
  line steadily (LED on), or 5 s after a run that did not finish.
- **To start again, use the power switch** (off, then on), or the reset button: both are a new start
  (it waits for the line at the start). If the Nano restarts by itself during a run (the battery dipped)
  it carries on where it was; lift it if you do not want that (see "What v3 does differently").
- **The LED**: on = line OK, it is about to go. On 0.5 s / off 0.5 s = no good line under the bar. On 1 s /
  off 1 s = DONE. (With `STOP_ON_FAULT 1`: short blinks and a pause = a fault, count them.)
- **Bring the USB cable** and a laptop with the Arduino IDE. After a bad run, pick the robot up off the
  lines, plug in and copy the run log (`=== E33 RUN LOG BEGIN` ... `END`): it says what happened.

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
- Below `VCC_WARN_MV` (4.5 V) the line ends in `LOW` and it prints
  `WARNING: low supply: weak battery (README: The battery)`. Then it goes on at once (v2 also blinked
  the LED for about 3 s first): this warning never stops the mission.
- What to do: a fresh battery. Better, 6 x AA (also 9 V, and much stronger than a PP3) or a 2-cell Li-ion
  pack (7.4 V). A 470 to 1000 uF capacitor across the servo supply, close to the servos, covers the short
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
  where it was expected and what is past it fits (line after a MID crossing, none after a column end).
- **a black mark beside the line** (dirt, a piece of tape): a "crossing" that did not light both outer
  sensors, well before the next crossing is due, is not counted (`not counted: a mark beside the line?`).
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

**It does not stop any more (`STOP_ON_FAULT 0` in `calibration.h`, the default now).** When one of the
faults below happens in the mission, the robot prints `FAULT n: ...` and `(STOP_ON_FAULT 0: it goes on)`
on Serial, notes it once in the run log, and carries on with the route. A lost line is searched for again
and again (a little on, then the swing left and right) until it is found. So a glitch no longer ends the
run; but after a real mistake (a wrong count) the robot may wander, so pick it up when it is clearly lost.
Set `STOP_ON_FAULT` to 1 to make it stop and blink the fault number, as below. The calibration still stops.

| blinks | meaning | look at |
|---|---|---|
| 1 | line lost and not found again | sensor height, lighting, `SENS_ON_PCT` |
| 2 | on a "crossing" for more than 1 s (driving along a line), or the whole bar sees black for too long | the turn before it; sensor levels |
| 3 | half crossing: on the wrong line | the 180 before it |
| 4 | a turn or an end was missed | `CELL_CM`, `ROW_CM`, battery |
| 5 | no line after a MID crossing | the count is off |
| 6 | a turn never found its line | dead band, battery, `TURN_SP` |
| 7 | no line under the bar after a turn | turn speed, calibration |
| 8 | no line ahead where a MID crossing should be | the count is off |
| 9 | no column end found, even at the next crossing | `ROW_CM`, the count |
| 10 | a 90 degree turn passed a line first | turn speed too high |
| 11 | a 180 took far too long or too short | calibration |
| 12 | no crossing for far too long: the wheels are not really turning | motor switch, battery, only USB power |

A **brown-out** (the battery could not supply the servos and motors) makes the Nano restart. The slides'
code then opened the gripper and drove off as if at the start; v2 stopped and blinked fast. v3 prints
`RESTARTED IN THE MIDDLE OF A RUN`, then `MISSION: carrying on at n=...`, and carries on from the count it
kept, with the servos where they were (see "What v3 does differently"). The run log shows
`RESTARTED here` at that point, and the record goes on after it. (The robot keeps a mark and the count in
memory while the mission runs; switching the power off clears it, a restart does not.) Lifted when it
restarts, or restarted by the reset button or USB, it waits for a new start instead. A fresh battery and a
470 to 1000 uF capacitor across the servo supply make restarts rarer.

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
| on | mission | line OK: it goes after 1.5 s (5 s after a run that did not finish) |
| on | calibrate | line seen: starts in 3 s, then stays on while it calibrates |
| on 0.9 s, off 0.1 s | calibrate | finished and saved: plug in USB to read the report |
| fast, 5 times a second | calibrate | stopped with a problem: plug in USB to read the report |
| on 1 s, off 1 s | calibrate | calibrated before: lift it and put it down to calibrate again, or upload `MODE_MISSION` |
| on 1 s, off 1 s | mission | DONE |
| 1 to 12 short blinks, then a pause | mission | a fault, only with `STOP_ON_FAULT 1`: see "When it stops by itself" |
| 2 times a second | sensor check | waiting for Enter (or 20 s) |
| 2 times a second | turn check | waiting for Enter (or 30 s) |
| on 0.5 s, off 0.5 s | turn check | waiting: no good line under the middle of the bar |
| on 1 s, off 1 s | turn check | finished: the report is printed, Enter runs it again |

## Testing without the robot

`sim/` runs this exact firmware in a simulated field:

```
./sim/e33_test.sh
./sim/e33_test.sh "--trimL=0.75 --trimLr=0.6" "--overhang=3"
VERBOSE=1 ./sim/e33_test.sh "--trimL=0.75"
```

Each condition runs the mission without calibration, then the calibration followed by the mission.
By default the simulator is ideal (no slipping, no momentum, a perfect battery). `--real` adds motors
that take time to speed up and slow down, rolling after a brake, tyre scrub in spins, a turning point that
wanders, and the battery sagging while the servos work (see `./sim/sim.exe --help`). Both show that the logic
is right, not that the real robot will be.

The simulator also has a simple supply model for the POWER lines: a battery (`--vbat`, `--rint`), the
regulator's dropout, and the currents of the Nano, the motors and the servos. By default it is a fresh
PP3 and VCC stays at 5.00 V; `--real` is a half-used one (8.2 V, 2.5 ohm): VCC dips to about 4.94 V
while a servo moves. `--vbat=7.6 --rint=3` gives the warnings (and the mission still completes);
`--vbat=6.0 --rint=4` browns out at the first pick: the run ends there, and the restart build
(`-DSIM_RUNMARK=0xE33A55C3UL`) shows what the Nano does next.

**Restarts in the middle of a run (v3).** A simulator with `--reset` (for example
`--reset=servo:1:300`, `--reset=spin:2:400`, `--reset=n:12:100`, `--resetcause=2` for the reset button)
restarts the firmware and keeps its memory, as the Nano does. Build it with the memory v3 keeps:
`-DSIM_NOINIT_VARS='X(runMark) X(keep) X(gripNow) X(armNow)'`. `--lift=0.6` gives the darker bar while
speeding up from standing.

**Route check.** `PATHCHECK=1 ./sim/e33_test.sh` (and the same with `DEFS="-DROUTE_ORDER=132"`) also
checks every count of each calibrated mission against the route with `sim/path_check.py`: where the bar
was at each count, the heading after each turn, each pick and place. A run can put all three objects in
place for the wrong reason (for example a 180 turned the wrong way round); this catches it.

**Worn tape and ruler errors.** `--erase=X:Y:R` (a round gap), `--wipe=X:Y:HW:HH` (a clean cut),
`--patch=X:Y:HW:HH` (a black mark) and `--row=CM` (the row pitch apart from `--cell`), with X and Y in
cm, C1 MID at (0, 0), east and north positive.

## How far it was tested (simulator)

With calibration, the mission completed in every condition tried, including:
- the left wheel at 45% of the right (yours is about 75%), or the right wheel at 55% of the left
- one wheel weaker going backward than forward
- the battery giving anywhere from 45% to 125% of the speed it had during calibration
- dead bands from 15 to 70
- no line sticking out past C1 / C4, gripper reach 1 to 2 cm off, the robot placed up to 12 cm behind the start or 9 cm past it with route 312, and within 7 cm with route 132
- very noisy, weak, uneven, stuck or partly dead sensors (except a dead sensor in the MIDDLE of the bar)
- the light changed after the calibration: white reading 100 at calibration (black 900) and anywhere from
  20 to 320 in the run. From 350 it failed; the start warning appears from 260.
- `ROW_CM` 5 cm too big or too small (rows 20 to 30 cm apart on the field)
- worn tape: see "Worn tape" for what it tolerates
- every count of every calibrated run checked against the route (`PATHCHECK=1`), both routes

Without calibration it only works when the guesses in `calibration.h` are close to your robot
(left wheel about 75% of the right). Calibrate once.

**v3, against the v3 before these changes (simulator, uncalibrated, as you run it):**
- your robot (`--student`, `--student-slow`, `--student-fast`, and the robot fitted to your 19 real runs),
  both routes, put down 4 cm before to 8 cm past C1: the same result in every case, and the same
  crossings and turns (the slow robot finishes about 15 s sooner: no LED waits);
- the 366 hard conditions of the earlier tests: the same result in every one;
- the darker bar while speeding up from standing (`--lift`, the false crossing your robot saw): from 120
  to 183 of 192 objects; the fitted robot now always finishes, and the "turns 180 again" of your run 1
  no longer happens;
- the Nano restarting at every pick and every place, in every turn and 180 (early, in the middle, late),
  and on the way to every crossing, both routes, also four restarts in one run and with the darker bar:
  every run finished with all three objects (the v3 before: it stopped at the first restart). Lifted when
  it restarts, it does not carry on; restarted by the reset button or USB, it waits for a new start.

## What is not known yet

- None of this has run on the real robot yet. The numbers in `calibration.h` marked as guesses need the ruler and the calibration.
- The simulator's field has lines sticking out 12 cm past C1 and C4 and objects 5 cm past the outer line. Measure yours.
- The PWM frequency fix (D3 made to run at the same 976 Hz as D6) explains a few percent of the left/right difference at most; the calibration handles the rest.
