# robot_e33: the e33 mission, built on the course slides

This is the firmware for the e33 worksheet (3 objects, 3 targets, 4 columns x 3 lines).
It keeps the slides' structure (robot04 to robot11): `beginFnc()`, `followLine()` with
`pidFNC(errorInput,0,1,0,0.7)`, `countGrid()`, the `switch(numGride)` from robot10, and the
helpers `turn90()`, `keep_item()` and `place_item()`. Every line that differs from the slides
is marked `[E33]` in the code, with the reason.

## The files

| file | what it is | edit it? |
|---|---|---|
| `calibration.h` | your robot and field numbers, the mode switch, the CAL VALUES | **yes, only this one** |
| `robot_e33.ino` | the route (robot10 style) and the three helpers | no |
| `controlLibrary.h` | the slides' library, with the [E33] changes | no |
| `pidLibrary.h` | the slides' PID, gains unchanged, two safety guards | no |
| `calibrationMode.h` | automatic calibration, sensor meter, motor check | no |

Open the Serial Monitor at **115200** baud to see what the robot is doing.

## Getting it running (about 20 minutes)

1. **Measure with a ruler** and put the numbers in `calibration.h`:
   `CELL_CM` (C1 to C4 along MID, divided by 3), `ROW_CM` (MID to TOP), `TAPE_W_CM`,
   `GRIP_REACH_CM` (wheel axle to the centre of an object held in the gripper),
   `OBJ_BEYOND_CM` and `TGT_BEYOND_CM` (outer line centre to object / target centre).
   `SENSOR_AHEAD_CM` is your 9.5 cm.
2. **Check the gripper angle.** Close the gripper slowly on a real object and note the angle where the
   jaws just touch it. `GRIP_CLOSED` should be 2 to 3 degrees tighter than that, not the "empty and fully
   closed" angle. A servo squeezing much tighter than the object pulls a lot of current from the 9 V battery.
3. **`MODE_METER`** (set `ROBOT_MODE` in `calibration.h`, upload). The robot never moves. Slide it over white, over the
   line and over a crossing. The raw numbers must change clearly (about 200 or more between white and black).
4. **`MODE_MOTOR_CHECK`** with the **wheels off the ground**. Each wheel runs forward and backward in turn.
   If one runs the wrong way, set `LEFT_REVERSED` or `RIGHT_REVERSED` to 1.
5. **`MODE_CALIBRATE`** with a **fresh battery**. Put the robot at the start (wheels over C1, on MID,
   facing east), switch on and step back. After 3 s it calibrates itself for about 90 s and stops back at
   the start. The LED stays on with short gaps when it finished, or blinks fast if it stopped with a problem.
   Then plug in USB and open the Serial Monitor: the Nano restarts and prints the saved report and the
   CAL VALUES block again. To calibrate again, lift the robot and put it down.
6. **`MODE_MISSION`**. Put the robot at the start and switch on. It waits, standing still, until it has seen
   the line steadily for 1.5 s, then goes. No buttons. It never makes a calibration movement at the start.
   (One exception, without any movement: if the sensors were never calibrated and the default levels cannot
   see the line properly, for example a bar whose black reads low, whose white reads above the default
   level, or which reads black LOW, it measures the levels from the line under the bar while standing
   still, and says so.)

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

The 180 at the end of a column is where the old code failed. There were four reasons:

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

## When it stops by itself (the LED on D13 blinks the number)

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

A **brown-out** (the battery could not supply the servos and motors) makes the Nano restart. The old code
then opened the gripper and drove off as if at the start. Now the robot recognises a restart in the middle of
a run, prints `RESTARTED IN THE MIDDLE OF A RUN`, blinks fast and does not drive. (The usual Nano bootloader
wipes the chip's own reset record, so the robot also keeps a mark in memory while the mission runs;
switching the power off clears it, a restart does not.) Fit a fresh battery, put the robot at the start and
press reset. A 470 to 1000 uF capacitor across the servo supply also helps. The same happens if you press
reset or plug in USB during a run: press reset once more to start again.

## Testing without the robot

`sim/` runs this exact firmware in a simulated field:

```
./sim/e33_test.sh
./sim/e33_test.sh "--trimL=0.75 --trimLr=0.6" "--overhang=3"
VERBOSE=1 ./sim/e33_test.sh "--trimL=0.75"
```

Each condition runs the mission without calibration, then the calibration followed by the mission.
The simulator is ideal in many ways (no slipping, no momentum, a perfect battery), so it shows that the
logic is right, not that the real robot will be.

## How far it was tested (simulator)

With calibration, the mission completed in every condition tried, including:
- the left wheel at 45% of the right (yours is about 75%), or the right wheel at 55% of the left
- one wheel weaker going backward than forward
- the battery giving anywhere from 45% to 125% of the speed it had during calibration
- dead bands from 15 to 70
- no line sticking out past C1 / C4, gripper reach 1 to 2 cm off, the robot placed up to 12 cm off
- very noisy, weak, uneven, stuck or partly dead sensors (except a dead sensor in the MIDDLE of the bar)

Without calibration it only works when the guesses in `calibration.h` are close to your robot
(left wheel about 75% of the right). Calibrate once.

## What is not known yet

- None of this has run on the real robot yet. The numbers in `calibration.h` marked as guesses need the ruler and the calibration.
- The simulator's field has lines sticking out 12 cm past C1 and C4 and objects 5 cm past the outer line. Measure yours.
- The PWM frequency fix (D3 made to run at the same 976 Hz as D6) explains a few percent of the left/right difference at most; the calibration handles the rest.
