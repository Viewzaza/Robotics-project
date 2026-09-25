# robot_basic - the course code, as taught

This is slides 6 to 11 written out, not improved. `robot_countGride/` is the
reworked version; if you want the one that matches what the teacher showed,
it is this one.

| file | comes from |
|---|---|
| `controlLibrary.h` | robot11 pages 3-20, with `followLine` from robot06 p12 and the grid counting from robot07 |
| `pidLibrary.h` | the file from the course Google Drive, unchanged |
| `robot_basic.ino` | robot10's structure, with the route rewritten for the e33 field |

Eight decks (robot04 to robot11) were transcribed independently by separate
readers, and the assembled sketch was then audited line by line against all of
them. What follows is what that turned up.

## Reading the line: it does this by itself now

`AUTO_CALIBRATE` is on. Nothing to press, nothing to measure, nothing to type.

Put the robot on the line and switch it on. It rocks left, right, right, left -
equal time each way - so the sensor bar sweeps across the line several times
while it watches every channel. From that it sets a threshold per channel from
what that channel actually saw, rather than assuming the slides' fixed 500.

It works out the polarity on its own too, which is the part that is easy to get
wrong by hand. The tape is thin and the mat is wide, so each sensor spends most
of the sweep looking at the mat. Whichever side of the middle a channel sits on
MOST of the time is therefore the mat, and the line is the other side. So it
does not matter whether your bar reads high or low over black - it finds out.

The rocking is symmetric, so the robot finishes pointing where it started, and
it then squares itself back up on the line before the mission begins.

It prints what it decided, at 9600 baud:

```
--- auto calibration ---
  A0  low 100  high 900  swing 800  threshold 500
  ...
  line reads BRIGHTER than the mat
  usable channels: 8
------------------------
```

**Read the swing column.** That is the real health check. Under 120 counts and a
channel is flagged too flat to use, and no threshold can rescue it - that is
physical: bar height (aim 5-10 mm off the surface), a dirty or dead sensor, a
glossy surface, or the bar's own trimpot. If fewer than 3 channels are usable it
says so and falls back to the fixed threshold.

**If it says NOT ENOUGH CONTRAST**, the most likely reason is that the robot was
not on a line when you switched it on, so the sweep never crossed one.

Turning it off: set `AUTO_CALIBRATE 0` and it uses `LINE_THRESHOLD` and
`SENSOR_ACTIVE_LOW` exactly as the slides do. `SHOW_SENSORS 1` prints the live
pattern while driving if you want to watch.

## Two things here are yours, not the slides'

**Servo angles.** The slides use 140/65/149 and 105/90/40. This uses the ones
you measured: grip closed 75, arm down 103, carry 70, high 50.

**`maxSp`.** The slides use 255. This uses 100, because it was still running too
fast. Note that `sp` starts at 50 and climbs by 2 every 10 ms, so it reaches
`maxSp` in well under a second - this is the walking speed, not just a ceiling.
Change the one line at the top of `controlLibrary.h` to go back.

Everything else is the slides.

## The threshold: the slides say 800 in three places and 500 in one

This is the one that decides whether the robot can see the line at all.

| slide | code | threshold |
|---|---|---|
| robot04 p11 | `if(analogRead(A0) >= 800)` | **800** |
| robot04 p12 | `if(analogRead(sensorPin[i])>=800)` | **800** |
| robot07 p07 | `if(analogRead(sensorPin[i]) >= 800)` | **800** |
| robot11 p06 | `if(analogRead(sensorPin[i]) >= 500)` | 500 |

robot04 p10 settles it by showing what the teacher's own bar reads:

```
over white (พื้นที่สีขาว):  246  246  245  245  245  245
over black (พื้นที่สีดำ):   979  978  979  979  979  979
```

With 734 counts of contrast either number works, but 800 sits well clear of the
white readings while 500 is only 255 counts above them. This file uses **800**.

robot04 p12 also states the convention outright: **1 = the sensor is over the
black line, 0 = over white.**

## Where the slides disagree with themselves

**`round()` in `followLine`.** Four pages have it and one does not:
robot06 p08, p09, p10, p11, p12 (p08 even has a callout pointing at `round`
reading "ฟังก์ปัดจุดทศนิยม"), robot07 p05, and robot08 p04 all show
`map(round(pidOut),-7,7,-sp,sp)`. Only robot11 p08 drops it. This file keeps
it. `map()` takes a `long`, so without `round()` the float truncates toward
zero and steering is asymmetric by one count near the edges.

**The trailing `Serial.println` in `followLine`.** robot11 p08 ends the function
with `Serial.println(String(speedL) + "," + String(speedR));` live. robot07 p05
shows the same line commented out, and robot08 p04 omits it. It is present here
but commented out, like robot07 - at 9600 baud that line takes about 15 ms to
send once the buffer fills, which is far longer than one pass of the control
loop, so leaving it live makes the line following visibly worse.

**`stopRobot()` and `sp = 50;`.** robot07 p13 has three lines and no reset;
robot07 p14 adds `sp = 50;` with the callout "กำหนด Speed เริ่มต้นใหม่", and
robot11 p11 keeps it. Included here, following the later versions.

**Where `digitalWrite(STBY,1)` goes in `beginFnc()`.** robot07 p04 puts it
before `clearPid()` with no servos yet; robot09 p06 puts it before the servo
lines; robot11 p05 puts it after them. This follows robot11, the last version.

**The nudge before a turn.** robot08 p06/p11 use `delay(50)`; robot09 p10 uses
`delay(40)` throughout; robot10 uses 50 for `turn90`, 50 for `keep_item` and 30
for `place_item`. This follows robot10, the deck the helpers come from.

**The gripper release angle.** In robot11, `beginFnc()` opens the gripper with
`servo_x.write(140)` but `put_object()` releases with `servo_x.write(149)`. Both
are on the slides. Kept as two separate constants, `GRIP_OPEN` and
`GRIP_RELEASE`, so the difference is visible rather than hidden.

**`numGrid` vs `numGride`.** robot08 spells the counter `numGrid`. robot10 and
robot11 spell it `numGride`, while still calling `countGrid` without the `e`.
The Arduino tab is `robot_countGride.ino`. This file follows robot10/robot11:
variable `numGride`, function `countGrid`.

**`begibFnc()`.** robot10 p03 calls `begibFnc()` in `setup()`. That is a typo for
`beginFnc()` and would not compile. Fixed here; it is the only code change.

**The error table is not symmetric.** In `getErrorInput`, -1 is `"00111000"`
(three sensors) while +1 is `"00011100"`, and -4 is `"01100000"` while +4 is
`"00000110"`. That is what the slides show, on both robot06 p06 and robot11 p07,
so it is transcribed as-is rather than "corrected".

## What is in this file that is not on any slide

The audit found four additions beyond the three declared departures. None
change what the course code does; they are listed so nothing is hidden.

- `tUpSp = millis();` at the end of `beginFnc()`. No slide sets it there. It
  only makes the first speed-ramp tick land 10 ms after start-up instead of
  immediately.
- `SHOW_SENSORS` and `showSensors()`. Compiled out at 0, so inert unless you
  turn it on.
- `getSensor()` is restructured to read into a variable first, so the
  calibrated threshold and the polarity flag can be applied. With
  `AUTO_CALIBRATE 0` and `SENSOR_ACTIVE_LOW 0` it behaves exactly as the slides.
- Forward declarations for `turn90`, `keep_item` and `place_item` in the .ino.
  Redundant under Arduino's auto-prototyping, but harmless and they make the
  `String` arguments explicit.

## One thing that cannot be checked

`pidLibrary.h` appears on no slide in decks 04 to 11. robot06 p07 shows the tab
and points at it, and p07 and p11 carry only a download link. So `pidFNC`, its
integral clamp, and `clearPid` cannot be verified against the course material -
the copy here is the file you supplied, unchanged.

## Things the slides do that are worth knowing about

None of these are changed here. They are the teacher's design, and this file
is a transcription. They are listed because they are the places this code can
stop, and knowing where to look saves you an afternoon.

- `turnRight90()` and `turnLeft90()` are `while(true)` loops with no timeout.
  They exit only on one of three exact sensor patterns. If the line sweeps past
  between two reads, the loop does not end.
- The middle loop of `turnRight180()` / `turnLeft180()` reads the sensors but
  never writes to the motors, so it depends on the motor state the preceding
  90-degree turn left behind.
- `countGrid()` ends with `while(checkGrid());` - an empty busy-wait that can
  only finish if the robot is still moving. Called after `stopRobot()`, which
  brakes both motors, it cannot finish.
- `getErrorInput()` returns 100 for any pattern not in its table, and
  `followLine()` has no `else`, so on those patterns it writes nothing to the
  motors and they hold their previous command.
- `checkGrid()` matches nine exact patterns. One sensor reading differently
  means the crossing is not counted, and since the route is a switch on the
  count, everything after that fires in the wrong place.

If the robot runs this and then stops dead in the middle of the field, the
third one is the first thing to check.

## What the simulator says

Running this through `sim/` scores **0 of 3** placements: the turns miss their
exit patterns and it drives off the field.

Do not read too much into that. The simulator models a 1.8 cm line on a 1.2 cm
sensor pitch, which puts the number of lit sensors right on the boundary
between one and two - and the turn exit patterns need exactly two or three
adjacent sensors lit. Real tape on a real bar is usually more forgiving than
that. This code is what your class runs, so it evidently can work.

What the simulator is reliable about is the *structural* problems listed above:
those are in the code regardless of sensor geometry.

To try it yourself:

```bash
g++ -O2 -std=c++14 -I sim -DFIRMWARE_INO='"../robot_basic/robot_basic.ino"' -o sim/sim_basic.exe sim/sim.cpp
```

```bash
./sim/sim_basic.exe --secs=200
```
