# robot_basic - the course code, as taught

This is slides 6 to 11 written out, not improved. `robot_countGride/` is the
reworked version; if you want the one that matches what the teacher showed,
it is this one.

| file | comes from |
|---|---|
| `controlLibrary.h` | robot11 pages 3-20, with `followLine` from robot06 p12 and the grid counting from robot07 |
| `pidLibrary.h` | the file from the course Google Drive, unchanged |
| `robot_basic.ino` | robot10's structure, with the route rewritten for the e33 field |

Six slide decks were transcribed independently and cross-checked against this.
What follows is what that turned up.

## If it cannot read the line

The slides hard-code the threshold: `analogRead(sensorPin[i]) >= 500`. 500 is a
guess about where your particular bar sits. If it guesses wrong, the robot reads
no line at all no matter how good the sensors are. Three knobs at the top of
`controlLibrary.h` cover it.

**First, get the numbers.** Upload `robot_test/robot_test.ino`, Serial Monitor at
115200, press `1`, and slide the robot on and off a line. It prints every channel
live and tracks the swing each one has seen.

**Then check the polarity.** Watch one channel as you slide it onto the tape.

- reading goes UP over the tape -> leave `SENSOR_ACTIVE_LOW 0`
- reading goes DOWN over the tape -> set `SENSOR_ACTIVE_LOW 1`

If this is backwards, every pattern is inverted and no threshold will help. It is
the first thing to rule out.

**Then set the threshold.** `LINE_THRESHOLD` goes halfway between the white-mat
reading and the tape reading. If a channel reads 180 over the mat and 640 over
the tape, use 410 - not 500.

**Then watch it work.** Set `SHOW_SENSORS 1` and it prints the pattern and the
raw values while it drives, so you can see what it is actually seeing. Turn it
back off for a real run; it slows the loop down.

If the gap between mat and tape is under about 150 counts, the problem is
physical and no threshold fixes it: bar height (aim 5-10 mm off the surface), a
dirty or dead sensor, a glossy surface, or the bar's own trimpot.

## Two things here are yours, not the slides'

**Servo angles.** The slides use 140/65/149 and 105/90/40. This uses the ones
you measured: grip closed 75, arm down 103, carry 70, high 50.

**`maxSp`.** The slides use 255. This uses 100, because it was still running too
fast. Note that `sp` starts at 50 and climbs by 2 every 10 ms, so it reaches
`maxSp` in well under a second - this is the walking speed, not just a ceiling.
Change the one line at the top of `controlLibrary.h` to go back.

Everything else is the slides.

## Where the slides disagree with themselves

**`round()` in `followLine`.** robot06 p12 - the slide that actually teaches the
function - has `map(round(pidOut),-7,7,-sp,sp)`, with a callout labelling
`round()` as "ฟังก์ปัดจุดทศนิยม". The robot11 summary slide drops it and shows
`map(pidOut,...)`. This file keeps `round()`, because robot06 is the lesson and
robot11 is a recap. Without it the float truncates toward zero and steering is
slightly asymmetric.

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
