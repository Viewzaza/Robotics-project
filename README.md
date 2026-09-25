# robot_countGride  Robot Mission

Arduino line-following robot with a gripper arm.

**Simulator result: 315 of 315 test conditions complete all three placements.
The original firmware completed 0 of 315.**

## Layout

| File | What it is |
|---|---|
| `robot_countGride/robot_countGride.ino` | Mission route, the turn/pick/place actions, calibration mode |
| `robot_countGride/config.h` | Every tunable constant, each marked with how to measure it |
| `robot_countGride/hal.h` | Sensing, odometry, motion primitives, AVR fast paths |
| `robot_countGride/controlLibrary.h` | The API the slides use, rebuilt on `hal.h` |
| `robot_countGride/pidLibrary.h` | Course PID helper, with three guards added |
| `robot_motor_test/robot_motor_test.ino` | Upload-and-watch motor check, no serial input needed |
| `robot_test/robot_test.ino` | Standalone bench test rig: sensors, motors, servos, calibration |
| `sim/` | Host-side simulator - field, kinematics, sensor bar, gripper |

Open `robot_countGride/robot_countGride.ino` in the Arduino IDE.
Board: **Arduino Nano** (a DIP Uno has no physical A6/A7, so two sensors would
read garbage). Needs the `Servo` library. Build: 14.2 KB flash, 439 B SRAM.

## The 9.5 cm rule

This is the measurement everything else hangs off.

The sensor bar sits **9.5 cm ahead of the wheel axle**, and the wheel axle is
the point the robot pivots about. So at the instant the bar reports a crossing,
the pivot centre is still 9.5 cm short of it:

```
        bar            axle
         |               |
    -----X---------------o-----> travel
         ^               ^
    sees the line   but turns HERE, 9.5 cm too early
```

Turning there pivots about a point 9.5 cm before the intersection and throws the
robot off the line. Every manoeuvre therefore advances the *measured remaining*
distance first, at a fixed low duty so the distance is repeatable, and only then
pivots.

The original code approximated this with `moveFor(); delay(50);` at whatever
speed the ramp had reached - `sp` could be anywhere from 50 to 255, about a 5x
spread in how far it actually went.

## Field and mission

```
        o1                                  o3
   -----+-------+-------+-------+-----      TOP
        |       |       |       |
   --[R]>-------+-------+-------+-----      MID
        |       |       |       |
   -----+-------+-------+-------+-----      BOT
        o2      x3      x2      x1
        C1      C2      C3      C4
```

`o` = object, `x` = drop position, `[R]>` = start (middle line at C1, facing right).

| Object | At | Goes to |
|---|---|---|
| o1 | top of C1 | x1 = bottom of C4 |
| o2 | bottom of C1 | x2 = bottom of C3 |
| o3 | top of C4 | x3 = bottom of C2 |

The worksheet lets you start with any object. This runs **3 → 1 → 2**: the robot
starts facing right so it takes the right-most object first, and the route never
drives along the bottom line, so it cannot knock over anything already placed.

### Route

`countGrid()` adds 1 per crossing. The intersection the robot starts on is not
counted. Every action case adds 1 of its own, which is why the numbers skip.

| n | Point | Heading | Action |
|---|---|---|---|
| 3 | (C4,MID) | E→N | `turn90("LEFT")` |
| 5 | (C4,TOP) | N→S | `keep_item` - pick o3 |
| 7 | (C4,MID) | S→W | `turn90("RIGHT")` |
| 10 | (C2,MID) | W→S | `turn90("LEFT")` |
| 12 | (C2,BOT) | S→N | `place_item` - drop at x3 |
| 14 | (C2,MID) | N→W | `turn90("LEFT")` |
| 16 | (C1,MID) | W→N | `turn90("RIGHT")` |
| 18 | (C1,TOP) | N→S | `keep_item` - pick o1 |
| 20 | (C1,MID) | S→E | `turn90("LEFT")` |
| 24 | (C4,MID) | E→S | `turn90("RIGHT")` |
| 26 | (C4,BOT) | S→N | `place_item` - drop at x1 |
| 28 | (C4,MID) | N→W | `turn90("LEFT")` |
| 32 | (C1,MID) | W→S | `turn90("LEFT")` |
| 34 | (C1,BOT) | S→N | `keep_item` - pick o2 |
| 36 | (C1,MID) | N→E | `turn90("RIGHT")` |
| 39 | (C3,MID) | E→S | `turn90("RIGHT")` |
| 41 | (C3,BOT) | S | `place_item("STOP")` - drop at x2 |
| 42 | - | - | done → `stopRobot()` |

Cases 1, 2, 9, 22, 23, 30, 31, 38 have no entry: those crossings are driven
straight through by the `default:` branch.

## Bring-up order

> **The numbers shipped in `config.h` are not your robot's numbers.**
> `CM_PER_S_AT_CAL`, `MS_PER_90DEG` and `DUTY_DEADBAND` were set to match the
> simulated robot so the logic could be tested. `GRIP_REACH_CM`, `CELL_CM` and
> `OBJECT_BEYOND_CM` are guesses. Only `SENSOR_AHEAD_CM` (9.5) is measured from
> your actual robot. Flash it, then calibrate, then re-flash. Running the full
> mission on the shipped numbers will send the robot to the wrong places at
> speed.

Do these in order. Do not skip to the mission.

1. **Upload and open the serial monitor at 115200.** You should see `ready`.
2. **Calibrate.** Hold the start button on D9 while powering up (button to GND;
   if you have no button, short D9 to GND at power-up). The robot sweeps across
   a line to learn each sensor's black and white levels, then drives straight
   for 3 s, then spins. Measure the straight run with a ruler, divide by 3, and
   put that in `CM_PER_S_AT_CAL`. Check the printed per-channel `lo`/`hi`: any
   channel flagged `WEAK` is dirty, loose, or dead.
3. **Check the turn.** `MS_PER_90DEG` must be close. Put the robot on a cross
   and watch one `turn90`. If it consistently stops short or long, scale the
   constant by the same ratio.
4. **One leg.** Start it on the middle line and confirm it counts `n=1`, `n=2`
   at the right crossings in the telemetry.
5. **Full mission.**

## Tuning

Everything lives in `config.h`. The ones that matter most, in order:

| Constant | Meaning |
|---|---|
| `LINE_WIDTH_CM` | Tape width. **The robot's only ruler.** It corrects its own odometry against this while it drives, so if this is wrong, the correction is wrong too. Lay a ruler across a line. |
| `CM_PER_S_AT_CAL` | Forward speed at `DUTY_CAL`. Sets every distance, but the robot now repairs errors in it from `LINE_WIDTH_CM`, so it no longer has to be perfect. |
| `MS_PER_90DEG` | Pivot time for 90°. Sets how accurate turns are. |
| `SENSOR_AHEAD_CM` | 9.5 - re-measure if you move the bar. |
| `GRIP_REACH_CM` | Axle to jaws. Wrong by 1 cm and pick-ups start missing. |
| `CELL_CM` | Grid pitch, used to slow down before the next crossing. |
| `TRIM_R_PCT` | Raise/lower if the robot curves while driving straight. |
| `DUTY_CRUISE` | Walking pace. Lower is more reliable; raise it once counting holds. |
| `ADC_PRESCALER` | Lower it (0x06) if the sensor mask still looks unstable. |

### The bench test rig

Upload `robot_test/robot_test.ino` instead of the mission sketch. It shares no
code with the firmware on purpose - it has to work when the firmware does not.
Open the Serial Monitor at **115200** with line ending set to **Newline**, then
type a number. Nothing moves until you ask.

| | test | gives you |
|---|---|---|
| `1` | sensors | live raw values and the swing each channel has seen |
| `2` | motors | each wheel forward then back, to check the wiring |
| `3` | servos | type `g75` / `a103` to find your own angles |
| `4` | speed | 3 s straight run -> `CM_PER_S_AT_CAL` |
| `5` | pivot | 4 turns -> `MS_PER_90DEG` |
| `6` | deadband | ramps the duty until the wheels start -> `DUTY_DEADBAND` |
| `0` | stop | brakes everything |

The motor driver is left disabled until a test needs it, and disabled again
afterwards. Type `0` at any point to stop.

**Wiring the motors.** There is no correct order for AO1/AO2 or BO1/BO2 - two
plain wires, and which way round they go only decides which way that wheel
turns. Wire left to AO1/AO2 and right to BO1/BO2 either way, then check it.

For just the motors there is `robot_motor_test/robot_motor_test.ino`: upload it,
put the robot on a book, and watch. It loops A forward, A back, B forward,
B back, both forward, both back, spin - announcing each over serial at 115200,
with no typing. Whichever wheel runs backwards, set `INVERT_LEFT` or
`INVERT_RIGHT` to 1 in `config.h`, or just swap that motor's two wires. No
unsoldering needed if you use the flag.

**Reading the sensor test.** You want each channel low over the mat and high
over the tape, with at least ~150 counts between - ideally 400+. A small swing
is physical, not a threshold problem: bar height (aim 5-10 mm), a dirty sensor,
a glossy surface, or the bar's own trimpot. The mission sketch also has
`DIAG_SENSORS` and `DIAG_MOTORS` in `config.h` for the same checks in place.

## What changed from the slides, and why

Each of these fixes a specific way the original ends the run early.

**Sensing.** The fixed `analogRead() >= 500` threshold was applied to all eight
channels. LED brightness and phototransistor gain vary channel to channel, and
both drift with ambient light and with the battery sagging. Now each channel has
its own calibrated threshold with hysteresis, stored in EEPROM.

**Line position.** `getErrorInput()` was a chain of 15 exact string matches;
anything not in the table returned `100` and `followLine()` then commanded the
motors *not at all*, leaving them on the previous PWM pair. Six physically
normal patterns - a single lit sensor anywhere but the extreme ends - were
missing from that table. Replaced with a weighted centroid, which is continuous
and cannot fall off the end of a lookup.

**Junction detection.** `checkGrid()` accepted 9 exact patterns out of the 163
that have four or more sensors lit. One dusty channel turns `11111111` into
`11110111`, the crossing is missed, and every later route step fires at the
wrong place. Now it counts bits.

**`countGrid()` deadlock.** The original ended with `while(checkGrid());` - an
unbounded wait that can only finish if the robot keeps moving. Called after
`stopRobot()`, which *brakes* both motors, it could never exit. Re-arming is now
by distance travelled and nothing blocks.

**Turns.** `turnRight90()` spun until the pattern was one of three exact values
and never stopped the motors itself, relying on momentum and caller latency to
centre. If the line swept past between samples it spun forever. Now: open-loop
through 85% of the expected angle, then close the loop on the line being near
the middle of the bar, with a hard stop at 118%. The window is deliberately
narrow - the bar traces a circle of radius 9.5 cm about the pivot, so it clips
*every* line at the junction, not just the one being looked for. Widening the
window makes turns end on the wrong line.

**Self-calibrating odometry.** `CM_PER_S_AT_CAL` is measured once, on one
battery, at one moment. A fresh cell or a sagging one moves the true speed by
±30%, and that scales every `advanceCm()` *and* every timed pivot by the same
factor, so a 90° turn becomes a 66° turn and the robot leaves the field. The
robot now fixes this itself while it drives. The tape is the one absolute length
it can see: while the bar sweeps a crossing at right angles every sensor is lit,
and the ground covered during that window is one tape width. Comparing that with
what the odometer thought it covered gives the scale error directly, with no
encoder. Measured on the sweep this is worth 274/315 → 314/315, because it
repairs the exact failure a wrong speed calibration causes.

**The 180° turnaround anchor.** After a 90° pivot the leftover advance error
becomes a lateral offset the line follower absorbs. After a 180° turnaround the
new heading is anti-parallel, so the error stays longitudinal and doubles:
the bar ends up `2 × 9.5 − advance` into the next cell, not 9.5. Anchoring it at
9.5 told `followLine()` there were about 7 cm more to run than there were, so it
never decelerated and hit the next crossing at cruise. On a 20 cm grid that next
line is only ~3.5 cm away.

**PID.** `dt` could be zero when two calls landed in the same millisecond, and
the derivative term divided by it. Also guarded: the first call after
`clearPid()`, and the output range, since `map()` does not clamp.

**Line lost.** Previously the motors kept their last command and the robot drove
away. Now it sweeps to re-acquire, with a timeout.

**Power.** Servos are stepped a few degrees at a time instead of commanded in
one jump. A single 9 V alkaline cannot supply an unrestrained servo sweep
without dipping the shared 5 V rail, which resets the Nano mid-mission. The
mission step is also checkpointed to EEPROM so a reset resumes rather than
restarting with objects already moved.

**SRAM.** `getSensor()` built an Arduino `String` by repeated `+=` on every call,
many times per second for minutes, on a 2 KB part. Sensor state is now a
`uint8_t`. Static RAM went *down* from 522 B to 428 B while the firmware grew.

## Using the Nano's full performance

The control loop was dominated by the eight `analogRead()` calls. Arduino leaves
the ADC prescaler at 128, giving a 125 kHz ADC clock and ~104 µs per conversion
- **832 µs just to read the bar**, before any thinking.

| Change | Effect |
|---|---|
| ADC prescaler 128 → 16 (1 MHz clock) | 8 channels: 832 µs → ~110 µs |
| Direction pins via `PORTB`/`PORTD` | ~50 cycles → 2 cycles per pin |
| Timer2 prescaler 64 → 32 | D3 PWM 490 Hz → 980 Hz, matching D6 |

The two motors were being driven at *different* PWM frequencies, because D3 is
on Timer2 and D6 on Timer0 with different Arduino defaults. They now match.

Faster sampling is not just tidiness: at 22 cm/s the robot moves 0.26 mm between
samples at the old rate. Sampling granularity is what limits how precisely a
junction crossing can be timed, and the junction time is what the whole
dead-reckoned route depends on.

All of it is behind `#if defined(__AVR__)`, so the simulator builds and exercises
the identical logic through the portable path.

## The simulator

```bash
g++ -O2 -std=c++14 -I sim -o sim/sim.exe sim/sim.cpp
```

```bash
./sim/sim.exe --secs=200
```

Models the field, differential-drive kinematics, the bar at its 9.5 cm offset,
and the gripper, then runs the real `setup()`/`loop()` against it. Prints a
trace and `score N/3`.

```bash
./sim/sweep.sh
```

Runs 315 conditions varying motor mismatch (0.90–1.10), battery state
(25–50 cm/s), grid pitch (20–30 cm) and gripper reach (11–13 cm), and reports
how many complete the mission. This is the regression test - run it after any
change to `robot_countGride/`.

### What the simulator does not model

Read this before trusting a number from it. It has a perfectly linear motor
model, no wheel slip, no sensor noise, no battery droop *over time*, no chassis
mass or momentum, and a gripper that either reaches the object or does not. It
is good for catching logic and geometry errors - which is what it was built for,
and it caught several - and it is **not** evidence that the robot works. Nothing
here has been run on hardware.

One measured example of why that distinction matters: making `advanceCm()` steer
on the centroid instead of driving blind *sounds* obviously right, and it took
the sweep from 267/315 down to 151/315, because within 9.5 cm of a junction the
bar clips the crossing line and the correction steers into it. That change was
reverted.

## Known limits

- **Not tested on hardware.** Every number above is from the simulator or the
  compiler.
- `CELL_CM`, `GRIP_REACH_CM` and `OBJECT_BEYOND_CM` are guesses until measured on
  the real field. The remaining 48 sweep failures cluster where these are wrong.
- There is no re-localisation. `numGride` is still the only state, so a miscount
  that survives the debounce still derails the rest of the run.
- `ENABLE_WATCHDOG` is off by default: a stock Nano with an old Optiboot
  bootloader reset-loops on a watchdog reset. Only turn it on if you know your
  bootloader is current.
