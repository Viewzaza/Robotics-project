# Real-robot test kit

Checks any version of the e33 firmware against what really happened on the student's robot
(the log of 19 real runs). Each condition in `sim/real_robot_conds.txt` copies one thing seen in
that log: the fitted robot, where the robot is put down, the stuck A0 sensor and the tilted
white levels, the false crossing after a standing start, the Nano restarting at a pick, a place,
a turn or the 180 after a pick, a tired battery, and worn tape.

## What is in `sim/`

| file | what it is |
|---|---|
| `sim.cpp`, `Arduino.h`, `EEPROM.h`, `Servo.h` | the kit simulator: the forensics simulator (with `--lift` and `--reset`) plus three restart points that do not depend on how a firmware moves its servos: `--reset=grip:K[:MS]`, `release:K[:MS]`, `pickturn:K[:MS]` |
| `real_robot_conds.txt` | the conditions, one per line: name, routes, sim flags, and the reason from the log |
| `real_robot_test.sh` | the test: `SKETCHDIR [ROUTE]` |
| `real_robot_test.py` | what the test runs (build, calibrate, run, read the outputs, print the table) |
| `path_check.py` | checks every count of a run against the route (the "path" column) |

## How to run it

You need `g++` and Python 3 (Git Bash on Windows is fine). From the top folder of the repository (the one that holds `sim/`):

```
./sim/real_robot_test.sh robot_e33_v2            # route 312, the one the student uses
./sim/real_robot_test.sh robot_e33_v3 both       # 312 and 132
./sim/real_robot_test.sh e33_mission both        # adds a calibrated column (e33_calibrate beside it)
./sim/real_robot_test.sh robot_e33_emergency     # one fixed route: 312 only
```

Options (environment variables):

- `ONLY=lift,rst-pick1` runs only those conditions.
- `CAL=1` adds a calibrated column for an all-in-one sketch too (v2, v3: its own MODE_CALIBRATE). `CAL=0` turns it off.
- `OUT=DIR` keeps every run's output there (default `./rrt_out/SKETCH`). Each `.txt` is the firmware's Serial lines and the simulator's events, so you can read exactly what went wrong.
- `J=N` simulator processes at once (default 4). A run with a restart is a chain of processes (the restarted copy runs while the old one waits), and the test counts all of them.
- `CONDS=FILE` another condition list in the same format.

A full run of one sketch on both routes takes under a minute.

## The table

```
condition      score  fault (n)      restart                    path      cal  cal fault (n)
realm            3/3  -              -                          PASS      3/3  -
lift             0/3  F5 n=9         -                          n=7       0/3  F5 n=9
rst-pick1        0/3  -              halted@5                   -         0/3  -
```

- **score**: objects on their targets, uncalibrated (the student runs uncalibrated).
- **fault (n)**: the first fault the firmware printed and the count it was at (`F5 n=9`). Without a printed fault: `no start` (it never drove), `stuck n=K`, `time cap n=K` (still driving at the end of the simulated time), `finished n=42` (it counted to the end but the objects are in the wrong places), `brown-out n=K`. A dash when it placed all three, or when it stopped on purpose after a restart. A firmware that carries on after a fault (v3) still shows its first fault.
- **restart**: what the firmware did after each restart in the middle of the run, and the count it was at:
  `halted@5` it did not drive again; `resumed@5` it set off again with its count kept;
  `restarted@5` it set off from count 0 as if at the start (at count 0 or 1 that is the same thing);
  `loop@5` it browned out again and again until the simulator ended the run; `x2` means two restarts
  came one after the other before it drove.
- **path**: `PASS`; `n=K` = the first count made at a wrong crossing, or a pick or place that took nothing;
  `~n=K` = only a pose outside the check's tolerances (for example a timed 180 that ends 3 cm to the side);
  a dash = it only stopped early (the fault column says where).
- **cal**, **cal fault**: the same run after the sketch's calibration on that condition's robot.
  The calibration run leaves out the events of the mission run (restarts, the false crossing, tape
  gaps, the start position, light drift); a `*` means that calibration saved nothing.
- **TOTAL**: objects placed out of 3 per condition, and how many conditions placed all three.

## The conditions

All of them use the robot fitted to the log (`REALM` in the file) put down with the wheels 5 cm past
C1 (`--startx=5`), unless the condition is about the robot model or the placement. The reason for each
is on its line in `real_robot_conds.txt`. Groups:

- the robot: `realm`, `student`, `student-slow`, `student-fast`
- placement: `place-x-4`, `place-x0`, `place-x8`
- sensors: `a0-stuck` (A0 about 640 on white), `a0-drift`, `tilt-run4`, `tilt-run5`, `tilt-run16` (the real white levels of those runs)
- the false crossing after a standing start: `lift` (the strength that reproduced run 1), `lift-strong`, `lift-tilt4`
- restarts: `rst-turn1`, `rst-pick1`, `rst-pick180`, `rst-place1`, `rst-twice`
- battery: `tired` (runs 12 and 14), `bor-servo` (a servo moving under load pulls the 5 V under the reset level)
- worn tape: `gap-*` holes, `mark-*` black marks beside the line

To add a condition, add a line. `@NAME = flags` defines a short name that `$NAME` uses later.

## How a restart works in the simulator

`--reset=...` (or a brown-out with `--bor=restart`) stops the robot where it is and starts the
firmware again from `setup()` in a new copy of the simulator, with the same clock, pose, objects
(an object stays in the jaws until the firmware opens them), the EEPROM, and the firmware's
`.noinit` variables. The test finds those variables itself: every variable declared with
`__attribute__((section(".noinit")))` or a `NOINIT` macro, except `resetFlags` and `resetR2`. For
`runMark` alone nothing is needed; for more (V3's `keep`, `gripNow`, `armNow`) the test builds with
`-DSIM_NOINIT_VARS='X(runMark) X(keep) ...'`. The restarted firmware reads the reset cause
"power-on", as the real restarts in the log did (`--resetcause=N` changes that).

## Byte-identical to the plain simulator

```
./sim/identical.sh BASE_SIM_DIR SKETCH_ROOT            # SKETCH_ROOT holds e33_mission/ and e33_calibrate/
WITH_NEW=1 ./sim/identical.sh FORENSICS_SIM_DIR SKETCH_ROOT   # also with --lift, --reset, --bor=restart
```

Results when this kit was made: against the plain simulator 80 of 80 outputs identical; against the
forensics simulator, with the lift, restart and brown-out conditions too, 110 of 110 identical.

## Results when the kit was made

`results/` holds the tables of: `v2.txt` (robot_e33_v2, the student's working version),
`v3r18.txt` (robot_e33_v3 as in r18), `mi.txt` (e33_mission from r16/base, with the calibrated
column), `em.txt` (robot_e33_emergency as it was then) and `v3wip.txt` (the V3 with restart
carry-on while it was still being made, r19/v3/tree; the final one was not out yet).
`$S` in them is the shared scratch folder.

## Limits

- The emergency sketch has one route, so `132` is skipped for it.
- The battery model only gives the firmware's VCC reading and the brown-out; the motors do not
  slow down with the battery (that is `--vmax` and `--sag`).
- After a restart that resumed, the path check usually flags the count where the restart hit (the
  robot is not where an uninterrupted run would be at that moment); the score is what counts there.
- The path check knows the routes of the e33 sketches (counts 0..42 for 312, 0..33 for 132). A
  sketch that counts differently gets `n=K` or `~n=K` in that column even when it placed the objects.
