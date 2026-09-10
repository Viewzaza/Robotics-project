# robot_countGride  Robot Mission 

Arduino sketch for a line-following robot with a gripper arm

## Files

| File | Source |
|---|---|
| `robot_countGride/robot_countGride.ino` | Mission program (written for the field in `e33.pdf`) |
| `robot_countGride/controlLibrary.h` | Transcribed from `robot11.pdf`, pages 3–20 |
| `robot_countGride/pidLibrary.h` | Provided with the course material |

Open `robot_countGride/robot_countGride.ino` in the Arduino IDE.

## Board

Must be an **Arduino Nano / Pro Mini**, because the code reads eight sensors on
`A0`–`A7`. A DIP Uno has no physical A6/A7 pins — it compiles, but those two
readings are garbage. The `Servo` library is required.

## Field and mission

```
        o1                                  o3
   ─────┬───────┬───────┬───────┬─────      TOP
        │       │       │       │
   ──[R]>───────┼───────┼───────┼─────      MID
        │       │       │       │
   ─────┴───────┴───────┴───────┴─────      BOT
        o2      x3      x2      x1
        C1      C2      C3      C4
```

`o` = object, `x` = drop position, `[R]>` = start position (on the middle line
at C1, facing right).

| Object | Located at | Must be placed at |
|---|---|---|
| o1 | top of C1 | x1 = bottom of C4 |
| o2 | bottom of C1 | x2 = bottom of C3 |
| o3 | top of C4 | x3 = bottom of C2 |

The worksheet allows starting with any object number. This program runs the
order **3 → 1 → 2**, because the robot starts facing right and o3 is the
right-most object.

## How numGride is counted

* `countGrid()` adds 1 every time a crossing line is detected.
* The intersection the robot starts on, `(C1,MID)`, is **not** counted — the
  first crossing it meets is 1.
* Every `case` that performs an action also does `numGride++`, so the case
  numbers skip.

(Same convention as the worked examples in `robot08` pages 10–11 and `robot10`.)

## Route table

| n | Point | Heading before → after | Action |
|---|---|---|---|
| 1 | (C2,MID) | E | followLine |
| 2 | (C3,MID) | E | followLine |
| 3 | (C4,MID) | E → N | `turn90("LEFT")` |
| 5 | (C4,TOP) | N → S | `keep_item` — pick object 3 |
| 7 | (C4,MID) | S → W | `turn90("RIGHT")` |
| 9 | (C3,MID) | W | followLine |
| 10 | (C2,MID) | W → S | `turn90("LEFT")` |
| 12 | (C2,BOT) | S → N | `place_item` — drop object 3 at x3 |
| 14 | (C2,MID) | N → W | `turn90("LEFT")` |
| 16 | (C1,MID) | W → N | `turn90("RIGHT")` |
| 18 | (C1,TOP) | N → S | `keep_item` — pick object 1 |
| 20 | (C1,MID) | S → E | `turn90("LEFT")` |
| 22 | (C2,MID) | E | followLine |
| 23 | (C3,MID) | E | followLine |
| 24 | (C4,MID) | E → S | `turn90("RIGHT")` |
| 26 | (C4,BOT) | S → N | `place_item` — drop object 1 at x1 |
| 28 | (C4,MID) | N → W | `turn90("LEFT")` |
| 30 | (C3,MID) | W | followLine |
| 31 | (C2,MID) | W | followLine |
| 32 | (C1,MID) | W → S | `turn90("LEFT")` |
| 34 | (C1,BOT) | S → N | `keep_item` — pick object 2 |
| 36 | (C1,MID) | N → E | `turn90("RIGHT")` |
| 38 | (C2,MID) | E | followLine |
| 39 | (C3,MID) | E → S | `turn90("RIGHT")` |
| 41 | (C3,BOT) | S | `place_item("STOP")` — drop object 2 at x2 |
| 42 | — | — | mission complete → `stopRobot()` |

The route never travels along the bottom line, so it cannot disturb objects it
has already placed.

## Tuning on the field

Edit these at the top of the `.ino`:

| Value | Meaning |
|---|---|
| `NUDGE_TURN` (50) | ms driven past an intersection before turning — adjust if the robot turns too early or too late |
| `NUDGE_PICK` (50) | ms driven toward the object before gripping — adjust if it misses the object or pushes it away |
| `NUDGE_PLACE` (30) | ms driven forward before releasing the object |

Other values that usually need tuning, in `controlLibrary.h`:

* `analogRead(...) >= 500` in `getSensor()` — the black/white threshold
* `maxSp = 255` — lower it (e.g. 150) if the robot is too fast and loses the
  line or overshoots intersections
* `pidFNC(errorInput, 0, 1, 0, 0.7)` in `followLine()` — kp / ki / kd
* servo angles in `keepup_object()`, `put_object()`, `arm_over_head()`

## Additions beyond the slides

`leaveGride()` in the `.ino`. After a turn completes, the robot can still be
straddling the intersection it just used. Left alone, `countGrid()` counts that
same intersection twice, and its internal `while(checkGrid())` never exits
because `stopRobot()` has already braked the wheels — the robot freezes in the
middle of the field. This function creeps forward slowly until the crossing is
clear, then hands control back to line following.

`followLine()` also keeps the `round()` from `robot08` page 4:
`map(round(pidOut), -7, 7, -sp, sp)`. The summary slide in `robot11` drops it,
but without `round()` the float truncates toward zero and steering becomes
asymmetric.
