# robot_power_test: why the robot restarts, and how to fix it

This is a small test sketch for the e33 robot. It does not follow a line and does not drive a route. It moves the servos and motors in a fixed order while it measures the 5 V supply, then tells you OK, WEAK or WILL RESTART.

## 1. What happened in your runs

Your robot has one 9 V battery (a PP3) for everything:

- the motors take power straight from the 9 V (through the TB6612),
- a 5 V step-down makes 5 V from the same 9 V for the Nano, the sensor bar and both servos.

Your run log showed this every time:

- standing still, the supply was 5.02 V (fine),
- at every pick and place it dropped to about 4.4 to 4.6 V (the `POWER pick ... min=4.41V LOW` lines),
- the Nano restarted 12 times, mostly right at a servo move or in the turn just after it.

**Why it restarts.** A moving servo takes about 0.3 to 0.7 A for a moment, and a motor that starts or reverses takes even more. A PP3 cannot give that much current. Its voltage collapses under the load, the step-down no longer has enough voltage to make 5 V, and the 5 V line falls. When it falls far enough, the Nano's reset circuit restarts the chip (on a Nano with the usual settings this is at 2.7 V). The program then starts again from the beginning, as if you had just switched it on. That is the "RESTARTED IN THE MIDDLE OF A RUN" message.

**Why it worked better with the USB cable.** The USB cable also feeds the Nano's 5 V line, from the computer, through a diode on the Nano board. The computer's USB port is a strong, steady source. When the battery sags, USB holds the 5 V line up, so the Nano and the servos keep going. But you cannot run the course with a cable attached, and USB does not help the motors (they run from the 9 V). So the cable hid the problem; it did not fix it.

**Why your log says "power-on" and not "BROWN-OUT".** After each restart the reset cause was "power-on". That means the supply fell very low, so the chip started as if it had just been switched on. Some Nano clones have the brown-out detector switched off. This test prints the brown-out setting of your Nano at the start, so you can see which it is. Either way, the cause is the supply.

## 2. How to run the test

You need the Arduino IDE, the USB cable and something to stand the robot on.

1. Open `robot_power_test/robot_power_test.ino` and upload it, with the same board and processor settings you use for robot_e33_v2.
2. Put the robot on a box or a cup so the **wheels are in the air** and the gripper and arm can move freely. Nothing should touch the gripper.

The test always does the same 15 steps (about 25 s with the countdown):

| Step | What moves |
|---|---|
| 0 | nothing (the supply at rest) |
| 1 | the servos switch on and hold their angles |
| 2, 3 | a pick: gripper closes 140 to 75, eases 3 degrees, arm lifts 103 to 70 |
| 4, 5, 6 | a place: arm down to 103, gripper opens to 149, arm high to 50 |
| 7 | both servos back to the start angles |
| 8 | the gripper jumps 140 to 75 and back at full speed (the slides' code moves servos like this) |
| 9, 10 | the motors start a spin, then reverse at once with no stop |
| 11 | motors stop |
| 12, 13 | together: a spin while the gripper closes and the arm lifts, then reverse while the arm goes high (the turn after a pick) |
| 14 | everything back, motors off, servos off |

### Run A: with the USB cable (to watch)

Switch the battery on, plug the USB cable in, open the Serial Monitor at **115200**. The test starts after 10 s (the LED blinks fast; send any key to cancel). Each step prints a line like:

```
step 12  lowest 4.36 V  average 4.81 V  49998 readings, 30% below 4.60 V  WILL RESTART
         together: spin starts, gripper closes, arm lifts
```

This run has the USB help, so it looks better than the real robot.

### Run B: battery only (the real result)

1. Unplug the USB cable. Switch the battery on.
2. Wait. The LED blinks fast for 10 s, stays on during the test, then blinks the verdict over and over:
   - **1 blink**: OK
   - **2 blinks**: WEAK
   - **3 blinks**: WILL RESTART (also when the Nano restarted during the test)
3. Switch the battery off, plug the USB cable in and open the Serial Monitor. The saved results of Run B print first, under `=== SAVED POWER TEST ===`. (Opening the Serial Monitor restarts the Nano, and the test then counts down again: send any key to cancel it, or let it run as a Run A for comparison.)

If the Nano restarts during the test, the next start prints `THE NANO RESTARTED DURING THE POWER TEST`, names the step where it happened, and does not run again (on the box it would only restart again). Press the reset button, or switch off and on, to run it again.

### Reading the result

For each step you get:

- **lowest**: the lowest reading in that step,
- **average**: the average of all readings in that step,
- **readings**: how many it took (thousands: it reads all the time),
- **% below 4.60 V**: how much of the step the supply spent low.

The verdict uses the lowest reading of each step, and the worst step decides:

| Lowest reading | Verdict | Meaning |
|---|---|---|
| 4.75 V or more | OK | the power is fine |
| 4.60 to 4.75 V | WEAK | it may restart now and then, more as the battery runs down |
| below 4.60 V, or a restart | WILL RESTART | the same as your real runs |

Step 0 (nothing moving) should be about 4.9 to 5.1 V. If it is lower, the battery is flat, or the battery switch is off and only USB is powering the Nano.

**Why the thresholds are so far above 2.7 V.** The chip resets at 2.7 V, but the test cannot see the very bottom of a dip. It reads the supply by measuring the chip's own 1.1 V reference against it, exactly like the POWER lines of robot_e33_v2. Each reading takes about 30 microseconds and is a kind of short average, and the deepest moments (a motor starting or reversing, a servo pulse) are very short spikes that fall between readings or get smoothed out. So a short dip goes much lower than the printed lowest. Your own log proves it: the readings said 4.4 to 4.6 V while the Nano was restarting. That is why anything below 4.60 V counts as WILL RESTART here.

The motor steps (9, 10) may look better than they are for the same reason: the start-up current of a motor lasts only a few milliseconds.

## 3. How to fix the power (cheapest first)

Test again after each fix (Run B). Stop when you get OK on the battery alone.

### Fix 1: a fresh battery

A new alkaline PP3 gives more current than a used one. This often turns WILL RESTART into WEAK, but it only lasts a short while, because a PP3 is weak even when new (see below).

### Fix 2: a big capacitor at the servos

Put a **470 to 1000 uF** electrolytic capacitor (rated 10 V or more, 16 V is common) across the servo 5 V and GND, close to the servos. It gives the servos the short bursts of current they need, so the 5 V line does not dip at each pulse.

**Polarity matters.** An electrolytic capacitor has a minus side, marked with a stripe and usually the shorter leg. The stripe side goes to **GND**, the other leg to **+5 V**. Fitted the wrong way round it gets hot and can burst.

### Fix 3: a separate supply for the servos (the real fix)

Give the servos their own battery, so they cannot pull down the Nano's 5 V:

- **4 x AA** batteries in a holder (about 6 V with alkaline cells, 4.8 V with rechargeable NiMH), or
- a **5 V BEC** (a small 5 V regulator made for servos, from a bigger battery).

Rules:

- the servo **+ (red)** wires go to the + of the new pack, **NOT to the Nano's 5 V pin**,
- the **grounds must be connected**: the pack's minus, the servo brown/black wires and the Nano GND all joined,
- the servo signal wires stay on D5 (gripper) and D4 (arm),
- check that your servos are rated for the pack voltage (SG90 and MG90S: 4.8 to 6 V).

```
   4 x AA pack (6 V)
   (+) ----+-------+-----------------+
           |       |                 |
          C+      red               red
   470 to  |   [GRIPPER servo]   [ARM servo]
   1000 uF |     brown  orange     brown  orange
          C-       |      |          |      |
  (stripe) |       |      +-> D5     |      +-> D4    (Nano signal pins)
           |       |                 |
   (-) ----+-------+-----------------+------------> Nano GND
                                                    (the common ground)

   Nano 5V pin: NOT connected to the servos.
   C: the capacitor of Fix 2, close to the servos, stripe (minus) on the (-) line.
```

The motors and the Nano stay on the 9 V and the step-down as before. If the motors still pull the supply down in steps 9 and 10, the next step up is a better battery for the motors too, for example 6 x AA, or two Li-ion cells (7.4 V) into the TB6612 and the step-down.

### Why a 9 V PP3 is a poor battery for motors and servos

A PP3 is made of six tiny cells. It holds little energy (about 500 mAh) and it has a high **internal resistance**: about 1.5 to 3 ohm when new, much more when partly used. Every amp you draw loses that many volts inside the battery: at 1 A a fresh PP3 drops to about 6 to 7 V, and a used one far lower. In practice a PP3 gives about **0.5 A at most** before its voltage collapses. Two motors starting plus a servo moving want 1 to 2 A for a moment. AA cells are much bigger: each has about a tenth of the resistance and several times the energy.

## 4. Test again to see the improvement

1. After a fix, put the robot on the box and do **Run B** (battery only, no USB).
2. Read the saved results with the USB cable, as above.
3. Compare the steps that were bad before (usually 2 to 8 for the servos and 12, 13 for servos with motors). The lowest readings should go up, and the verdict should become OK, with 1 LED blink.
4. Then upload robot_e33_v2 (or v3) again and do a real run. The `POWER pick` and `POWER place` lines of the run log should now show `min=` above 4.75 V, without `LOW`, and no restarts.

Note: the test saves its results in the last 34 bytes of the EEPROM. Your calibration is not touched. The run log of the mission sketch can only lose its last few records, and the next run writes a new log anyway.
