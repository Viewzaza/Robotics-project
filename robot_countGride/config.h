/* config.h -- every tunable number lives here.
 *
 * The values marked MEASURE ME must be measured on the real robot. The
 * calibration mode in the sketch measures most of them for you and stores the
 * result in EEPROM; the defaults below are only the fallback used when EEPROM
 * holds no valid calibration.
 */
#ifndef CONFIG_H
#define CONFIG_H

/* ------------------------------------------------------------------ pins -- */
#define sp_L   6     /* TB6612 PWMA - left  */
#define F_L   12     /* TB6612 AIN1         */
#define B_L   11     /* TB6612 AIN2         */

#define sp_R   3     /* TB6612 PWMB - right */
#define F_R    7     /* TB6612 BIN1         */
#define B_R    8     /* TB6612 BIN2         */
#define STBY  10     /* TB6612 standby      */

#define x_pin  5     /* gripper servo  */
#define y_pin  4     /* arm lift servo */

#define PIN_START  9   /* optional start button to GND, INPUT_PULLUP */
#define PIN_LED   13   /* on-board LED                               */

/* -------------------------------------------------------------- geometry -- */
/* Distance from the sensor bar to the wheel axle, which is also the centre of
 * rotation. MEASURED BY THE USER = 9.5 cm.
 * The moment the bar sees a crossing line, the pivot centre is still this far
 * short of it, so the robot must advance exactly this much before pivoting. */
#define SENSOR_AHEAD_CM   9.5f

/* Spacing between adjacent sensors on the bar, cm. MEASURE ME. */
#define SENSOR_PITCH_CM   1.2f

/* Distance from the wheel axle to the gripper jaws, cm. MEASURE ME. */
#define GRIP_REACH_CM    12.0f

/* Grid pitch: distance between neighbouring field lines, cm. MEASURE ME.
 * Used to predict where the next junction is so the robot can slow down
 * before it instead of hitting it at full speed. */
#define CELL_CM          25.0f

/* How far past the outer line the objects and targets sit, cm. MEASURE ME. */
#define OBJECT_BEYOND_CM  5.0f

/* Half the width of the line tape, cm. 19 mm electrical tape -> 0.95.
 * This is only the STARTING guess: the firmware re-measures it every time it
 * drives straight through a junction. It matters because a junction is declared
 * when the bar's leading edge reaches the tape, not when the bar is over the
 * centre of it, so every distance measured from a junction is short by this
 * much until it is corrected. MEASURE ME (or just let the robot do it). */
#define LINE_HALF_W_CM    0.9f

/* ----------------------------------------------------------------- speed -- */
/* Forward speed in cm/s at DUTY_CAL. This is THE calibration constant for all
 * distance work. Measure it with calibration mode: the robot drives straight
 * for 3 s at DUTY_CAL, you measure the distance with a ruler, divide by 3.
 * MEASURE ME. */
#define DUTY_CAL          120
#define CM_PER_S_AT_CAL   16.5f

/* Duty below which the gearmotors do not turn at all. MEASURE ME:
 * calibration mode ramps the duty up until the wheels start moving. */
#define DUTY_DEADBAND      25

/* Walking pace. Lowered from the original 170/90/80/110/70 after running on the
 * real robot: slower is simply more reliable here. The bar is 9.5 cm ahead of
 * the pivot, so every centimetre of overshoot past a crossing becomes a turn
 * about the wrong point, and the slower the approach the smaller that error is.
 * Raise DUTY_CRUISE again if you want a faster run and the counting still holds. */
#define DUTY_MIN_MOVE     (DUTY_DEADBAND + 10)
#define DUTY_CRUISE       115   /* line-following cruise                   */
#define DUTY_APPROACH      75   /* slowed down near an expected junction   */
#define DUTY_PRECISE       70   /* fixed-distance advances - repeatable    */
#define DUTY_PIVOT         95   /* spinning on the spot                    */
#define DUTY_START         60   /* speed right after a turn, before ramp   */

/* Ramp: how fast the cruise speed builds up, duty units per 10 ms. */
#define RAMP_STEP           4

/* Start decelerating this far before the junction the robot expects next.
 * Arriving at an intersection at full cruise is what makes the count unreliable
 * and the turn overshoot. */
#define APPROACH_CM      9.0f

/* ------------------------------------------------------------- PID gains -- */
/* The course values are kp=1, ki=0, kd=0.7 against an error of -7..+7. The
 * centroid gives the same range but continuously and at a much higher loop
 * rate, so the derivative gain has to come down or it amplifies sensor noise. */
#define PID_KP           1.10f
#define PID_KI           0.00f
#define PID_KD           0.22f

/* Milliseconds for a 90 degree pivot at DUTY_PIVOT. Used as the coarse,
 * open-loop part of a turn; the line sensor then finishes the job. MEASURE ME:
 * calibration mode spins the robot for you to time it.
 *
 * THIS VALUE IS TIED TO DUTY_PIVOT. Turn rate is roughly proportional to
 * (DUTY_PIVOT - DUTY_DEADBAND), so if you change DUTY_PIVOT you must rescale
 * this by the inverse ratio or every turn comes up short. Dropping the pivot
 * duty from 110 to 95 took the numerator from 85 to 70, so this went from 584
 * to 584 * 85/70 = 709. Re-measure it rather than trusting the arithmetic. */
#define MS_PER_90DEG      709

/* Left/right motor mismatch trim, percent applied to the right motor.
 * 100 = matched. MEASURE ME if the robot curves while driving "straight". */
#define TRIM_R_PCT        100

/* ADC prescaler. Arduino leaves this at 128 (0x07) = 125 kHz ADC clock and
 * ~104 us per conversion, so reading the eight channels costs 832 us and that
 * alone sets the control-loop period.
 *   0x04 = /16  -> 1   MHz, ~13 us  (fastest, least settling time)
 *   0x05 = /32  -> 500 kHz, ~26 us  (default here)
 *   0x06 = /64  -> 250 kHz, ~52 us
 * The datasheet quotes full 10-bit accuracy only to 200 kHz, which costs a
 * couple of LSB here. Irrelevant for a threshold decision with ~700 counts of
 * black/white contrast, but bump to 0x05 if your bar has a high output
 * impedance and the mask looks unstable. */
#define ADC_PRESCALER   0x05

/* --------------------------------------------------------------- sensing -- */
/* Fallback threshold when no EEPROM calibration is present. Values above this
 * count as "on the line". */
#define THRESH_FALLBACK   500

/* Hysteresis around the calibrated threshold, in raw ADC counts. Stops a
 * sensor sitting on the edge of the tape from chattering. */
#define THRESH_HYST        40

/* A junction is declared when at least this many sensors see line at once. */
#define JUNCTION_MIN_BITS   5

/* ------------------------------------------------- self-calibrating scale -- */
/* Width of the tape, cm. MEASURE ME -- lay a ruler across a line.
 *
 * This is the one ABSOLUTE length the robot can see for itself. Every other
 * length it deals with (cell pitch, how far past the line an object sits) is a
 * property of a field it has never seen. The tape it drives on is the ruler it
 * carries with it: while the bar sweeps across a line at right angles, every
 * sensor is lit at once, and the ground the wheels cover during that window IS
 * the tape width. Comparing that with the distance the odometer THINKS it
 * covered gives the odometer's scale error directly -- no encoder needed.
 *
 * That single number is what the whole mission hangs on. CM_PER_S_AT_CAL is
 * calibrated on one battery at one moment; a fresh cell or a sagging one moves
 * the true speed by +-30 percent, which scales every advanceCm() AND every
 * timed pivot by the same factor. A 90 degree turn becomes a 66 degree turn and
 * the robot drives off the field. */
#define LINE_WIDTH_CM     1.8f

/* Only believe a dwell measurement inside this band, as a multiple of
 * LINE_WIDTH_CM. Outside it the bar was skewed, or clipped a corner, or the
 * robot was braking mid-junction. */
#define SCALE_MEAS_MIN    0.30f
#define SCALE_MEAS_MAX    4.00f

/* How much of each measured error to apply. The first fix is nearly complete
 * because there are only two pass-through crossings before the first turn, and
 * the turn is what a scale error destroys. Later fixes are damped, because by
 * then the estimate is close and the measurement noise is not. */
#define SCALE_GAIN_FIRST  0.90f
#define SCALE_GAIN_LATER  0.50f

/* Hard bounds on the cumulative correction, so one wild measurement can never
 * leave the robot with a calibration it cannot drive on. */
#define SCALE_CLAMP_LO    0.35f
#define SCALE_CLAMP_HI    2.60f

/* Debounce: a junction must persist this long to count, and the robot must
 * then travel this far before another junction can be counted. */
#define JUNCTION_DEBOUNCE_MS   12
#define JUNCTION_REARM_CM      8.0f

/* ---------------------------------------------------------- diagnostics --- */
/* Set to 1 and re-upload to turn the robot into a sensor meter: it never
 * drives, it just streams every channel's raw reading, its threshold, and the
 * resulting mask, several times a second. Slide the robot on and off a line and
 * watch the numbers.
 *
 * What you are looking for, per channel:
 *   - over the WHITE mat, a low number, and over the TAPE a high one
 *   - a gap between those two of at least ~150 counts, ideally 400+
 *   - all eight channels roughly agreeing with each other
 * If the gap is small, the bar is too high or too low off the surface (aim for
 * 5-10 mm), the surface is not matte enough, or the bar's own trimpot needs
 * adjusting. No threshold setting can rescue a channel that barely moves.
 * Set back to 0 before running the mission. */
#define DIAG_SENSORS        0

/* Set to 1 to test the motor wiring. The robot lifts its wheels' worth of
 * doubt: it drives LEFT forward alone, then LEFT back, then RIGHT forward,
 * then RIGHT back, announcing each over serial. Put it on a book so the wheels
 * spin free, watch, and set the INVERT flags below for whichever came out
 * backwards. Set back to 0 afterwards. */
#define DIAG_MOTORS         0

/* Motor direction. Which way round each motor's two wires go into AO1/AO2 and
 * BO1/BO2 decides which way that wheel turns, and neither order is "right" --
 * they are just two plain wires. Wire them either way, run DIAG_MOTORS, and
 * flip the flag for whichever wheel spun the wrong way. */
#define INVERT_LEFT         0
#define INVERT_RIGHT        0

/* -------------------------------------------------------------- servos ---- */
/* Measured on the real arm. */
#define SERVO_GRIP_OPEN    140
#define SERVO_GRIP_CLOSED   75
#define SERVO_GRIP_RELEASE 149
#define SERVO_ARM_DOWN     103
#define SERVO_ARM_CARRY     70
#define SERVO_ARM_HIGH      50

/* Servos are stepped this many degrees every SERVO_STEP_MS instead of being
 * commanded straight to the target. A single 9 V alkaline cannot supply the
 * inrush of an unrestrained servo sweep without dipping the 5 V rail, which
 * browns out the Nano mid-mission. */
#define SERVO_STEP_DEG       3
#define SERVO_STEP_MS        12

/* ------------------------------------------------------------- timeouts --- */
/* Nothing in this firmware is allowed to block forever. Each of these is the
 * point at which the robot gives up on a manoeuvre and reports a fault rather
 * than sitting still with the motors braked. */
#define TIMEOUT_PIVOT_MS    3500
#define TIMEOUT_LEG_MS     12000
#define TIMEOUT_SEARCH_MS   1500

/* --------------------------------------------------------------- serial --- */
/* 9600 baud is too slow: one 40-character telemetry line takes ~42 ms and
 * blocks the control loop once the 64-byte TX buffer fills. */
#define SERIAL_BAUD      115200
#define TELEMETRY_MS        250

/* --------------------------------------------------------------- EEPROM --- */
#define EE_MAGIC      0x52424Bul   /* "RBK" */
#define EE_VERSION             3
#define EE_ADDR_CAL            0
#define EE_ADDR_CHECKPOINT    64

/* Set to 1 to have the robot resume the mission after an unexpected reset
 * instead of starting over with objects already moved. Requires a bootloader
 * that clears the watchdog properly - see README. */
#define ENABLE_CRASH_RESUME    1

/* The hardware watchdog is OFF by default: the stock Nano (old Optiboot)
 * reset-loops on a watchdog reset. Turn on ONLY if your board has a current
 * bootloader. See README. */
#define ENABLE_WATCHDOG        0



#endif
