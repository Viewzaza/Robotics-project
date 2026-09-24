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

#define DUTY_MIN_MOVE     (DUTY_DEADBAND + 10)
#define DUTY_CRUISE       170   /* line-following cruise                   */
#define DUTY_APPROACH      90   /* slowed down near an expected junction   */
#define DUTY_PRECISE       80   /* fixed-distance advances - repeatable    */
#define DUTY_PIVOT        110   /* spinning on the spot                    */
#define DUTY_START         70   /* speed right after a turn, before ramp   */

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
 * calibration mode spins the robot for you to time it. */
#define MS_PER_90DEG      584

/* Left/right motor mismatch trim, percent applied to the right motor.
 * 100 = matched. MEASURE ME if the robot curves while driving "straight". */
#define TRIM_R_PCT        100

/* ADC prescaler. Arduino leaves this at 128 (0x07) = 125 kHz ADC clock and
 * ~104 us per conversion, so reading the eight channels costs 832 us and that
 * alone sets the control-loop period.
 *   0x04 = /16  -> 1   MHz, ~13 us  (default here; ~8x faster)
 *   0x05 = /32  -> 500 kHz, ~26 us  (use this if readings look noisy)
 *   0x06 = /64  -> 250 kHz, ~52 us
 * The datasheet quotes full 10-bit accuracy only to 200 kHz, which costs a
 * couple of LSB here. Irrelevant for a threshold decision with ~700 counts of
 * black/white contrast, but bump to 0x05 if your bar has a high output
 * impedance and the mask looks unstable. */
#define ADC_PRESCALER   0x04

/* --------------------------------------------------------------- sensing -- */
/* Fallback threshold when no EEPROM calibration is present. Values above this
 * count as "on the line". */
#define THRESH_FALLBACK   500

/* Hysteresis around the calibrated threshold, in raw ADC counts. Stops a
 * sensor sitting on the edge of the tape from chattering. */
#define THRESH_HYST        40

/* A junction is declared when at least this many sensors see line at once. */
#define JUNCTION_MIN_BITS   5

/* Debounce: a junction must persist this long to count, and the robot must
 * then travel this far before another junction can be counted. */
#define JUNCTION_DEBOUNCE_MS   12
#define JUNCTION_REARM_CM      8.0f

/* -------------------------------------------------------------- servos ---- */
#define SERVO_GRIP_OPEN    140
#define SERVO_GRIP_CLOSED   65
#define SERVO_GRIP_RELEASE 149
#define SERVO_ARM_DOWN     105
#define SERVO_ARM_CARRY     90
#define SERVO_ARM_HIGH      40

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
