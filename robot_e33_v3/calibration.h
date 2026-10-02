/* =====================================================================
 *  calibration.h: THE ONLY FILE YOU NEED TO EDIT
 * =====================================================================
 *
 *  Everything that belongs to YOUR robot and YOUR field is here.
 *  The other files are the course library and should not need changes.
 *
 *  HOW TO USE IT (the same steps as "Getting it running" in README.md)
 *
 *   1. Measure the field and robot sizes below with a ruler
 *      (CELL_CM matters most).
 *
 *   2. The checks. For each one set ROBOT_MODE (below), upload, and open
 *      the Serial Monitor (115200 baud, "Newline"). Lift the robot or
 *      switch the motors off BEFORE you upload: the new mode starts at once.
 *        MODE_GRIPPER_CHECK  the robot never drives: check GRIP_CLOSED
 *        MODE_SENSOR_CHECK   the robot never moves: every sensor OK
 *        MODE_MOTOR_CHECK    WHEELS OFF THE GROUND: each wheel turns the
 *                            right way (if not: LEFT_ / RIGHT_REVERSED)
 *
 *   3. Set ROBOT_MODE to MODE_CALIBRATE and upload.
 *      Put a fresh battery in. Place the robot at the mission start:
 *      wheels (axle) over the C1 line, on the MID line, facing east.
 *      Switch on and step back. It waits until it sees the line, then
 *      calibrates itself for about 90 seconds and stops at the start.
 *      Nothing to press. Then plug in USB and read the report (README.md:
 *      "When the calibration stops" if it says STOPPED).
 *
 *   3b. (Optional, after 3) MODE_TURN_CHECK: put the WHEELS (axle) on the
 *      C2 MID or C3 MID crossing, facing along MID. It turns left and right,
 *      90 and 180, 24 times on the spot with the saved calibration, then
 *      prints a report: copy it from BEGIN to END and send it.
 *
 *   4. Set ROBOT_MODE back to MODE_MISSION and upload. The mission loads
 *      the calibration from EEPROM automatically.
 *      (Optional: the calibration prints a block of #define lines. Paste
 *      it over the CAL VALUES section at the bottom of this file and set
 *      CAL_USE_EEPROM to 0 to freeze the values in the source code.)
 *
 *  Open the Serial Monitor at 115200 baud to see what the robot is doing.
 * ===================================================================== */
#ifndef CALIBRATION_H
#define CALIBRATION_H

/* ------------------------------------------------------------ mode ----- */
#define MODE_MISSION      0   /* run the e33 mission                         */
#define MODE_CALIBRATE    1   /* measure sensors and motors, save to EEPROM   */
#define MODE_METER        2   /* print the sensors live; the robot never moves */
#define MODE_MOTOR_CHECK  3   /* wheels off the ground: test each motor        */
#define MODE_SENSOR_CHECK 4   /* white vs black of every sensor, as a report  */
                              /* you can copy and send; the robot never moves */
#define MODE_GRIPPER_CHECK 5  /* move the gripper / arm from the Serial Monitor */
                              /* to find your angles; the robot never drives  */
#define MODE_TURN_CHECK   6   /* the mission's turns, again and again, on one  */
                              /* MID crossing, timed; the robot never drives  */
                              /* along a line                                 */

#ifndef ROBOT_MODE                /* (the simulator can set it from outside) */
#define ROBOT_MODE   MODE_MISSION   /* <- change the mode here */
#endif

#define CAL_USE_EEPROM    1   /* 1 = use the saved calibration if there is one
                               * 0 = ignore EEPROM, use the CAL VALUES below    */
#define SERIAL_BAUD  115200
#ifndef TRACE_ROUTE
#define TRACE_ROUTE       1   /* print one line at every waypoint (0 frees
                               * about 1000 bytes of flash if a change ever
                               * does not fit; the run log still keeps the
                               * record)                                    */
#endif
#define RUN_LOG           1   /* keep a record of the last run in EEPROM and
                               * print it at the next start (see runLog.h)    */
/* [E33] What a fault does in the mission (the calibration always stops):
 * 0 = the robot does NOT stop. It prints the fault on Serial, notes it in
 *     the run log once, and goes on with the route. A lost line is
 *     searched for again and again: on a little, then the swing.
 * 1 = it stops for good, as before, and the LED blinks the fault number. */
#ifndef STOP_ON_FAULT
#define STOP_ON_FAULT     0
#endif

/* ------------------------------------------------------------ route ---- */
#ifndef ROUTE_ORDER
#define ROUTE_ORDER     312   /* the order of the objects (README: "Two routes")
                               * 312 = objects 3, 1, 2: tested the most
                               * 132 = objects 1, 3, 2: 9 turns instead of 11
                               *       and 19 crossings instead of 25, but it
                               *       turns where it stands at the start, so
                               *       put the wheels exactly over C1         */
#endif
#ifndef PRACTICE
#define PRACTICE          0   /* 0 = the whole mission. 1 or 2 = stop after
                               * putting down that many objects, to practise
                               * the first object(s) and their turns again
                               * and again without setting up all three     */
#endif

/* ------------------------------------------------- measure with a ruler --- */
/* Distance between neighbouring field lines, centre to centre. Measure from
 * the C1 line to the C4 line along MID and divide by 3. THE MOST IMPORTANT
 * NUMBER: the robot uses it to check it is where it thinks it is. */
#define CELL_CM          25.0f

/* Distance from the MID line to the TOP line (and to the BOT line), centre
 * to centre. Usually the same as CELL_CM, but measure it to be sure. */
#define ROW_CM           25.0f

/* Sensor bar to the wheel axle. You measured this: 9.5 cm. The axle is the
 * point the robot turns about, so when the bar sees a crossing the robot is
 * still 9.5 cm short of it. */
#define SENSOR_AHEAD_CM   9.5f

/* Width of the black tape. */
#define TAPE_W_CM         1.8f

/* Wheel axle to the centre of an object held in the gripper
 * (arm down, jaws closed on it). */
#define GRIP_REACH_CM    12.0f

/* Centre of the outer line to the centre of an object (o) and of a target (x). */
#define OBJ_BEYOND_CM     5.0f
#define TGT_BEYOND_CM     5.0f

/* ------------------------------------------------ measured servo angles --- */
/* These are the angles you measured on your own arm. */
#define GRIP_OPEN       140
#define GRIP_CLOSED      75
#define GRIP_RELEASE    149
#define ARM_DOWN        103
#define ARM_CARRY        70
#define ARM_HIGH         50

/* After closing on an object the gripper opens this many degrees again, so
 * the servo HOLDS the object instead of pushing against it at full current.
 * A servo pushing at full current from a 9 V battery can reset the Nano. */
#define GRIP_BACKOFF_DEG  3   /* if the object slips out while carried: 1 or 0 */

/* ------------------------------------------------ supply voltage ------- */
/* [E33] The Nano measures its own 5 V supply (VCC) with no extra wire: at
 * the start, and while the servos move at every pick and place. The run log
 * keeps it (POWER lines). Below VCC_WARN_MV it prints a WARNING and the LED
 * blinks long-short-short-short twice ("B" for battery); it never stops the
 * mission for it. The chip's built-in 1.1 V it measures against is only
 * about 10 % exact, so the volts are approximate: compare them with each
 * other (at rest against while the servos move, one run against the next).
 * To make them exact: measure the 5V pin with a multimeter while the robot
 * waits at the start, then set
 *   BANDGAP_MV = BANDGAP_MV x (meter volts) / (the POWER start rest= volts). */
#define VCC_WARN_MV    4500   /* warn below 4.5 V (a brown-out comes near 2.7 V) */
#define BANDGAP_MV     1100   /* the chip's reference in mV (1.0 to 1.2 V)       */

/* ------------------------------------------------------------ speed ---- */
/* Speeds are in the slides' units (0..255). After calibration the robot
 * removes each motor's dead band and the left/right mismatch, so the same
 * number makes both wheels turn at the same speed. */
#define MAX_SP          110   /* walking speed (slides: maxSp 255)          */
#define SP_START         50   /* speed after every stop (slides: sp = 50)   */
#define MOVE_SP          70   /* short exact moves before turns and grips   */
#define TURN_SP          75   /* spinning on the spot                       */
#define TURN_SLOW_SP     60   /* last part of a turn, closing on the line   */

/* -------------------------------------------------- motor wiring -------- */
/* If a wheel runs backwards, set its flag to 1 instead of swapping wires.
 * MODE_MOTOR_CHECK tells you. */
#define LEFT_REVERSED     0
#define RIGHT_REVERSED    0

/* ------------------------------------------------- sensor sensitivity --- */
/* A sensor switches ON when its reading passes this percentage of the way
 * from white to black, and OFF again below the second number. These are
 * percentages of EACH channel's own contrast, so a weak channel still works.
 * Lower SENS_ON_PCT = more sensitive (sees fainter line, more noise). */
#define SENS_ON_PCT      45
#define SENS_OFF_PCT     30

/* =====================================================================
 *  CAL VALUES
 *  Used when there is no calibration in EEPROM, or when CAL_USE_EEPROM is 0.
 *  The calibration prints a block in exactly this format: paste it here.
 *  The numbers below are measured on YOUR robot (see the note below).
 * ===================================================================== */
/* [E33] v3: these are YOUR robot's numbers, so it runs well without
 * MODE_CALIBRATE. The sensor levels are your printouts (white on the field,
 * black on the tape). The speeds and turn times come from your 19 real runs:
 * v2 learned that the robot drives about 1.6 to 1.8 times faster than its
 * guess (cruise about 25 cm/s), and the turns took about 600 to 700 ms
 * (500 / 460 ms at full turn speed). The old guesses were: CAL_LO 250 and
 * CAL_HI 750 for every sensor, V_CRUISE 1500, V_MOVE 950, LAG_MOVE 120,
 * T90 850 / 850. A calibration (MODE_CALIBRATE) still replaces them. */
#define CAL_VALUES_FROM_RUN   0          /* 0 = never calibrated              */
#define CAL_LINE_LOW          0          /* 1 = black reads LOW on your bar   */
#define CAL_LO   { 203, 159, 74, 78, 72, 98, 99, 99 }  /* white readings */
#define CAL_HI   { 970, 971, 964, 965, 966, 975, 971, 971 }  /* black readings */
#define CAL_DEAD_MASK         0x00       /* channels too weak to use          */
#define CAL_DB_LF            45          /* duty where each wheel starts:     */
#define CAL_DB_LR            45          /*   left fwd, left rev,             */
#define CAL_DB_RF            30          /*   right fwd, right rev            */
#define CAL_DB_RR            30          /* (guess: your left motor is weaker) */
#define CAL_TRIM_L         1000          /* per mille; the weaker motor is 1000 */
#define CAL_TRIM_R          800          /* guess: "left 270 vs right 360" is 750;
                                          * 800 also works uncalibrated for a left
                                          * motor anywhere from 55% to 90%      */
#define CAL_TRIM_LR        1000          /* the same, driving backward          */
#define CAL_TRIM_RR         800          /* (spins use one wheel backward)      */
#define CAL_V_CRUISE_X100  2500          /* cm/s x100 at MAX_SP                */
#define CAL_V_MOVE_X100    1800          /* cm/s x100 at MOVE_SP, from rest    */
#define CAL_LAG_MOVE_MS      70          /* start-up delay of a move from rest */
#define CAL_T90_L_MS        500          /* ms per 90 degrees at TURN_SP       */
#define CAL_T90_R_MS        460
/* ---- end CAL VALUES ---- */

#endif
