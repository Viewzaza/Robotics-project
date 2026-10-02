/* =====================================================================
 *  calibration.h: THE ONLY FILE YOU NEED TO EDIT
 * =====================================================================
 *
 *  Everything that belongs to YOUR robot and YOUR field is here.
 *  The other files are the course library and should not need changes.
 *
 *  THIS SAME FILE IS IN BOTH PROGRAMS, e33_calibrate/ and e33_mission/,
 *  and both must use the same numbers (sizes, speeds, servo angles). Edit
 *  either copy, save, then double-click tools\copy_settings.bat (the tools
 *  folder next to e33_mission): it copies the one you changed last over the
 *  other. Do it after EVERY change, before the next upload.
 *
 *  HOW TO USE IT (the same steps as "Getting it running" in
 *  e33_mission/README.md)
 *  There are two programs:
 *    e33_calibrate   the checks and the calibration: choose the STEP at the
 *                    top of e33_calibrate.ino, upload, open the Serial Monitor
 *    e33_mission     the real run: upload it, put the robot at the start
 *
 *   1. Measure the field and robot sizes below with a ruler
 *      (CELL_CM matters most).
 *
 *   2. The checks, with e33_calibrate, in this order. For each one set STEP
 *      at the top of e33_calibrate.ino, upload, and open the Serial Monitor
 *      (115200 baud, "New Line"). Lift the robot off the field BEFORE you
 *      upload: the new program starts at once.
 *        MODE_SENSOR_CHECK   the robot never moves: every sensor OK (its
 *                            report ends with the sensor part of the CAL
 *                            VALUES; pasting it is not needed)
 *        MODE_GRIPPER_CHECK  the robot never drives: check GRIP_CLOSED
 *        MODE_MOTOR_CHECK    WHEELS OFF THE GROUND: each wheel turns the
 *                            right way (if not: LEFT_ / RIGHT_REVERSED)
 *
 *   3. e33_calibrate with STEP MODE_CALIBRATE: upload.
 *      Put a fresh battery in. Place the robot at the mission start:
 *      wheels (axle) over the C1 line, on the MID line, facing east.
 *      Switch on and step back. It waits until it sees the line, then
 *      calibrates itself for about a minute and stops at the start.
 *      Nothing to press. Then plug in USB and read the report
 *      (e33_mission/README.md: "When the calibration stops" if it says
 *      STOPPED).
 *
 *   3b. (Optional, after 3) STEP MODE_TURN_CHECK: put the WHEELS (axle) on the
 *      C2 MID or C3 MID crossing, facing along MID. It turns left and right,
 *      90 and 180, 24 times on the spot with the saved calibration, then
 *      prints a report: copy it from BEGIN to END and send it.
 *
 *   4. Upload e33_mission. It loads the calibration from EEPROM
 *      automatically, and goes as soon as the line is under the middle of
 *      the bar. It never calibrates by itself.
 *      (Optional: the calibration prints a block of #define lines. Select
 *      the CAL VALUES block at the bottom of this file, from
 *      #define CAL_VALUES_FROM_RUN down to "end CAL VALUES", paste the
 *      printed block over it, set CAL_USE_EEPROM to 0 to freeze the values,
 *      and run copy_settings. A new calibration is then IGNORED until you
 *      set CAL_USE_EEPROM back to 1.)
 *
 *  Open the Serial Monitor at 115200 baud to see what the robot is doing.
 * ===================================================================== */
#ifndef CALIBRATION_H
#define CALIBRATION_H

/* ------------------------------------------------------------ mode ----- */
/* [E33] Nothing to set here: MODE_MISSION is e33_mission, and the others
 * are the STEPs of e33_calibrate (chosen at the top of e33_calibrate.ino). */
#define MODE_MISSION      0   /* e33_mission: run the e33 mission            */
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

/* The program sets the mode: e33_mission is always the mission, and
 * e33_calibrate takes it from STEP at the top of e33_calibrate.ino. */
#ifndef ROBOT_MODE
#define ROBOT_MODE   MODE_MISSION
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

/* ------------------------------------------------------------ route ---- */
#ifndef ROUTE_ORDER
#define ROUTE_ORDER     312   /* the order of the objects (README: "Two routes")
                               * 312 = objects 3, 1, 2: tested the most
                               * 132 = objects 1, 3, 2: 9 turns instead of 11
                               *       and 19 crossings instead of 25, but it
                               *       turns where it stands at the start, so
                               *       put the wheels exactly over C1         */
#endif
#ifndef START_WAIT_MS
#define START_WAIT_MS   200   /* it goes once the line has been under the
                               * middle of the bar this long (ms)             */
#endif
#ifndef START_WAIT_AFTER_STOP_MS
#define START_WAIT_AFTER_STOP_MS 5000  /* ... after a run that did not finish */
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
 * Lower SENS_ON_PCT = more sensitive (sees fainter line, more noise).
 * [E33] SENS_OFF_PCT was 30. Where the white of the field reads darker than
 * at the calibration (a grey patch, a seam, a lamp or sunlight), a sensor
 * that just left the line stayed on while that white was above 30%: a
 * crossing that never ended (FAULT 2) or a turn that never saw the bar clear
 * (FAULT 6 / 7). In the simulator with your sensors that began where a patch
 * or a light change made the white read about 250 higher. At 38 the robot
 * copes up to about 300 higher (the limit is then SENS_ON_PCT itself; see
 * "Setting up the field" in the README). Do not set it closer to SENS_ON_PCT: the
 * gap between the two is what stops a noisy sensor flickering at a line edge. */
#define SENS_ON_PCT      45
#define SENS_OFF_PCT     38

/* Which way black reads is CAL_LINE_LOW below, and the robot never guesses
 * it: your bar reads black HIGH (about 970) and white LOW (about 70 to 200),
 * so it is 0. (MODE_SENSOR_CHECK says which way yours reads.) */
#ifndef CAL_SENSORS_MEASURED
#define CAL_SENSORS_MEASURED  1   /* 1 = CAL_LO / CAL_HI below were measured on
                                   * this robot, so the mission uses them as
                                   * they are. 0 = guesses: it measures the
                                   * levels itself, standing still, if the
                                   * guesses do not see the line.            */
#endif

/* =====================================================================
 *  CAL VALUES
 *  Used when there is no calibration in EEPROM, or when CAL_USE_EEPROM is 0.
 *  The calibration prints a block in exactly this format: select from
 *  #define CAL_VALUES_FROM_RUN down to "end CAL VALUES" and paste over it.
 *  (Pasted above the old lines, the upload stops with "CAL VALUES twice".)
 *  The sensor levels (CAL_LO, CAL_HI) were measured on your robot; the
 *  motor and turn numbers are safe starting guesses, not measurements.
 * ===================================================================== */
#if defined(CAL_VALUES_FROM_RUN) || defined(CAL_LINE_LOW) || defined(CAL_LO) || defined(CAL_HI)
  #error "CAL VALUES twice in calibration.h: new lines were pasted ABOVE the old ones. Delete the old lines, or paste OVER them."
#endif
#define CAL_VALUES_FROM_RUN   0          /* 0 = never calibrated              */
#define CAL_LINE_LOW          0          /* 1 = black reads LOW on your bar   */
/* measured on your robot (Serial Monitor, 2026-09-29): white is the highest
 * reading each sensor gave on white, black the lowest it gave on black */
#define CAL_LO   { 203, 159,  74,  78,  72,  98,  99,  99 }  /* white readings */
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
#define CAL_V_CRUISE_X100  1500          /* cm/s x100 at MAX_SP                */
#define CAL_V_MOVE_X100     950          /* cm/s x100 at MOVE_SP, from rest    */
#define CAL_LAG_MOVE_MS     120          /* start-up delay of a move from rest */
#define CAL_T90_L_MS        850          /* ms per 90 degrees at TURN_SP       */
#define CAL_T90_R_MS        850
/* ---- end CAL VALUES ---- */

#endif
