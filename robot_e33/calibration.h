/* =====================================================================
 *  calibration.h: THE ONLY FILE YOU NEED TO EDIT
 * =====================================================================
 *
 *  Everything that belongs to YOUR robot and YOUR field is here.
 *  The other files are the course library and should not need changes.
 *
 *  HOW TO USE IT
 *
 *   1. Measure the field and robot sizes below with a ruler
 *      (CELL_CM matters most).
 *
 *   2. Set ROBOT_MODE to MODE_CALIBRATE and upload.
 *      Put a fresh battery in. Place the robot at the mission start:
 *      wheels (axle) over the C1 line, on the MID line, facing east.
 *      Switch on and step back. It waits until it sees the line, then
 *      calibrates itself for about 90 seconds and stops at the start.
 *      Nothing to press.
 *
 *   3. Set ROBOT_MODE back to MODE_MISSION and upload. The mission loads
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

#ifndef ROBOT_MODE                /* (the simulator can set it from outside) */
#define ROBOT_MODE   MODE_MISSION
#endif

#define CAL_USE_EEPROM    1   /* 1 = use the saved calibration if there is one
                               * 0 = ignore EEPROM, use the CAL VALUES below    */
#define SERIAL_BAUD  115200
#define TRACE_ROUTE       1   /* print one line at every waypoint             */

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
#define GRIP_BACKOFF_DEG  3

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
 *  The numbers below are safe starting guesses, not measurements.
 * ===================================================================== */
#define CAL_VALUES_FROM_RUN   0          /* 0 = never calibrated              */
#define CAL_LINE_LOW          0          /* 1 = black reads LOW on your bar   */
#define CAL_LO   { 250, 250, 250, 250, 250, 250, 250, 250 }  /* white readings */
#define CAL_HI   { 750, 750, 750, 750, 750, 750, 750, 750 }  /* black readings */
#define CAL_DEAD_MASK         0x00       /* channels too weak to use          */
#define CAL_DB_LF            45          /* duty where each wheel starts:     */
#define CAL_DB_LR            45          /*   left fwd, left rev,             */
#define CAL_DB_RF            30          /*   right fwd, right rev            */
#define CAL_DB_RR            30          /* (guess: your left motor is weaker) */
#define CAL_TRIM_L         1000          /* per mille; the weaker motor is 1000 */
#define CAL_TRIM_R          800          /* guess from "left 270 vs right 360"  */
#define CAL_TRIM_LR        1000          /* the same, driving backward          */
#define CAL_TRIM_RR         800          /* (spins use one wheel backward)      */
#define CAL_V_CRUISE_X100  1500          /* cm/s x100 at MAX_SP                */
#define CAL_V_MOVE_X100     950          /* cm/s x100 at MOVE_SP, from rest    */
#define CAL_LAG_MOVE_MS     120          /* start-up delay of a move from rest */
#define CAL_T90_L_MS        850          /* ms per 90 degrees at TURN_SP       */
#define CAL_T90_R_MS        850
/* ---- end CAL VALUES ---- */

#endif
