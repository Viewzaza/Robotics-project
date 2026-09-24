/* hal.h -- sensing, odometry and motion primitives.
 *
 * Everything here is deliberately non-blocking-with-a-timeout: no loop in this
 * file can spin forever, because a robot frozen mid-field with its wheels
 * braked cannot be recovered by anything except a human.
 */
#ifndef HAL_H
#define HAL_H

#include <Servo.h>
#include <EEPROM.h>
#include "config.h"
#include "pidLibrary.h"

/* ===================================================================== state */

Servo servo_x;          /* gripper */
Servo servo_y;          /* arm     */

static const uint8_t SENSOR_PIN[8] = { A0, A1, A2, A3, A4, A5, A6, A7 };

struct Calib {
  uint32_t magic;
  uint8_t  version;
  uint16_t lo[8];          /* raw reading over the pale background */
  uint16_t hi[8];          /* raw reading over the tape            */
  float    cmPerSAtCal;    /* forward speed at DUTY_CAL            */
  uint16_t msPer90;        /* pivot time for 90 degrees            */
  int8_t   trimRPct;       /* right-motor trim, percent offset     */
  uint8_t  checksum;
};

static Calib g_cal;
static bool  g_calValid = false;

/* commanded state, kept so odometry can integrate speed over time */
static int16_t  g_dutyL = 0, g_dutyR = 0;   /* signed: negative = reverse */
static float    g_odoCm = 0;                /* monotonic path length      */
static uint32_t g_odoLastUs = 0;

static uint8_t  g_mask = 0;                 /* bit7 = A0 (left) .. bit0 = A7 */
static int16_t  g_pos = 0;                  /* -1000 (hard left) .. +1000    */
static bool     g_lineSeen = false;

static int16_t  g_servoXNow = SERVO_GRIP_OPEN;
static int16_t  g_servoYNow = SERVO_ARM_DOWN;

static const char *g_fault = 0;             /* set on any timeout */


/* ================================================== ATmega328P fast paths ===
 * The control loop was dominated by eight analogRead() calls. Arduino leaves
 * the ADC prescaler at 128, giving a 125 kHz ADC clock and ~104 us per
 * conversion: 832 us just to read the bar, before any thinking. At 22 cm/s the
 * robot moves 0.26 mm between samples, and that sampling granularity is what
 * limits how precisely a junction can be timed.
 *
 * Prescaler 16 gives a 1 MHz ADC clock and ~13 us per conversion - the eight
 * channels drop from 832 us to about 110 us. The datasheet quotes full 10-bit
 * accuracy only up to 200 kHz, so this costs roughly 2 LSB; irrelevant when the
 * decision is "is this reading above a threshold" with ~700 counts of contrast.
 *
 * Direction pins are written straight to PORTB/PORTD. digitalWrite() is about
 * 50 cycles of pin-map lookup and interrupt masking; a port write is 2.
 *
 * All of it is guarded so the host simulator still builds and still exercises
 * exactly the same logic through the portable path.
 */
#if defined(__AVR__)
  #include <avr/io.h>

  static inline void fastIoInit() {
    ADCSRA = (uint8_t)((ADCSRA & ~0x07) | ADC_PRESCALER);
    /* Timer2 (pin D3) defaults to a prescaler of 64 -> 490 Hz, while Timer0
     * (pin D6) runs at 976 Hz. The two motors were being driven at different
     * PWM frequencies. Prescaler 32 puts D3 on 980 Hz to match D6. */
    TCCR2B = (uint8_t)((TCCR2B & 0xF8) | 0x03);
  }

  static inline uint16_t adcRead(uint8_t ch) {
    ADMUX = (uint8_t)((1 << REFS0) | (ch & 0x0F));
    ADCSRA |= (uint8_t)(1 << ADSC);
    while (ADCSRA & (1 << ADSC)) { }
    return ADC;
  }

  /* D12=PB4  D11=PB3  D8=PB0  D10=PB2   D7=PD7 */
  #define WR_FL(v) do { if (v) PORTB |= _BV(4); else PORTB &= (uint8_t)~_BV(4); } while (0)
  #define WR_BL(v) do { if (v) PORTB |= _BV(3); else PORTB &= (uint8_t)~_BV(3); } while (0)
  #define WR_BR(v) do { if (v) PORTB |= _BV(0); else PORTB &= (uint8_t)~_BV(0); } while (0)
  #define WR_FR(v) do { if (v) PORTD |= _BV(7); else PORTD &= (uint8_t)~_BV(7); } while (0)
#else
  static inline void fastIoInit() {}
  static inline uint16_t adcRead(uint8_t ch) { return (uint16_t)analogRead(A0 + ch); }
  #define WR_FL(v) digitalWrite(F_L, (v))
  #define WR_BL(v) digitalWrite(B_L, (v))
  #define WR_BR(v) digitalWrite(B_R, (v))
  #define WR_FR(v) digitalWrite(F_R, (v))
#endif

/* =============================================================== calibration */

static uint8_t calChecksum(const Calib &c) {
  const uint8_t *p = (const uint8_t *)&c;
  uint8_t s = 0x5A;
  for (unsigned i = 0; i < sizeof(Calib) - 1; i++) s = (uint8_t)(s * 31 + p[i]);
  return s;
}

static void calDefaults() {
  g_cal.magic = EE_MAGIC;
  g_cal.version = EE_VERSION;
  for (uint8_t i = 0; i < 8; i++) { g_cal.lo[i] = 120; g_cal.hi[i] = 880; }
  g_cal.cmPerSAtCal = CM_PER_S_AT_CAL;
  g_cal.msPer90 = MS_PER_90DEG;
  g_cal.trimRPct = TRIM_R_PCT - 100;
  g_cal.checksum = calChecksum(g_cal);
}

static bool calLoad() {
  EEPROM.get(EE_ADDR_CAL, g_cal);
  if (g_cal.magic == EE_MAGIC && g_cal.version == EE_VERSION &&
      g_cal.checksum == calChecksum(g_cal) && g_cal.cmPerSAtCal > 1.0f) {
    g_calValid = true;
    return true;
  }
  calDefaults();
  g_calValid = false;
  return false;
}

static void calSave() {
  g_cal.magic = EE_MAGIC;
  g_cal.version = EE_VERSION;
  g_cal.checksum = calChecksum(g_cal);
  EEPROM.put(EE_ADDR_CAL, g_cal);
  g_calValid = true;
}

/* Per-channel threshold, midway between the calibrated background and tape.
 * A fixed 500 for all eight channels is wrong on a real bar: LED brightness
 * and phototransistor gain vary channel to channel, and both drift with
 * ambient light and battery voltage. */
static inline uint16_t threshOf(uint8_t i) {
  if (!g_calValid) return THRESH_FALLBACK;
  uint16_t lo = g_cal.lo[i], hi = g_cal.hi[i];
  if (hi <= lo + 60) return THRESH_FALLBACK;   /* channel looks dead */
  return (uint16_t)((lo + hi) / 2);
}

/* ==================================================================== speed */

/* Linear model through the dead band: below DUTY_DEADBAND the gearmotors do
 * not turn at all, so speed is NOT proportional to duty from zero. */
static float cmPerS(int duty) {
  if (duty <= DUTY_DEADBAND) return 0.0f;
  float span = (float)(DUTY_CAL - DUTY_DEADBAND);
  return g_cal.cmPerSAtCal * ((float)(duty - DUTY_DEADBAND) / span);
}

static uint32_t msForCm(float cm, int duty) {
  float v = cmPerS(duty);
  if (v < 0.5f) return 0;
  return (uint32_t)((cm / v) * 1000.0f);
}

/* ================================================================== odometry */

/* Integrates commanded speed over time. With no encoders this is open loop, so
 * it is only trusted over short distances -- one cell at most -- and is reset
 * at every junction, which is a true position fix. */
static void odoTick() {
  uint32_t now = micros();
  uint32_t dt = now - g_odoLastUs;
  g_odoLastUs = now;
  if (dt > 200000ul) return;                  /* first call or a long stall */
  int16_t fwd = (g_dutyL + g_dutyR) / 2;
  if (fwd < 0) fwd = -fwd;
  g_odoCm += cmPerS(fwd) * (dt * 1e-6f);
}

/* ==================================================== online scale estimate ==
 * The odometer is open loop: it integrates the speed the CALIBRATION says the
 * commanded duty produces. If the real speed differs -- a fresh battery, a cold
 * gearbox, a different surface -- then every distance is wrong by that factor,
 * and so is every timed pivot, because MS_PER_90DEG is a time and the angle it
 * sweeps is proportional to wheel speed. A 25 percent slow battery turns a
 * 90 degree turn into a 66 degree one, which is unrecoverable.
 *
 * measureScale() is handed the distance the odometer THINKS the robot rolled
 * while the bar was completely over a crossing line. The truth is the tape
 * width. The ratio is the scale error, and correcting cmPerSAtCal and msPer90
 * by it fixes the distances and the turns in one step.
 *
 * It is self-extinguishing: once the calibration is right the next measurement
 * reads LINE_WIDTH_CM and asks for no change, so it can be left running for the
 * whole mission and will track a battery that sags as it goes.
 */
static uint8_t g_scaleFixes = 0;
static float   g_scaleTotal = 1.0f;     /* cumulative correction, for telemetry */

static void measureScale(float dwellCm) {
  if (dwellCm < LINE_WIDTH_CM * SCALE_MEAS_MIN) return;
  if (dwellCm > LINE_WIDTH_CM * SCALE_MEAS_MAX) return;

  float s = dwellCm / LINE_WIDTH_CM;              /* odometer over-reads by s */
  float gain = (g_scaleFixes == 0) ? SCALE_GAIN_FIRST : SCALE_GAIN_LATER;
  float corr = 1.0f + (s - 1.0f) * gain;
  if (corr < 0.5f) corr = 0.5f;
  if (corr > 2.0f) corr = 2.0f;

  float total = g_scaleTotal * corr;
  if (total < SCALE_CLAMP_LO || total > SCALE_CLAMP_HI) return;
  g_scaleTotal = total;

  /* odometer reads high by corr  ->  it believes the robot is faster than it
   * is  ->  divide the speed constant; and the true time for 90 degrees is
   * longer by the same factor. */
  g_cal.cmPerSAtCal /= corr;
  uint32_t m = (uint32_t)((float)g_cal.msPer90 * corr);
  if (m < 150ul)  m = 150ul;
  if (m > 3000ul) m = 3000ul;
  g_cal.msPer90 = (uint16_t)m;
  g_scaleFixes++;

#if !defined(__AVR__) && defined(SIM_DEBUG)
  printf("      SCALE dwell=%.2f corr=%.3f total=%.3f cmps=%.2f ms90=%d\n",
         (double)dwellCm, (double)corr, (double)g_scaleTotal,
         (double)g_cal.cmPerSAtCal, (int)g_cal.msPer90);
#endif
}

/* ==================================================================== motors */

static void motorsRaw(int16_t l, int16_t r) {
  g_dutyL = l;
  g_dutyR = r;
  if (l > 0)      { WR_FL(1); WR_BL(0); analogWrite(sp_L, l); }
  else if (l < 0) { WR_FL(0); WR_BL(1); analogWrite(sp_L, -l); }
  else            { WR_FL(1); WR_BL(1); analogWrite(sp_L, 0); }
  if (r > 0)      { WR_FR(1); WR_BR(0); analogWrite(sp_R, r); }
  else if (r < 0) { WR_FR(0); WR_BR(1); analogWrite(sp_R, -r); }
  else            { WR_FR(1); WR_BR(1); analogWrite(sp_R, 0); }
}

/* Applies the right-motor trim and clamps. Duties between 1 and the dead band
 * are pushed up to DUTY_MIN_MOVE: commanding 20 just makes the motor buzz and
 * heat while the robot does not move. */
static int16_t shape(int16_t d) {
  if (d == 0) return 0;
  int16_t s = d < 0 ? -1 : 1;
  int16_t m = d < 0 ? -d : d;
  if (m > 255) m = 255;
  if (m < DUTY_MIN_MOVE) m = DUTY_MIN_MOVE;
  return (int16_t)(s * m);
}

static void motors(int16_t l, int16_t r) {
  long rr = (long)r * (100 + g_cal.trimRPct) / 100;
  motorsRaw(shape(l), shape((int16_t)rr));
}

static void brake() { motorsRaw(0, 0); }

static void driveStraight(int16_t duty) { motors(duty, duty); }
static void spin(int8_t dir, int16_t duty) {   /* dir +1 = clockwise / right */
  if (dir > 0) motors(duty, -duty);
  else         motors(-duty, duty);
}

/* =================================================================== sensing */

/* Reads all eight channels into a bitmask and computes a weighted centroid.
 * The centroid gives a continuous error instead of the 15 discrete steps the
 * pattern lookup produced, which is what lets the PID run smoothly. */
static uint8_t readMask() {
  odoTick();
  uint8_t m = 0;
  long num = 0;
  int8_t n = 0;
  for (uint8_t i = 0; i < 8; i++) {
    uint16_t v = adcRead(i);
    uint16_t t = threshOf(i);
    bool was = (g_mask >> (7 - i)) & 1;
    bool on  = was ? (v > (uint16_t)(t - THRESH_HYST))    /* hysteresis: */
                   : (v > (uint16_t)(t + THRESH_HYST));   /* no chatter  */
    if (on) {
      m |= (uint8_t)(1 << (7 - i));
      num += ((int)i - 3) * 2 + 1;   /* -7,-5,-3,-1,+1,+3,+5,+7 */
      n++;
    }
  }
  g_mask = m;
  g_lineSeen = (n > 0);
  if (n > 0) g_pos = (int16_t)((num * 1000L) / (7L * n));
  return m;
}

static inline uint8_t bitCount(uint8_t m) {
  uint8_t c = 0;
  while (m) { c += (uint8_t)(m & 1); m >>= 1; }
  return c;
}

/* A crossing line lies across the bar, so many sensors read at once. */
static bool isJunction(uint8_t m) { return bitCount(m) >= JUNCTION_MIN_BITS; }

/* Which side branches are present, sampled at the moment of the junction.
 * Lets the mission layer assert that it is where it thinks it is. */
#define BRANCH_LEFT   0x01
#define BRANCH_RIGHT  0x02
static uint8_t branchesOf(uint8_t m) {
  uint8_t b = 0;
  if (m & 0xE0) b |= BRANCH_LEFT;    /* A0..A2 */
  if (m & 0x07) b |= BRANCH_RIGHT;   /* A5..A7 */
  return b;
}

/* Compatibility shim: the course slides talk in 8-character strings. Nothing
 * on the hot path uses this any more -- building an Arduino String eight times
 * per control cycle fragments a 2 KB heap until the robot dies silently. */
static String maskToString(uint8_t m) {
  String s = "";
  for (int8_t i = 7; i >= 0; i--) s += ((m >> i) & 1) ? "1" : "0";
  return s;
}
static String getSensor() { return maskToString(readMask()); }

/* ================================================================== servos */

/* Steps the servo to its target instead of commanding it in one jump. An
 * unrestrained sweep draws enough inrush to sag the shared 5 V rail and reset
 * the Nano; stepping spreads that over tens of milliseconds. */
static void servoTo(Servo &s, int16_t &now, int16_t target) {
  while (now != target) {
    int16_t d = target - now;
    if (d > SERVO_STEP_DEG) d = SERVO_STEP_DEG;
    if (d < -SERVO_STEP_DEG) d = -SERVO_STEP_DEG;
    now += d;
    s.write(now);
    delay(SERVO_STEP_MS);
  }
}
static void gripTo(int16_t a) { servoTo(servo_x, g_servoXNow, a); }
static void armTo(int16_t a)  { servoTo(servo_y, g_servoYNow, a); }

/* ================================================================== motion */

/* Drive a fixed distance. The whole point of the 9.5 cm measurement: when the
 * bar reports a junction, the pivot centre is SENSOR_AHEAD_CM short of it, so
 * advanceCm(SENSOR_AHEAD_CM) puts the pivot exactly on the crossing.
 *
 * Always driven at DUTY_PRECISE rather than whatever the ramp had reached, so
 * the distance is repeatable. The old code advanced for a fixed 50 ms at a
 * speed that could be anywhere from 50 to 255 duty -- a 5x spread in distance. */
static void advanceCm(float cm, int16_t duty = DUTY_PRECISE) {
  if (cm <= 0.05f) return;
  uint32_t ms = msForCm(cm, duty);
  if (ms == 0) return;
  uint32_t t0 = millis();
  driveStraight(duty);
  while ((uint32_t)(millis() - t0) < ms) {
    /* Deliberately open loop. Steering off the centroid here was measured to
     * HURT: within SENSOR_AHEAD_CM of a junction the bar clips the crossing
     * line, the centroid swings toward it, and the robot steers into the
     * crossing instead of through it. Mission completion across the simulator
     * sweep fell from 267/315 to 151/315 with correction enabled. */
    odoTick();
    if ((uint32_t)(millis() - t0) > ms + TIMEOUT_LEG_MS) break;
  }
  brake();
  delay(40);                       /* let the chassis settle before pivoting */
}

static void reverseCm(float cm, int16_t duty = DUTY_PRECISE) {
  if (cm <= 0.05f) return;
  uint32_t ms = msForCm(cm, duty);
  uint32_t t0 = millis();
  motors(-duty, -duty);
  while ((uint32_t)(millis() - t0) < ms) odoTick();
  brake();
  delay(40);
}

/* Give back a short advance. Same duty as advanceCm(), so the same speed model
 * applies in both directions and the error cancels: whatever the speed
 * calibration is wrong by, creepBack(d) undoes advanceCm(d) exactly. */
static void creepBack(float cm) {
  if (cm > 0.05f) reverseCm(cm);
}

/* Pivot on the spot through `deg` degrees, finishing CENTRED on the line it
 * finds. Two phases:
 *   A. spin open-loop for most of the expected time, so the line we started on
 *      cannot be mistaken for the line we are looking for;
 *   B. keep spinning, slower, until the bar sees a line near its centre.
 * If phase B never finds a line, fall back to the open-loop estimate and raise
 * a fault rather than spinning forever -- which is exactly what the original
 * while(true) turn primitives did when they missed their exit pattern. */
static bool pivotDeg(int8_t dir, int16_t deg) {
  uint32_t full   = (uint32_t)g_cal.msPer90 * (uint32_t)deg / 90ul;
  uint32_t coarse = (full * 85ul) / 100ul;   /* blind through most of the turn */
  uint32_t hard   = (full * 118ul) / 100ul;  /* never spin past this          */
  uint32_t t0 = millis();

  /* Phase A: spin open-loop. Going blind for most of the sweep matters because
   * the bar traces a circle of radius SENSOR_AHEAD_CM about the pivot, so it
   * clips the lines it is NOT looking for partway round. Only start looking
   * once we are near the expected angle. */
  spin(dir, DUTY_PIVOT);
  while ((uint32_t)(millis() - t0) < coarse) {
    readMask();
    if ((uint32_t)(millis() - t0) > TIMEOUT_PIVOT_MS) { brake(); g_fault = "pivot A"; return false; }
  }

  /* Phase B: creep round until the line sits near the MIDDLE of the bar, which
   * only happens when the robot is square to it. Two consecutive confirmations,
   * so a single sample taken as the bar clips a line does not end the turn. */
  spin(dir, DUTY_PIVOT - 25);
  uint8_t hits = 0;
  bool found = false;
  while ((uint32_t)(millis() - t0) < hard) {
    uint8_t m = readMask();
    int16_t p = g_pos < 0 ? -g_pos : g_pos;
    if (m && bitCount(m) <= 4 && p <= 150) {
      if (++hits >= 2) { found = true; break; }
    } else {
      hits = 0;
    }
  }
  brake();
  delay(60);
  /* Not finding the line is NOT a fault: the open-loop estimate has already put
   * us within about +-18 percent of the target angle, which the line follower
   * absorbs on the next straight. Widening the window instead would let the
   * turn terminate on the WRONG line -- the bar sweeps a circle of radius
   * SENSOR_AHEAD_CM about the pivot, so it crosses every line at the junction,
   * not just the one we want. */
  return found;
}

/* Re-acquire the line by sweeping if it is lost. The original code simply left
 * the motors at their last command when no pattern matched, which drives the
 * robot away from the line at whatever speed it was doing. */
static bool searchLine() {
  uint32_t t0 = millis();
  int8_t dir = (g_pos >= 0) ? 1 : -1;      /* sweep toward where it went */
  for (uint8_t attempt = 0; attempt < 4; attempt++) {
    uint32_t span = TIMEOUT_SEARCH_MS / 4 * (attempt + 1);
    spin(dir, DUTY_PIVOT - 30);
    uint32_t s0 = millis();
    while ((uint32_t)(millis() - s0) < span) {
      if (readMask()) { brake(); return true; }
      if ((uint32_t)(millis() - t0) > TIMEOUT_SEARCH_MS * 3) { brake(); g_fault = "line lost"; return false; }
    }
    dir = (int8_t)-dir;
  }
  brake();
  g_fault = "line lost";
  return false;
}

#endif
