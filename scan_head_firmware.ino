/*
 * NEMA23 + TB6600 + AS5600, ESP32 as a serial device for a Raspberry Pi
 * Slow, precise output-shaft positioning for a LiDAR scan head.
 * Closed-loop: AS5600 on the motor shaft verifies and continuously
 * corrects final position.
 *
 * Wiring:
 *   TB6600  PUL+ -> GPIO25   DIR+ -> GPIO26   ENA+ -> GPIO27  (all -'s to GND)
 *   AS5600  SDA  -> GPIO21   SCL  -> GPIO22   VCC -> 3V3
 *   PCNT:  GPIO25 -> GPIO33 (PCNT_PULSE_PIN), GPIO26 -> GPIO32 (PCNT_CTRL_PIN)
 *
 * MICROSTEPPING: 1/32, TB6600 DIP switches MUST match.
 *   Motor step 0.05625 deg, output step 0.00225 deg (8.1 arcsec).
 *   AS5600 raw: 0.0879 deg motor / 0.00352 deg output.
 *   AS5600 is the resolution bottleneck -- upgrade to AS5047P/AS5048A for more.
 *
 * Serial protocol (115200 baud, '\n' terminated):
 *   M<deg>      relative move, MOTOR-shaft degrees
 *   T<deg>      absolute move, OUTPUT-shaft degrees
 *   S<val>      max speed, steps/sec (clamped to 4000)
 *   A<val>      acceleration, steps/sec^2 (clamped to 4000)
 *   C<tol>,<n>  closed-loop tolerance (output deg) and max attempts
 *   G           get angle now
 *   R<ms>       stream ANGLE lines every <ms> ms (0 = stop)
 *   P           hardware vs software step count
 *   X           stop motion immediately
 *   E / D       enable / disable driver
 *   L<max> or L<min>,<max>   OUTPUT-shaft travel limit
 *   Z           auto-detect motor/encoder direction (small test move,
 *               result is saved to flash and reloaded on every boot --
 *               only needs to be run once per hardware configuration,
 *               ideally during commissioning or after rewiring)
 *   ?           print status
 *
 * L must be sent before any M/T. Zero point is wherever the mount was
 * at boot -- start each power-up from a known-safe position.
 */

#include <AccelStepper.h>
#include <Wire.h>
#include <Preferences.h>
#include "driver/pcnt.h"

// ---- Pin config ----
#define STEP_PIN   25
#define DIR_PIN    26
#define ENABLE_PIN 27
#define AS5600_SDA 21
#define AS5600_SCL 22

// ---- Motor / gearbox ----
const int  FULL_STEPS_PER_REV  = 200;
const int  MICROSTEPS          = 32;
const long STEPS_PER_MOTOR_REV = (long)FULL_STEPS_PER_REV * MICROSTEPS;
const float GEAR_RATIO         = 25.0f;

// ---- Output-shaft hard limit ----
bool  limitsSet         = false;
float limitMinOutputDeg = 0.0f;
float limitMaxOutputDeg = 0.0f;

// ---- Hard safety limits ----
const float MAX_SPEED_STEPS_PER_SEC  = 4000.0f;
const float MAX_ACCEL_STEPS_PER_SEC2 = 4000.0f;
const float DEFAULT_SPEED_STEPS_PER_SEC  = 1000.0f;
const float DEFAULT_ACCEL_STEPS_PER_SEC2 = 800.0f;

// ---- Closed-loop correction ----
float closedLoopToleranceDeg = 0.008f;
int   maxCorrectionAttempts  = 30;
const unsigned long CLOSED_LOOP_SETTLE_MS   = 60;
const unsigned long CLOSED_LOOP_POLL_MS     = 40;
const long          CL_MAX_CORRECTION_STEPS = 400;

bool  closedLoopActive       = false;
float closedLoopTargetOutDeg = 0.0f;
int   correctionAttempts     = 0;
unsigned long moveCompleteMs = 0;
bool  wasMoving              = false;
unsigned long lastClPollMs   = 0;

// ---- AS5600 ----
const uint8_t AS5600_ADDR        = 0x36;
const uint8_t AS5600_REG_STATUS  = 0x0B;
const uint8_t AS5600_REG_CONF    = 0x07;
const uint8_t AS5600_REG_ANGLE   = 0x0E;
const int     ANGLE_AVG_SAMPLES  = 8;

// AS5600 STATUS register (0x0B) bit meanings:
//   bit5 (0x20) MD - Magnet Detected  -> 1 = magnet properly sensed (GOOD)
//   bit4 (0x10) ML - AGC min overflow -> 1 = magnet too strong    (BAD)
//   bit3 (0x08) MH - AGC max overflow -> 1 = magnet too weak      (BAD)
const uint8_t AS5600_STATUS_MD = 0x20;
const uint8_t AS5600_STATUS_ML = 0x10;
const uint8_t AS5600_STATUS_MH = 0x08;

// ---- PCNT ----
#define PCNT_PULSE_PIN 33
#define PCNT_CTRL_PIN  32
const pcnt_unit_t PCNT_UNIT = PCNT_UNIT_0;
const int16_t PCNT_H_LIM = 30000;
const int16_t PCNT_L_LIM = -30000;
volatile int64_t hwPulseAccum = 0;
static portMUX_TYPE pcntMux = portMUX_INITIALIZER_UNLOCKED;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// ---- Direction auto-detect / persistence ----
Preferences prefs;
bool motorDirInverted = false;

bool driverEnabled = true;
float lastRawDeg = 0.0f;
float unwrappedDeg = 0.0f;
bool haveFirstReading = false;

unsigned long streamIntervalMs = 0;
unsigned long lastStreamMs = 0;

// ---------------------------------------------------------------------------
// AS5600
// ---------------------------------------------------------------------------
bool as5600ReadRawOnce(float &degOut) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_REG_ANGLE);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)AS5600_ADDR, 2) != 2) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  uint16_t raw = ((uint16_t)hi << 8) | lo;
  degOut = (raw * 360.0f) / 4096.0f;
  return true;
}

float angleDiffDeg(float from, float to) {
  float d = to - from;
  while (d >  180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

bool as5600ReadAveraged(float &degOut) {
  float sumSin = 0, sumCos = 0;
  int ok = 0;
  for (int i = 0; i < ANGLE_AVG_SAMPLES; i++) {
    float d;
    if (as5600ReadRawOnce(d)) {
      float rad = d * PI / 180.0f;
      sumSin += sin(rad);
      sumCos += cos(rad);
      ok++;
    }
    delayMicroseconds(200);
  }
  if (ok == 0) return false;
  float meanRad = atan2(sumSin / ok, sumCos / ok);
  float meanDeg = meanRad * 180.0f / PI;
  if (meanDeg < 0) meanDeg += 360.0f;
  degOut = meanDeg;
  return true;
}

// FIXED: MD (0x20) is set when the magnet IS correctly detected, so the
// error condition is the bit being CLEAR, not set. ML/MH (0x10, 0x08) were
// already correct (error = bit set).
bool as5600MagnetOk(uint8_t &statusOut) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_REG_STATUS);
  if (Wire.endTransmission(false) != 0) { statusOut = 0xFF; return false; }
  if (Wire.requestFrom((int)AS5600_ADDR, 1) != 1) { statusOut = 0xFF; return false; }
  uint8_t st = Wire.read();
  statusOut = st;
  if (!(st & AS5600_STATUS_MD)) return false; // no magnet detected
  if (st & AS5600_STATUS_ML)    return false; // magnet too strong
  if (st & AS5600_STATUS_MH)    return false; // magnet too weak
  return true;
}

void as5600ConfigureFilter() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_REG_CONF);
  Wire.write(0x03);
  Wire.write(0x00);
  Wire.endTransmission();
}

void updateAngleTracking() {
  float deg;
  if (!as5600ReadAveraged(deg)) return;
  if (!haveFirstReading) {
    lastRawDeg = deg;
    unwrappedDeg = deg;
    haveFirstReading = true;
    return;
  }
  float delta = angleDiffDeg(lastRawDeg, deg);
  unwrappedDeg += delta;
  lastRawDeg = deg;
}

float currentOutputDeg() {
  return unwrappedDeg / GEAR_RATIO;
}

// ---------------------------------------------------------------------------
// PCNT
// ---------------------------------------------------------------------------
static void IRAM_ATTR pcntIsrHandler(void *arg) {
  uint32_t status;
  pcnt_get_event_status(PCNT_UNIT, &status);
  portENTER_CRITICAL_ISR(&pcntMux);
  if (status & PCNT_EVT_H_LIM) hwPulseAccum += PCNT_H_LIM;
  if (status & PCNT_EVT_L_LIM) hwPulseAccum += PCNT_L_LIM;
  portEXIT_CRITICAL_ISR(&pcntMux);
}

void setupPcnt() {
  pcnt_config_t cfg = {};
  cfg.pulse_gpio_num = PCNT_PULSE_PIN;
  cfg.ctrl_gpio_num  = PCNT_CTRL_PIN;
  cfg.channel        = PCNT_CHANNEL_0;
  cfg.unit           = PCNT_UNIT;
  cfg.pos_mode       = PCNT_COUNT_INC;
  cfg.neg_mode       = PCNT_COUNT_DIS;
  cfg.hctrl_mode     = PCNT_MODE_KEEP;
  cfg.lctrl_mode     = PCNT_MODE_REVERSE;
  cfg.counter_h_lim  = PCNT_H_LIM;
  cfg.counter_l_lim  = PCNT_L_LIM;
  pcnt_unit_config(&cfg);

  pcnt_set_filter_value(PCNT_UNIT, 100);
  pcnt_filter_enable(PCNT_UNIT);

  pcnt_event_enable(PCNT_UNIT, PCNT_EVT_H_LIM);
  pcnt_event_enable(PCNT_UNIT, PCNT_EVT_L_LIM);
  pcnt_isr_service_install(0);
  pcnt_isr_handler_add(PCNT_UNIT, pcntIsrHandler, NULL);

  pcnt_counter_pause(PCNT_UNIT);
  pcnt_counter_clear(PCNT_UNIT);
  hwPulseAccum = 0;
  pcnt_counter_resume(PCNT_UNIT);
}

// FIXED: hwPulseAccum (written by the ISR) and the live counter register
// are now read together inside a critical section, so a limit-event
// interrupt firing between the two reads can no longer produce a
// transient off-by-PCNT_H_LIM glitch in the reported count.
int64_t readHwStepCount() {
  int16_t c = 0;
  int64_t accum;
  portENTER_CRITICAL(&pcntMux);
  pcnt_get_counter_value(PCNT_UNIT, &c);
  accum = hwPulseAccum;
  portEXIT_CRITICAL(&pcntMux);
  return accum + c;
}

// ---------------------------------------------------------------------------
// Direction auto-detect
// ---------------------------------------------------------------------------
void loadDirInversion() {
  prefs.begin("scanhead", true);
  motorDirInverted = prefs.getBool("dirinv", false);
  prefs.end();
  stepper.setPinsInverted(motorDirInverted, false, false);
}

void saveDirInversion(bool inv) {
  prefs.begin("scanhead", false);
  prefs.putBool("dirinv", inv);
  prefs.end();
}

// Blocking move used only by the direction self-test. It runs outside the
// normal loop(), so it re-implements the limit watchdog inline -- it must
// never be able to drive past a configured travel limit.
bool runBoundedTestMove(long steps) {
  stepper.move(steps);
  unsigned long lastUpd = millis();
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    unsigned long now = millis();
    if (now - lastUpd >= 20) {
      updateAngleTracking();
      lastUpd = now;
      if (limitsSet) {
        float outDeg = currentOutputDeg();
        if (outDeg < limitMinOutputDeg || outDeg > limitMaxOutputDeg) {
          stepper.stop();
          stepper.setCurrentPosition(stepper.currentPosition());
          return false;
        }
      }
    }
  }
  return true;
}

// Commands a small test move and checks whether the AS5600 angle moved the
// way a positive step count should. If not, flips the DIR pin polarity
// (which also keeps the PCNT hardware counter consistent, since it shares
// the DIR line) and persists the result to flash.
//
// Return codes:
//    1  already correct, nothing changed
//    0  was inverted, corrected and saved
//   -1  no reliable motion measured (mechanical/encoder fault, not a
//       direction problem)
//   -2  aborted for safety (no room within configured limits, or a limit
//       was hit mid-test)
int detectMotorDirection() {
  const long  testSteps  = 800;  // ~45 deg motor / ~1.8 deg output at default gearing
  const float MIN_MOVE_DEG = 1.0f; // motor-shaft degrees; well above AS5600 noise floor

  if (!driverEnabled) setEnabled(true);

  updateAngleTracking();
  delay(50);
  updateAngleTracking();
  float startDeg = unwrappedDeg;

  long dir = 1;
  if (limitsSet) {
    float curOut = currentOutputDeg();
    float testOutputDeg = fabs((testSteps / (float)STEPS_PER_MOTOR_REV) * 360.0f / GEAR_RATIO);
    bool roomUp   = (curOut + testOutputDeg) <= limitMaxOutputDeg;
    bool roomDown = (curOut - testOutputDeg) >= limitMinOutputDeg;
    if (!roomUp && !roomDown) return -2;
    dir = roomUp ? 1 : -1;
  }

  bool completed = runBoundedTestMove(dir * testSteps);
  delay(CLOSED_LOOP_SETTLE_MS);
  updateAngleTracking();
  float measuredDelta = unwrappedDeg - startDeg;

  // Always return to the starting position, whether the outbound leg
  // finished cleanly or was cut short by a limit.
  long stepsToUndo = lround(((startDeg - unwrappedDeg) / 360.0f) * STEPS_PER_MOTOR_REV);
  runBoundedTestMove(stepsToUndo);
  delay(CLOSED_LOOP_SETTLE_MS);
  updateAngleTracking();

  if (!completed) return -2;
  if (fabs(measuredDelta) < MIN_MOVE_DEG) return -1;

  bool commandedPositive = (dir > 0);
  bool measuredPositive  = (measuredDelta > 0);
  bool mismatched = (commandedPositive != measuredPositive);

  if (mismatched) {
    motorDirInverted = !motorDirInverted;
    stepper.setPinsInverted(motorDirInverted, false, false);
  }
  saveDirInversion(motorDirInverted);
  return mismatched ? 0 : 1;
}

// ---------------------------------------------------------------------------
// Motion
// ---------------------------------------------------------------------------
void setEnabled(bool en) {
  driverEnabled = en;
  digitalWrite(ENABLE_PIN, en ? LOW : HIGH);
}

void sendAngleLine() {
  Serial.print("ANGLE,");
  Serial.print(lastRawDeg, 3);
  Serial.print(",");
  Serial.print(unwrappedDeg, 3);
  Serial.print(",");
  Serial.print(currentOutputDeg(), 4);
  Serial.print(",");
  Serial.println(stepper.currentPosition());
}

float issueClosedLoopMoveToOutputTarget(float rawTargetOutDeg) {
  float clampedTarget = constrain(rawTargetOutDeg, limitMinOutputDeg, limitMaxOutputDeg);
  float currentOut = currentOutputDeg();
  float outputDelta = clampedTarget - currentOut;
  float motorDeg = outputDelta * GEAR_RATIO;
  long steps = lround((motorDeg / 360.0f) * STEPS_PER_MOTOR_REV);

  stepper.move(steps);
  closedLoopTargetOutDeg = clampedTarget;
  closedLoopActive = true;
  correctionAttempts = 0;
  lastClPollMs = millis();
  return clampedTarget;
}

// ---------------------------------------------------------------------------
// Command handling
// ---------------------------------------------------------------------------
void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  char cmd = line.charAt(0);
  String arg = line.substring(1);
  arg.trim();

  switch (cmd) {
    case 'M': {
      if (!limitsSet) { Serial.println("ERR,LIMITS_NOT_SET"); break; }
      float deg = arg.toFloat();
      float rawTargetOut = currentOutputDeg() + (deg / GEAR_RATIO);
      float clamped = issueClosedLoopMoveToOutputTarget(rawTargetOut);
      Serial.print("OK,MOVE,");
      Serial.print(clamped, 4);
      if (fabs(clamped - rawTargetOut) > 0.0001f) Serial.print(",CLAMPED");
      Serial.println();
      break;
    }
    case 'T': {
      if (!limitsSet) { Serial.println("ERR,LIMITS_NOT_SET"); break; }
      float rawTargetOut = arg.toFloat();
      float clamped = issueClosedLoopMoveToOutputTarget(rawTargetOut);
      Serial.print("OK,TARGET,");
      Serial.print(clamped, 4);
      if (fabs(clamped - rawTargetOut) > 0.0001f) Serial.print(",CLAMPED");
      Serial.println();
      break;
    }
    case 'C': {
      int commaIdx = arg.indexOf(',');
      if (commaIdx < 0) { Serial.println("ERR,BAD_C"); break; }
      float tol = arg.substring(0, commaIdx).toFloat();
      int   n   = arg.substring(commaIdx + 1).toInt();
      if (tol <= 0 || n <= 0) { Serial.println("ERR,BAD_C"); break; }
      closedLoopToleranceDeg = tol;
      maxCorrectionAttempts  = n;
      Serial.print("OK,CLOSEDLOOP,");
      Serial.print(closedLoopToleranceDeg, 4);
      Serial.print(",");
      Serial.println(maxCorrectionAttempts);
      break;
    }
    case 'L': {
      float minD, maxD;
      int commaIdx = arg.indexOf(',');
      if (commaIdx >= 0) {
        minD = arg.substring(0, commaIdx).toFloat();
        maxD = arg.substring(commaIdx + 1).toFloat();
      } else {
        maxD = fabs(arg.toFloat());
        minD = -maxD;
      }
      if (minD >= maxD) { Serial.println("ERR,BAD_LIMIT"); break; }
      limitMinOutputDeg = minD;
      limitMaxOutputDeg = maxD;
      limitsSet = true;
      Serial.print("OK,LIMIT,");
      Serial.print(limitMinOutputDeg, 3);
      Serial.print(",");
      Serial.println(limitMaxOutputDeg, 3);
      break;
    }
    case 'S': {
      float v = constrain(arg.toFloat(), 1.0f, MAX_SPEED_STEPS_PER_SEC);
      stepper.setMaxSpeed(v);
      Serial.print("OK,SPEED,");
      Serial.println(v, 1);
      break;
    }
    case 'A': {
      float v = constrain(arg.toFloat(), 1.0f, MAX_ACCEL_STEPS_PER_SEC2);
      stepper.setAcceleration(v);
      Serial.print("OK,ACCEL,");
      Serial.println(v, 1);
      break;
    }
    case 'G':
      updateAngleTracking();
      sendAngleLine();
      break;
    case 'R': {
      long ms = arg.toInt();
      streamIntervalMs = (ms > 0) ? (unsigned long)ms : 0;
      Serial.print("OK,STREAM,");
      Serial.println(streamIntervalMs);
      break;
    }
    case 'X':
      stepper.stop();
      stepper.setCurrentPosition(stepper.currentPosition());
      closedLoopActive = false;
      Serial.println("OK,STOP");
      break;
    case 'E':
      setEnabled(true);
      Serial.println("OK,ENABLED");
      break;
    case 'D':
      setEnabled(false);
      Serial.println("OK,DISABLED");
      break;
    case 'Z': {
      int r = detectMotorDirection();
      switch (r) {
        case 1:  Serial.println("OK,DIR_DETECT,ALREADY_CORRECT"); break;
        case 0:  Serial.println("OK,DIR_DETECT,INVERTED_CORRECTED"); break;
        case -1: Serial.println("ERR,DIR_DETECT,NO_MOTION"); break;
        case -2: Serial.println("ERR,DIR_DETECT,NO_ROOM_OR_LIMIT_HIT"); break;
      }
      break;
    }
    case 'P': {
      int64_t hw = readHwStepCount();
      long sw = stepper.currentPosition();
      Serial.print("PCNT,");
      Serial.print((long)hw);
      Serial.print(",");
      Serial.print(sw);
      Serial.print(",");
      Serial.println((long)hw - sw);
      break;
    }
    case '?':
      Serial.print("STATUS,speed_max=");
      Serial.print(stepper.maxSpeed(), 1);
      Serial.print(",enabled=");
      Serial.print(driverEnabled ? 1 : 0);
      Serial.print(",limits_set=");
      Serial.print(limitsSet ? 1 : 0);
      if (limitsSet) {
        Serial.print(",limit_min=");
        Serial.print(limitMinOutputDeg, 3);
        Serial.print(",limit_max=");
        Serial.print(limitMaxOutputDeg, 3);
      }
      Serial.print(",cl_tol=");
      Serial.print(closedLoopToleranceDeg, 4);
      Serial.print(",cl_max_attempts=");
      Serial.print(maxCorrectionAttempts);
      Serial.print(",cl_active=");
      Serial.print(closedLoopActive ? 1 : 0);
      Serial.print(",dir_inverted=");
      Serial.println(motorDirInverted ? 1 : 0);
      break;
    default:
      Serial.print("ERR,");
      Serial.println(line);
  }
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) { }

  pinMode(ENABLE_PIN, OUTPUT);
  setEnabled(true);

  Wire.begin(AS5600_SDA, AS5600_SCL);
  Wire.setClock(400000);

  as5600ConfigureFilter();
  delay(10);
  uint8_t st = 0;
  if (!as5600MagnetOk(st)) {
    Serial.print("ERR,AS5600_STATUS,0x");
    Serial.println(st, HEX);
  }

  setupPcnt();

  stepper.setMaxSpeed(DEFAULT_SPEED_STEPS_PER_SEC);
  stepper.setAcceleration(DEFAULT_ACCEL_STEPS_PER_SEC2);
  stepper.setCurrentPosition(0);
  loadDirInversion(); // apply any direction correction saved by a previous 'Z'

  updateAngleTracking();
  Serial.println("READY,nema23_tb6600_as5600_serial_closedloop");
}

void loop() {
  stepper.run();

  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleCommand(line);
  }

  bool isMoving = (stepper.distanceToGo() != 0);
  if (wasMoving && !isMoving) {
    moveCompleteMs = millis();
    lastClPollMs   = moveCompleteMs;
  }
  wasMoving = isMoving;

  static unsigned long lastAngleUpdateMs = 0;
  unsigned long now = millis();
  if (now - lastAngleUpdateMs >= 20) {
    updateAngleTracking();
    lastAngleUpdateMs = now;

    if (limitsSet) {
      float outDeg = currentOutputDeg();
      if (outDeg < limitMinOutputDeg || outDeg > limitMaxOutputDeg) {
        stepper.stop();
        stepper.setCurrentPosition(stepper.currentPosition());
        closedLoopActive = false;
        Serial.print("LIMIT,STOP,");
        Serial.println(outDeg, 4);
      }
    }
  }

  if (closedLoopActive && !isMoving &&
      (now - moveCompleteMs >= CLOSED_LOOP_SETTLE_MS) &&
      (now - lastClPollMs   >= CLOSED_LOOP_POLL_MS)) {
    lastClPollMs = now;
    float error  = closedLoopTargetOutDeg - currentOutputDeg();
    float absErr = fabs(error);

    if (absErr <= closedLoopToleranceDeg) {
      closedLoopActive = false;
      Serial.print("OK,SETTLED,");
      Serial.print(currentOutputDeg(), 4);
      Serial.print(",err=");       Serial.print(error, 4);
      Serial.print(",attempts=");  Serial.println(correctionAttempts);

    } else if (correctionAttempts >= maxCorrectionAttempts) {
      closedLoopActive = false;
      Serial.print("OK,GAVEUP,");
      Serial.print(currentOutputDeg(), 4);
      Serial.print(",err=");       Serial.print(error, 4);
      Serial.print(",attempts=");  Serial.println(correctionAttempts);

    } else {
      correctionAttempts++;
      float correctionMotorDeg = error * GEAR_RATIO;
      long correctionSteps = lround((correctionMotorDeg / 360.0f) * STEPS_PER_MOTOR_REV);
      if (correctionSteps >  CL_MAX_CORRECTION_STEPS) correctionSteps =  CL_MAX_CORRECTION_STEPS;
      if (correctionSteps < -CL_MAX_CORRECTION_STEPS) correctionSteps = -CL_MAX_CORRECTION_STEPS;

      if (correctionSteps != 0) {
        stepper.move(correctionSteps);
        Serial.print("CORRECT,");
        Serial.print(correctionAttempts);
        Serial.print(",err=");   Serial.print(error, 4);
        Serial.print(",steps="); Serial.println(correctionSteps);
      } else {
        closedLoopActive = false;
        Serial.print("OK,SETTLED,");
        Serial.print(currentOutputDeg(), 4);
        Serial.print(",err=");   Serial.print(error, 4);
        Serial.println(",steps=0");
      }
    }
  }

  if (streamIntervalMs > 0 && (now - lastStreamMs >= streamIntervalMs)) {
    sendAngleLine();
    lastStreamMs = now;
  }
}
