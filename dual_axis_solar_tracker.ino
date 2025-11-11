/*
  Dual Axis Solar Tracker
  -------------------------------------------
  Hardware:
    - Arduino UNO
    - Two VNH2SP30 motor drivers controlling 12 V DC wiper motors
    - Four LDR sensors arranged in a cross pattern inside a light box

  Behaviour:
    - Calibrates light sensor baselines at startup (5 s averaging)
    - Computes adaptive dead zone from live brightness to avoid jitter
    - Uses proportional control bursts (~300 ms) to re-center when light shifts
    - Enters Night Mode when all sensors fall below the night threshold
    - Provides serial telemetry and simple manual jog commands

  Serial commands (115200 baud):
    r -> recalibrate sensors
    p -> print current telemetry snapshot
    m -> toggle manual jog mode (j/l = azimuth, i/k = elevation, x = stop, q = exit)
*/

#include <math.h>
#include <string.h>

// --- Pin assignments -------------------------------------------------------
const uint8_t PIN_LDR_TOP    = A0;
const uint8_t PIN_LDR_BOTTOM = A1;
const uint8_t PIN_LDR_RIGHT  = A2;
const uint8_t PIN_LDR_LEFT   = A3;

const uint8_t PIN_AZ_CS  = A4;
const uint8_t PIN_AZ_INA = 9;
const uint8_t PIN_AZ_INB = 10;
const uint8_t PIN_AZ_PWM = 5;

const uint8_t PIN_EL_CS  = A5;
const uint8_t PIN_EL_INA = 3;
const uint8_t PIN_EL_INB = 2;
const uint8_t PIN_EL_PWM = 6;

// --- Tracker configuration -------------------------------------------------
const unsigned long CALIBRATION_DURATION_MS = 5000UL;
const unsigned long SAMPLE_INTERVAL_MS      = 200UL;
const unsigned long MOVE_BURST_MS           = 300UL;
const uint8_t       SAMPLES_PER_READING     = 10;
const uint16_t      NIGHT_THRESHOLD         = 150;

const float   KP_GAIN   = 0.6f;
const uint8_t PWM_MIN   = 80;
const uint8_t PWM_MAX   = 220;

// Change these to invert motor directions if required by wiring.
const int AZIMUTH_LEFT_DIR     = 1;   // set to -1 if motion is reversed
const int AZIMUTH_RIGHT_DIR    = -AZIMUTH_LEFT_DIR;
const int ELEVATION_UP_DIR     = 1;   // set to -1 if motion is reversed
const int ELEVATION_DOWN_DIR   = -ELEVATION_UP_DIR;

// --- Sensor indexing -------------------------------------------------------
enum SensorIndex {
  SENSOR_TOP = 0,
  SENSOR_BOTTOM,
  SENSOR_RIGHT,
  SENSOR_LEFT,
  SENSOR_COUNT
};

const uint8_t SENSOR_PINS[SENSOR_COUNT] = {
  PIN_LDR_TOP,
  PIN_LDR_BOTTOM,
  PIN_LDR_RIGHT,
  PIN_LDR_LEFT
};

// --- Axis driver state -----------------------------------------------------
struct AxisDriver {
  uint8_t inA;
  uint8_t inB;
  uint8_t pwmPin;
  uint8_t csPin;
  int direction;           // -1, 0, +1
  int currentPwm;          // last PWM command (0-255)
  unsigned long stopTime;  // millis timestamp when burst should end
  bool manualHold;         // true when manual mode is commanding the axis
};

AxisDriver azimuth  = {PIN_AZ_INA, PIN_AZ_INB, PIN_AZ_PWM, PIN_AZ_CS, 0, 0, 0UL, false};
AxisDriver elevation = {PIN_EL_INA, PIN_EL_INB, PIN_EL_PWM, PIN_EL_CS, 0, 0, 0UL, false};

// --- Sensor snapshot -------------------------------------------------------
struct SensorSnapshot {
  uint16_t raw[SENSOR_COUNT];
  float adjusted[SENSOR_COUNT];
  float normalized[SENSOR_COUNT];
  float avgBrightness;
  float errorX;
  float errorY;
  float deadZone;
  bool night;
  uint16_t currentAz;
  uint16_t currentEl;
};

// --- Globals ----------------------------------------------------------------
float baseline[SENSOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
bool baselineReady = false;
bool manualMode = false;
bool nightMode = false;

SensorSnapshot lastSnapshot = {};
unsigned long lastSampleMillis = 0UL;

// --- Utility helpers -------------------------------------------------------
void appendAction(char *buffer, size_t length, const char *fragment) {
  if (!fragment || !length) {
    return;
  }
  size_t used = strlen(buffer);
  size_t fragmentLen = strlen(fragment);
  if (used == 0) {
    if (fragmentLen < length) {
      strcpy(buffer, fragment);
    }
    return;
  }
  if (used + 1 + fragmentLen >= length) {
    return;  // not enough room to append
  }
  buffer[used] = ' ';
  strcpy(buffer + used + 1, fragment);
}

bool axisAutoActive(const AxisDriver &axis, unsigned long now) {
  return (!axis.manualHold) && axis.direction != 0 && now < axis.stopTime;
}

void stopAxis(AxisDriver &axis) {
  analogWrite(axis.pwmPin, 0);
  digitalWrite(axis.inA, LOW);
  digitalWrite(axis.inB, LOW);
  axis.direction = 0;
  axis.currentPwm = 0;
  axis.stopTime = 0;
  axis.manualHold = false;
}

void driveAxisAuto(AxisDriver &axis, int direction, int pwm, unsigned long now) {
  pwm = constrain(pwm, 0, 255);
  if (pwm == 0 || direction == 0) {
    stopAxis(axis);
    return;
  }

  if (direction > 0) {
    digitalWrite(axis.inA, HIGH);
    digitalWrite(axis.inB, LOW);
    axis.direction = 1;
  } else {
    digitalWrite(axis.inA, LOW);
    digitalWrite(axis.inB, HIGH);
    axis.direction = -1;
  }

  analogWrite(axis.pwmPin, pwm);
  axis.currentPwm = pwm;
  axis.stopTime = now + MOVE_BURST_MS;
  axis.manualHold = false;
}

void driveAxisManual(AxisDriver &axis, int direction, int pwm) {
  pwm = constrain(pwm, PWM_MIN, PWM_MAX);
  if (pwm == 0 || direction == 0) {
    stopAxis(axis);
    return;
  }

  if (direction > 0) {
    digitalWrite(axis.inA, HIGH);
    digitalWrite(axis.inB, LOW);
    axis.direction = 1;
  } else {
    digitalWrite(axis.inA, LOW);
    digitalWrite(axis.inB, HIGH);
    axis.direction = -1;
  }

  analogWrite(axis.pwmPin, pwm);
  axis.currentPwm = pwm;
  axis.manualHold = true;
  axis.stopTime = 0;
}

void updateAxisTimer(AxisDriver &axis, unsigned long now) {
  if (axis.manualHold) {
    return;
  }
  if (axis.direction != 0 && now >= axis.stopTime) {
    stopAxis(axis);
  }
}

int computePwmFromError(float errorMagnitude) {
  if (errorMagnitude <= 0.0f) {
    return 0;
  }
  float pwmFloat = KP_GAIN * errorMagnitude * 255.0f;
  if (pwmFloat < PWM_MIN) {
    pwmFloat = PWM_MIN;
  }
  if (pwmFloat > PWM_MAX) {
    pwmFloat = PWM_MAX;
  }
  return static_cast<int>(pwmFloat + 0.5f);
}

void captureSensors(SensorSnapshot &snapshot) {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    uint32_t total = 0;
    for (uint8_t sample = 0; sample < SAMPLES_PER_READING; ++sample) {
      total += analogRead(SENSOR_PINS[i]);
      delayMicroseconds(200);
    }
    snapshot.raw[i] = total / SAMPLES_PER_READING;
  }

  snapshot.avgBrightness = (snapshot.raw[SENSOR_TOP] +
                            snapshot.raw[SENSOR_BOTTOM] +
                            snapshot.raw[SENSOR_LEFT] +
                            snapshot.raw[SENSOR_RIGHT]) * 0.25f;

  float maxAdjusted = 0.0f;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    float adjusted = static_cast<float>(snapshot.raw[i]) - baseline[i];
    if (adjusted < 0.0f) {
      adjusted = 0.0f;
    }
    snapshot.adjusted[i] = adjusted;
    if (adjusted > maxAdjusted) {
      maxAdjusted = adjusted;
    }
  }

  float denom = (maxAdjusted < 1.0f) ? 1.0f : maxAdjusted;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    float normalized = snapshot.adjusted[i] / denom;
    if (normalized > 1.0f) {
      normalized = 1.0f;
    }
    snapshot.normalized[i] = normalized;
  }

  bool allBelow = true;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (snapshot.raw[i] >= NIGHT_THRESHOLD) {
      allBelow = false;
      break;
    }
  }
  snapshot.night = allBelow;

  snapshot.deadZone = 0.1f * (snapshot.avgBrightness / 1023.0f);
  snapshot.errorX = snapshot.normalized[SENSOR_LEFT] - snapshot.normalized[SENSOR_RIGHT];
  snapshot.errorY = snapshot.normalized[SENSOR_TOP] - snapshot.normalized[SENSOR_BOTTOM];

  snapshot.currentAz = analogRead(azimuth.csPin);
  snapshot.currentEl = analogRead(elevation.csPin);
}

void printTelemetry(const SensorSnapshot &snapshot, const char *action) {
  Serial.print(F("T:"));
  Serial.print(snapshot.raw[SENSOR_TOP]);
  Serial.print(F(" B:"));
  Serial.print(snapshot.raw[SENSOR_BOTTOM]);
  Serial.print(F(" L:"));
  Serial.print(snapshot.raw[SENSOR_LEFT]);
  Serial.print(F(" R:"));
  Serial.print(snapshot.raw[SENSOR_RIGHT]);
  Serial.print(F(" | eX:"));
  Serial.print(snapshot.errorX, 4);
  Serial.print(F(" eY:"));
  Serial.print(snapshot.errorY, 4);
  Serial.print(F(" | Action:"));
  Serial.println(action);
}

void printStatusSummary() {
  Serial.println(F("\n==== Tracker status ===="));
  if (!baselineReady) {
    Serial.println(F("Baselines not ready. Run 'r' to calibrate."));
  } else {
    Serial.print(F("Baselines T/B/R/L: "));
    Serial.print(baseline[SENSOR_TOP], 1);
    Serial.print(F(" / "));
    Serial.print(baseline[SENSOR_BOTTOM], 1);
    Serial.print(F(" / "));
    Serial.print(baseline[SENSOR_RIGHT], 1);
    Serial.print(F(" / "));
    Serial.println(baseline[SENSOR_LEFT], 1);
  }

  Serial.print(F("Night mode: "));
  Serial.println(nightMode ? F("ON") : F("OFF"));

  Serial.print(F("Manual mode: "));
  Serial.println(manualMode ? F("ON") : F("OFF"));

  Serial.print(F("Last dead zone: "));
  Serial.println(lastSnapshot.deadZone, 4);

  Serial.print(F("Last normalized T/B/R/L: "));
  Serial.print(lastSnapshot.normalized[SENSOR_TOP], 3);
  Serial.print(F(" / "));
  Serial.print(lastSnapshot.normalized[SENSOR_BOTTOM], 3);
  Serial.print(F(" / "));
  Serial.print(lastSnapshot.normalized[SENSOR_RIGHT], 3);
  Serial.print(F(" / "));
  Serial.println(lastSnapshot.normalized[SENSOR_LEFT], 3);

  Serial.print(F("Last errorX / errorY: "));
  Serial.print(lastSnapshot.errorX, 4);
  Serial.print(F(" / "));
  Serial.println(lastSnapshot.errorY, 4);

  Serial.print(F("Motor currents (raw ADC) AZ / EL: "));
  Serial.print(lastSnapshot.currentAz);
  Serial.print(F(" / "));
  Serial.println(lastSnapshot.currentEl);
  Serial.println(F("================================\n"));
}

void calibrateSensors() {
  Serial.println(F("\nCalibrating sensors (5 s)..."));
  unsigned long start = millis();
  uint32_t sums[SENSOR_COUNT] = {0, 0, 0, 0};
  uint32_t samples = 0;

  while (millis() - start < CALIBRATION_DURATION_MS) {
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
      sums[i] += analogRead(SENSOR_PINS[i]);
    }
    ++samples;
    delay(10);
  }

  if (samples == 0) {
    Serial.println(F("Calibration failed: no samples collected."));
    return;
  }

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    baseline[i] = static_cast<float>(sums[i]) / static_cast<float>(samples);
  }

  baselineReady = true;

  Serial.print(F("Baseline T/B/R/L: "));
  Serial.print(baseline[SENSOR_TOP], 1);
  Serial.print(F(" / "));
  Serial.print(baseline[SENSOR_BOTTOM], 1);
  Serial.print(F(" / "));
  Serial.print(baseline[SENSOR_RIGHT], 1);
  Serial.print(F(" / "));
  Serial.println(baseline[SENSOR_LEFT], 1);

  Serial.println(F("Calibration complete."));
}

void handleManualCommand(char command) {
  switch (command) {
    case 'j':
      driveAxisManual(azimuth, AZIMUTH_LEFT_DIR, PWM_MIN);
      Serial.println(F("Manual: azimuth LEFT"));
      break;
    case 'l':
      driveAxisManual(azimuth, AZIMUTH_RIGHT_DIR, PWM_MIN);
      Serial.println(F("Manual: azimuth RIGHT"));
      break;
    case 'i':
      driveAxisManual(elevation, ELEVATION_UP_DIR, PWM_MIN);
      Serial.println(F("Manual: elevation UP"));
      break;
    case 'k':
      driveAxisManual(elevation, ELEVATION_DOWN_DIR, PWM_MIN);
      Serial.println(F("Manual: elevation DOWN"));
      break;
    case 'x':
      stopAxis(azimuth);
      stopAxis(elevation);
      Serial.println(F("Manual: all motors STOP"));
      break;
    case 'q':
      stopAxis(azimuth);
      stopAxis(elevation);
      manualMode = false;
      Serial.println(F("Manual mode OFF. Resuming automatic tracking."));
      break;
    default:
      Serial.println(F("Manual commands: j=left l=right i=up k=down x=stop q=exit"));
      break;
  }
}

void handleSerialInput() {
  while (Serial.available() > 0) {
    char command = Serial.read();
    if (command == '\n' || command == '\r') {
      continue;
    }

    if (manualMode) {
      handleManualCommand(command);
      continue;
    }

    switch (command) {
      case 'r':
        stopAxis(azimuth);
        stopAxis(elevation);
        calibrateSensors();
        break;
      case 'p':
        printStatusSummary();
        break;
      case 'm':
        manualMode = true;
        stopAxis(azimuth);
        stopAxis(elevation);
        Serial.println(F("Manual mode ON. Use j/l/i/k to jog, x to stop, q to exit."));
        break;
      default:
        Serial.println(F("Commands: r=recalibrate p=print m=manual"));
        break;
    }
  }
}

void processTracking(unsigned long now) {
  if (!baselineReady) {
    return;
  }

  SensorSnapshot snapshot;
  captureSensors(snapshot);
  lastSnapshot = snapshot;
  nightMode = snapshot.night;

  char action[48] = "";

  if (manualMode) {
    strncpy(action, "Manual", sizeof(action) - 1);
    printTelemetry(snapshot, action);
    return;
  }

  if (snapshot.night) {
    if (azimuth.direction != 0 || elevation.direction != 0) {
      stopAxis(azimuth);
      stopAxis(elevation);
    }
    strncpy(action, "Night", sizeof(action) - 1);
    printTelemetry(snapshot, action);
    return;
  }

  bool anyMove = false;

  float absErrorX = fabs(snapshot.errorX);
  float absErrorY = fabs(snapshot.errorY);

  bool azActive = axisAutoActive(azimuth, now);
  bool elActive = axisAutoActive(elevation, now);

  if (absErrorX < snapshot.deadZone) {
    if (azimuth.direction != 0) {
      stopAxis(azimuth);
    }
  } else {
    int desiredDir = (snapshot.errorX > 0.0f) ? AZIMUTH_LEFT_DIR : AZIMUTH_RIGHT_DIR;
    int pwm = computePwmFromError(absErrorX);
    if (!azActive || azimuth.direction != desiredDir) {
      driveAxisAuto(azimuth, desiredDir, pwm, now);
    } else if (azimuth.currentPwm != pwm) {
      analogWrite(azimuth.pwmPin, pwm);
      azimuth.currentPwm = pwm;
      azimuth.stopTime = now + MOVE_BURST_MS;
    } else {
      azimuth.stopTime = now + MOVE_BURST_MS;
    }
    appendAction(action, sizeof(action), (desiredDir == AZIMUTH_LEFT_DIR) ? "AZ+" : "AZ-");
    anyMove = true;
  }

  if (absErrorY < snapshot.deadZone) {
    if (elevation.direction != 0) {
      stopAxis(elevation);
    }
  } else {
    int desiredDir = (snapshot.errorY > 0.0f) ? ELEVATION_UP_DIR : ELEVATION_DOWN_DIR;
    int pwm = computePwmFromError(absErrorY);
    if (!elActive || elevation.direction != desiredDir) {
      driveAxisAuto(elevation, desiredDir, pwm, now);
    } else if (elevation.currentPwm != pwm) {
      analogWrite(elevation.pwmPin, pwm);
      elevation.currentPwm = pwm;
      elevation.stopTime = now + MOVE_BURST_MS;
    } else {
      elevation.stopTime = now + MOVE_BURST_MS;
    }
    appendAction(action, sizeof(action), (desiredDir == ELEVATION_UP_DIR) ? "EL+" : "EL-");
    anyMove = true;
  }

  if (!anyMove) {
    strncpy(action, "Hold", sizeof(action) - 1);
  }

  printTelemetry(snapshot, action);
}

void configurePins() {
  pinMode(azimuth.inA, OUTPUT);
  pinMode(azimuth.inB, OUTPUT);
  pinMode(azimuth.pwmPin, OUTPUT);
  pinMode(azimuth.csPin, INPUT);

  pinMode(elevation.inA, OUTPUT);
  pinMode(elevation.inB, OUTPUT);
  pinMode(elevation.pwmPin, OUTPUT);
  pinMode(elevation.csPin, INPUT);

  stopAxis(azimuth);
  stopAxis(elevation);
}

// --- Arduino lifecycle ------------------------------------------------------
void setup() {
  Serial.begin(115200);
  configurePins();

  Serial.println(F("Dual-axis solar tracker starting..."));
  Serial.println(F("Stabilising sensors for 2 seconds..."));
  delay(2000);

  calibrateSensors();
  lastSampleMillis = millis();
}

void loop() {
  unsigned long now = millis();
  handleSerialInput();

  updateAxisTimer(azimuth, now);
  updateAxisTimer(elevation, now);

  if (now - lastSampleMillis >= SAMPLE_INTERVAL_MS) {
    lastSampleMillis = now;
    processTracking(now);
  }
}
