/*
 * Dual Axis Solar Tracker
 * ------------------------------------------------------------
 * Tracks the brightest point using four LDR sensors arranged in a cross.
 * Drives two DC wiper motors (azimuth & elevation) via VNH2SP30 motor drivers.
 * The tracker moves in short bursts only when the light imbalance exceeds an
 * adaptive dead zone, keeping the dish steady once aligned.
 *
 * Hardware Summary
 *  - Controller: Arduino UNO
 *  - Motors: 12V DC wiper motors (azimuth and elevation)
 *  - Drivers: VNH2SP30 H-bridge (one per axis)
 *  - Sensors: Four LDRs in a light box (Top/Bottom/Left/Right)
 *  - All grounds must be common between power supply, drivers, and Arduino.
 *
 * Serial Commands (115200 baud)
 *  - 'r' : Recalibrate LDR baselines (5 s averaging)
 *  - 'p' : Print the latest sensor and control status snapshot
 *  - 'm' : Toggle manual jog mode (use WASD while enabled)
 *          'w' up, 's' down, 'a' left, 'd' right (short bursts for testing)
 *
 * Adjust the direction mappings inside startAzimuthMove/startElevationMove
 * if the motors turn opposite of the intended direction.
 */

#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// ------------------------------ Pin Mapping -------------------------------
const uint8_t LDR_TOP_PIN = A0;
const uint8_t LDR_BOTTOM_PIN = A1;
const uint8_t LDR_RIGHT_PIN = A2;
const uint8_t LDR_LEFT_PIN = A3;

const uint8_t AZ_CS_PIN = A4;
const uint8_t AZ_INA_PIN = 9;
const uint8_t AZ_INB_PIN = 10;
const uint8_t AZ_PWM_PIN = 5;

const uint8_t EL_CS_PIN = A5;
const uint8_t EL_INA_PIN = 3;
const uint8_t EL_INB_PIN = 2;
const uint8_t EL_PWM_PIN = 6;

// ------------------------- Control Configuration -------------------------
const float KP = 0.6f;                        // Proportional gain
const uint8_t MIN_PWM = 80;                   // Minimum effective PWM
const uint8_t MAX_PWM = 220;                  // Safety cap (never exceed 220)
const uint16_t NIGHT_THRESHOLD = 150;         // Analog units indicating night
const uint8_t SENSOR_SAMPLES = 10;            // Samples per reading (smoothing)
const unsigned long SAMPLE_INTERVAL_MS = 200; // Tracking refresh cadence
const unsigned long MOTOR_BURST_MS = 300;     // Movement burst duration
const unsigned long CALIBRATION_TIME_MS = 5000; // Startup/command calibration
const uint8_t MANUAL_PWM = 160;               // PWM used for manual jogs

struct SensorReadings {
  float top;
  float bottom;
  float left;
  float right;
};

// Calibration & tracking state
float baselineValues[4] = {512.0f, 512.0f, 512.0f, 512.0f};
float gainFactors[4] = {1.0f, 1.0f, 1.0f, 1.0f}; // Equalize sensor response

SensorReadings lastRawReadings = {0.0f, 0.0f, 0.0f, 0.0f};
float lastErrorX = 0.0f;
float lastErrorY = 0.0f;
char lastAction[64] = "Boot";

bool nightMode = false;
bool manualMode = false;

bool azMoving = false;
int8_t azDirection = 0; // -1 left, +1 right
uint8_t azLastCommandPwm = 0;
unsigned long azStopTime = 0;

bool elMoving = false;
int8_t elDirection = 0; // -1 down, +1 up
uint8_t elLastCommandPwm = 0;
unsigned long elStopTime = 0;

unsigned long lastSampleMillis = 0;

// -------------------------- Function Prototypes --------------------------
void calibrateSensors(bool announce);
SensorReadings readSensors(uint8_t samples, bool applyGain);
SensorReadings normalizeReadings(const SensorReadings &readings, float maxValue);
float maxReading(const SensorReadings &readings);
float averageReading(const SensorReadings &readings);
uint8_t computePwm(float errorMagnitude);
void processTrackingCycle();
void handleSerial();
void startAzimuthMove(int direction, uint8_t pwmValue);
void stopAzimuth();
void startElevationMove(int direction, uint8_t pwmValue);
void stopElevation();
void updateMovementState(unsigned long now);
void enterNightMode();
void exitNightMode();
void manualJogAzimuth(int direction);
void manualJogElevation(int direction);
void printStatusSnapshot(const SensorReadings &raw, float errorX, float errorY,
                         const char *action);
void storeSnapshot(const SensorReadings &raw, float errorX, float errorY,
                   const char *action);

// ------------------------------- Setup -----------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println(F("\nDual Axis Solar Tracker booting..."));

  // Configure motor driver pins
  pinMode(AZ_INA_PIN, OUTPUT);
  pinMode(AZ_INB_PIN, OUTPUT);
  pinMode(AZ_PWM_PIN, OUTPUT);

  pinMode(EL_INA_PIN, OUTPUT);
  pinMode(EL_INB_PIN, OUTPUT);
  pinMode(EL_PWM_PIN, OUTPUT);

  // Current sense pins default to INPUT (analog), no explicit pinMode needed.
  // Ensure motors are stopped before calibration.
  stopAzimuth();
  stopElevation();

  calibrateSensors(true);

  Serial.println(F("Tracker ready. Send 'm' for manual jog, 'r' to recalibrate."));
}

// -------------------------------- Loop -----------------------------------
void loop() {
  handleSerial();
  unsigned long now = millis();
  updateMovementState(now);

  if (!manualMode && now - lastSampleMillis >= SAMPLE_INTERVAL_MS) {
    processTrackingCycle();
  }
}

// --------------------------- Core Functions ------------------------------
void processTrackingCycle() {
  unsigned long now = millis();
  lastSampleMillis = now;

  SensorReadings raw = readSensors(SENSOR_SAMPLES, true);
  float maxVal = maxReading(raw);
  SensorReadings normalized = normalizeReadings(raw, maxVal);
  float avgBrightness = averageReading(raw);
  float adaptiveDeadZone = 0.1f * (avgBrightness / 1023.0f);

  float errorX = normalized.left - normalized.right;
  float errorY = normalized.top - normalized.bottom;
  float absErrorX = fabsf(errorX);
  float absErrorY = fabsf(errorY);

  bool withinNight = maxVal < NIGHT_THRESHOLD;
  char actionBuffer[64] = "";

  if (withinNight) {
    enterNightMode();
    strncpy(actionBuffer, "Night", sizeof(actionBuffer));
    actionBuffer[sizeof(actionBuffer) - 1] = '\0';
  } else {
    if (nightMode) {
      exitNightMode();
    }

    bool azWithinDeadZone = absErrorX < adaptiveDeadZone;
    bool elWithinDeadZone = absErrorY < adaptiveDeadZone;

    if (azWithinDeadZone && azMoving) {
      stopAzimuth();
    }
    if (elWithinDeadZone && elMoving) {
      stopElevation();
    }

    if (!azWithinDeadZone) {
      int desiredDirection = (errorX > 0.0f) ? -1 : 1; // Left brighter -> turn left
      uint8_t pwmValue = computePwm(absErrorX);

      if (azMoving) {
        if (desiredDirection != azDirection) {
          stopAzimuth();
          startAzimuthMove(desiredDirection, pwmValue);
        } else {
          azLastCommandPwm = pwmValue;
          analogWrite(AZ_PWM_PIN, pwmValue);
          azStopTime = millis() + MOTOR_BURST_MS;
        }
      } else {
        startAzimuthMove(desiredDirection, pwmValue);
      }
    }

    if (!elWithinDeadZone) {
      int desiredDirection = (errorY > 0.0f) ? 1 : -1; // Top brighter -> tilt up
      uint8_t pwmValue = computePwm(absErrorY);

      if (elMoving) {
        if (desiredDirection != elDirection) {
          stopElevation();
          startElevationMove(desiredDirection, pwmValue);
        } else {
          elLastCommandPwm = pwmValue;
          analogWrite(EL_PWM_PIN, pwmValue);
          elStopTime = millis() + MOTOR_BURST_MS;
        }
      } else {
        startElevationMove(desiredDirection, pwmValue);
      }
    }

    int len = 0;
    if (azMoving) {
      len += snprintf(actionBuffer + len, sizeof(actionBuffer) - len,
                      "AZ_%s PWM:%u",
                      (azDirection > 0) ? "RIGHT" : "LEFT", azLastCommandPwm);
    } else {
      len += snprintf(actionBuffer + len, sizeof(actionBuffer) - len, "AZ_HOLD");
    }

    if (len < (int)sizeof(actionBuffer) - 1) {
      len += snprintf(actionBuffer + len, sizeof(actionBuffer) - len, " ");
    }

    if (elMoving) {
      len += snprintf(actionBuffer + len, sizeof(actionBuffer) - len,
                      "EL_%s PWM:%u",
                      (elDirection > 0) ? "UP" : "DOWN", elLastCommandPwm);
    } else {
      len += snprintf(actionBuffer + len, sizeof(actionBuffer) - len, "EL_HOLD");
    }

    if (len <= 0) {
      strncpy(actionBuffer, "Hold", sizeof(actionBuffer));
      actionBuffer[sizeof(actionBuffer) - 1] = '\0';
    }
  }

  storeSnapshot(raw, errorX, errorY, actionBuffer);
  printStatusSnapshot(raw, errorX, errorY, actionBuffer);
}

// ------------------------ Sensor & Calibration ---------------------------
void calibrateSensors(bool announce) {
  stopAzimuth();
  stopElevation();

  if (announce) {
    Serial.println(F("\n--- Sensor calibration (5 s) ---"));
  }

  float accum[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  unsigned long samples = 0;
  unsigned long start = millis();

  while (millis() - start < CALIBRATION_TIME_MS) {
    SensorReadings raw = readSensors(4, false);
    accum[0] += raw.top;
    accum[1] += raw.bottom;
    accum[2] += raw.left;
    accum[3] += raw.right;
    samples++;
    delay(10);
  }

  if (samples == 0) {
    samples = 1;
  }

  for (uint8_t i = 0; i < 4; ++i) {
    baselineValues[i] = accum[i] / samples;
  }

  float baselineAvg = (baselineValues[0] + baselineValues[1] + baselineValues[2] +
                       baselineValues[3]) *
                      0.25f;
  if (baselineAvg < 1.0f) {
    baselineAvg = 1.0f;
  }

  float maxBaseline = baselineValues[0];
  for (uint8_t i = 0; i < 4; ++i) {
    float safeValue = baselineValues[i];
    if (safeValue < 1.0f) {
      safeValue = 1.0f;
    }
    gainFactors[i] = baselineAvg / safeValue;
    if (baselineValues[i] > maxBaseline) {
      maxBaseline = baselineValues[i];
    }
  }

  nightMode = maxBaseline < NIGHT_THRESHOLD;
  lastSampleMillis = millis();

  if (announce) {
    Serial.print(F(" Baselines  T:"));
    Serial.print((int)baselineValues[0]);
    Serial.print(F(" B:"));
    Serial.print((int)baselineValues[1]);
    Serial.print(F(" L:"));
    Serial.print((int)baselineValues[2]);
    Serial.print(F(" R:"));
    Serial.println((int)baselineValues[3]);

    if (nightMode) {
      Serial.println(F(" Night mode active (ambient below threshold)."));
    } else {
      Serial.println(F(" Calibration complete."));
    }
  }
}

SensorReadings readSensors(uint8_t samples, bool applyGain) {
  if (samples == 0) {
    samples = 1;
  }

  SensorReadings readings = {0.0f, 0.0f, 0.0f, 0.0f};

  for (uint8_t i = 0; i < samples; ++i) {
    readings.top += analogRead(LDR_TOP_PIN);
    readings.bottom += analogRead(LDR_BOTTOM_PIN);
    readings.right += analogRead(LDR_RIGHT_PIN);
    readings.left += analogRead(LDR_LEFT_PIN);
    delayMicroseconds(200); // Allow ADC multiplexer to settle
  }

  float invSamples = 1.0f / samples;
  readings.top *= invSamples;
  readings.bottom *= invSamples;
  readings.right *= invSamples;
  readings.left *= invSamples;

  if (applyGain) {
    readings.top *= gainFactors[0];
    readings.bottom *= gainFactors[1];
    readings.left *= gainFactors[2];
    readings.right *= gainFactors[3];
  }

  return readings;
}

SensorReadings normalizeReadings(const SensorReadings &readings, float maxValue) {
  SensorReadings normalized = {0.0f, 0.0f, 0.0f, 0.0f};
  if (maxValue < 1.0f) {
    return normalized;
  }

  float inv = 1.0f / maxValue;
  normalized.top = constrain(readings.top * inv, 0.0f, 1.0f);
  normalized.bottom = constrain(readings.bottom * inv, 0.0f, 1.0f);
  normalized.left = constrain(readings.left * inv, 0.0f, 1.0f);
  normalized.right = constrain(readings.right * inv, 0.0f, 1.0f);
  return normalized;
}

float maxReading(const SensorReadings &readings) {
  float maxTB = (readings.top > readings.bottom) ? readings.top : readings.bottom;
  float maxLR = (readings.left > readings.right) ? readings.left : readings.right;
  return (maxTB > maxLR) ? maxTB : maxLR;
}

float averageReading(const SensorReadings &readings) {
  return (readings.top + readings.bottom + readings.left + readings.right) * 0.25f;
}

uint8_t computePwm(float errorMagnitude) {
  float commanded = KP * errorMagnitude * 255.0f;
  if (commanded < MIN_PWM) {
    commanded = MIN_PWM;
  }
  if (commanded > MAX_PWM) {
    commanded = MAX_PWM;
  }
  return static_cast<uint8_t>(commanded);
}

// ---------------------------- Motor Control ------------------------------
void startAzimuthMove(int direction, uint8_t pwmValue) {
  azDirection = (direction >= 0) ? 1 : -1;
  azLastCommandPwm = pwmValue;

  if (azDirection > 0) {
    digitalWrite(AZ_INA_PIN, HIGH);
    digitalWrite(AZ_INB_PIN, LOW);
  } else {
    digitalWrite(AZ_INA_PIN, LOW);
    digitalWrite(AZ_INB_PIN, HIGH);
  }
  analogWrite(AZ_PWM_PIN, pwmValue);

  azMoving = true;
  azStopTime = millis() + MOTOR_BURST_MS;
}

void stopAzimuth() {
  analogWrite(AZ_PWM_PIN, 0);
  digitalWrite(AZ_INA_PIN, LOW);
  digitalWrite(AZ_INB_PIN, LOW);
  azMoving = false;
  azDirection = 0;
  azLastCommandPwm = 0;
  azStopTime = millis();
}

void startElevationMove(int direction, uint8_t pwmValue) {
  elDirection = (direction >= 0) ? 1 : -1;
  elLastCommandPwm = pwmValue;

  if (elDirection > 0) {
    digitalWrite(EL_INA_PIN, HIGH);
    digitalWrite(EL_INB_PIN, LOW);
  } else {
    digitalWrite(EL_INA_PIN, LOW);
    digitalWrite(EL_INB_PIN, HIGH);
  }
  analogWrite(EL_PWM_PIN, pwmValue);

  elMoving = true;
  elStopTime = millis() + MOTOR_BURST_MS;
}

void stopElevation() {
  analogWrite(EL_PWM_PIN, 0);
  digitalWrite(EL_INA_PIN, LOW);
  digitalWrite(EL_INB_PIN, LOW);
  elMoving = false;
  elDirection = 0;
  elLastCommandPwm = 0;
  elStopTime = millis();
}

void updateMovementState(unsigned long now) {
  if (azMoving && ((long)(azStopTime - now) <= 0)) {
    stopAzimuth();
  }
  if (elMoving && ((long)(elStopTime - now) <= 0)) {
    stopElevation();
  }
}

void enterNightMode() {
  nightMode = true;
  stopAzimuth();
  stopElevation();
  strncpy(lastAction, "Night", sizeof(lastAction));
  lastAction[sizeof(lastAction) - 1] = '\0';
}

void exitNightMode() {
  nightMode = false;
}

// ------------------------------ Serial I/O -------------------------------
void handleSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    switch (c) {
    case 'r':
    case 'R':
      calibrateSensors(true);
      break;
    case 'p':
    case 'P':
      printStatusSnapshot(lastRawReadings, lastErrorX, lastErrorY, lastAction);
      break;
    case 'm':
    case 'M':
      manualMode = !manualMode;
      stopAzimuth();
      stopElevation();
      if (manualMode) {
        Serial.println(
            F("Manual jog mode ON (w=up, s=down, a=left, d=right). Send 'm' to "
              "exit."));
        strncpy(lastAction, "Manual mode", sizeof(lastAction));
      } else {
        Serial.println(F("Manual jog mode OFF. Resuming auto tracking."));
        strncpy(lastAction, "Auto resume", sizeof(lastAction));
        lastSampleMillis = millis(); // reset timing to avoid rapid loop
      }
      lastAction[sizeof(lastAction) - 1] = '\0';
      break;
    case 'w':
    case 'W':
      if (manualMode) {
        manualJogElevation(1);
      }
      break;
    case 's':
    case 'S':
      if (manualMode) {
        manualJogElevation(-1);
      }
      break;
    case 'a':
    case 'A':
      if (manualMode) {
        manualJogAzimuth(-1);
      }
      break;
    case 'd':
    case 'D':
      if (manualMode) {
        manualJogAzimuth(1);
      }
      break;
    default:
      break;
    }
  }
}

void manualJogAzimuth(int direction) {
  startAzimuthMove(direction, MANUAL_PWM);
  strncpy(lastAction, (direction > 0) ? "Manual AZ right" : "Manual AZ left",
          sizeof(lastAction));
  lastAction[sizeof(lastAction) - 1] = '\0';
}

void manualJogElevation(int direction) {
  startElevationMove(direction, MANUAL_PWM);
  strncpy(lastAction, (direction > 0) ? "Manual EL up" : "Manual EL down",
          sizeof(lastAction));
  lastAction[sizeof(lastAction) - 1] = '\0';
}

// ---------------------------- Status Output ------------------------------
void storeSnapshot(const SensorReadings &raw, float errorX, float errorY,
                   const char *action) {
  lastRawReadings = raw;
  lastErrorX = errorX;
  lastErrorY = errorY;
  strncpy(lastAction, action, sizeof(lastAction));
  lastAction[sizeof(lastAction) - 1] = '\0';
}

void printStatusSnapshot(const SensorReadings &raw, float errorX, float errorY,
                         const char *action) {
  Serial.print(F("T:"));
  Serial.print((int)round(raw.top));
  Serial.print(F(" B:"));
  Serial.print((int)round(raw.bottom));
  Serial.print(F(" L:"));
  Serial.print((int)round(raw.left));
  Serial.print(F(" R:"));
  Serial.print((int)round(raw.right));

  Serial.print(F(" | eX:"));
  Serial.print(errorX, 3);
  Serial.print(F(" eY:"));
  Serial.print(errorY, 3);
  Serial.print(F(" | Action:"));
  Serial.println(action);
}
