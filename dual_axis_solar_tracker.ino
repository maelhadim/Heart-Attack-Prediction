/*
  Dual-Axis Solar Tracker – Adaptive Burst Control

  Hardware:
    - Arduino UNO
    - Four LDR sensors (Top=A0, Bottom=A1, Right=A2, Left=A3)
    - Two VNH2SP30 motor drivers (Azimuth / Elevation) driving 12 V wiper motors
    - Common ground between Arduino, drivers, and motor supply

  Features:
    - 5 s baseline calibration on startup (and on demand with 'r')
    - Adaptive dead zone proportional to ambient brightness (~10%)
    - Burst style proportional control (Kp = 0.6) with min/max PWM limits
    - Movement only when light imbalance exceeds dead zone
    - Automatic night mode when total light < 150 (motors off)
    - Optional manual jog sequence ('m') and status report ('p')

  Serial @ 115200 baud. Status line format:
    T:___ B:___ L:___ R:___ | eX:___ eY:___ | Action:____
*/

#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Pin assignments
// ---------------------------------------------------------------------------
const uint8_t PIN_LDR_TOP = A0;
const uint8_t PIN_LDR_BOTTOM = A1;
const uint8_t PIN_LDR_RIGHT = A2;
const uint8_t PIN_LDR_LEFT = A3;

const uint8_t PIN_AZ_CS = A4;  // optional current sense
const uint8_t PIN_AZ_INA = 9;
const uint8_t PIN_AZ_INB = 10;
const uint8_t PIN_AZ_PWM = 5;

const uint8_t PIN_EL_CS = A5;  // optional current sense
const uint8_t PIN_EL_INA = 3;
const uint8_t PIN_EL_INB = 2;
const uint8_t PIN_EL_PWM = 6;

// ---------------------------------------------------------------------------
// Control constants
// ---------------------------------------------------------------------------
const float KP_GAIN = 0.6f;
const int MIN_PWM = 80;
const int MAX_PWM = 220;

const int NIGHT_THRESHOLD = 150;

const unsigned long SENSOR_INTERVAL_MS = 200;
const unsigned long MOVE_BURST_MS = 300;
const unsigned long SETTLE_MS = 250;
const unsigned long MANUAL_PAUSE_MS = 600;

const uint8_t SAMPLE_COUNT = 10;
const unsigned int SAMPLE_DELAY_US = 400;

// Axis direction helpers (change if wiring is reversed)
const int AZIMUTH_DIR_RIGHT = 1;   // INA=HIGH, INB=LOW
const int AZIMUTH_DIR_LEFT = -1;   // INA=LOW, INB=HIGH
const int ELEVATION_DIR_UP = 1;
const int ELEVATION_DIR_DOWN = -1;

// Positive error handling (errorX = left-right, errorY = top-bottom)
const int AZIMUTH_POSITIVE_ERROR_DIR = AZIMUTH_DIR_LEFT;   // left brighter -> rotate left
const int ELEVATION_POSITIVE_ERROR_DIR = ELEVATION_DIR_UP; // top brighter -> tilt up

// ---------------------------------------------------------------------------
// Axis state machine
// ---------------------------------------------------------------------------
enum AxisStateLabel : uint8_t {
  AXIS_IDLE = 0,
  AXIS_MOVING,
  AXIS_SETTLE
};

struct AxisControl {
  uint8_t pinINA;
  uint8_t pinINB;
  uint8_t pinPWM;
  uint8_t pinCS;
  const char *labelPositive;
  const char *labelNegative;
  AxisStateLabel state;
  unsigned long stateStarted;
  int currentDirection;  // -1, 0, 1
  int currentPWM;        // absolute PWM value
  int lastCommand;       // signed PWM for printing
};

AxisControl azimuth = {
  PIN_AZ_INA, PIN_AZ_INB, PIN_AZ_PWM, PIN_AZ_CS,
  "Right", "Left",
  AXIS_IDLE, 0UL, 0, 0, 0
};

AxisControl elevation = {
  PIN_EL_INA, PIN_EL_INB, PIN_EL_PWM, PIN_EL_CS,
  "Up", "Down",
  AXIS_IDLE, 0UL, 0, 0, 0
};

// ---------------------------------------------------------------------------
// Sensor storage
// ---------------------------------------------------------------------------
float baselineTop = 0.0f;
float baselineBottom = 0.0f;
float baselineLeft = 0.0f;
float baselineRight = 0.0f;

int rawTop = 0;
int rawBottom = 0;
int rawLeft = 0;
int rawRight = 0;

float normTop = 0.0f;
float normBottom = 0.0f;
float normLeft = 0.0f;
float normRight = 0.0f;

float errorX = 0.0f;
float errorY = 0.0f;
float avgRaw = 0.0f;
float lastDeadZone = 0.0f;

// ---------------------------------------------------------------------------
// Runtime control flags
// ---------------------------------------------------------------------------
bool sensorsCalibrated = false;
bool nightModeActive = false;

bool recalibrateRequested = false;
bool manualJogRequested = false;

unsigned long lastSensorSample = 0UL;
unsigned long trackingResumeAt = 0UL;

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void calibrateSensors();
void readSensors();
int readAveragedAnalog(uint8_t pin);
void updateTracking();
void updateAxis(AxisControl &axis, float error, float deadZone, int positiveErrorDirection, unsigned long now, bool allowMotion);
void driveMotor(AxisControl &axis, int direction, int pwm);
void stopMotor(AxisControl &axis);
void composeAction(char *buffer, size_t size);
void handleSerial();
void printStatus();
void printDebugInfo();
void manualJogSequence();
void enterNightMode();
void leaveNightMode();

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(azimuth.pinINA, OUTPUT);
  pinMode(azimuth.pinINB, OUTPUT);
  pinMode(azimuth.pinPWM, OUTPUT);
  pinMode(elevation.pinINA, OUTPUT);
  pinMode(elevation.pinINB, OUTPUT);
  pinMode(elevation.pinPWM, OUTPUT);

  stopMotor(azimuth);
  stopMotor(elevation);

  Serial.println();
  Serial.println(F("Dual-axis solar tracker (adaptive burst control)"));
  Serial.println(F("Commands: r=recalibrate, p=print status, m=manual jog"));
  Serial.println();

  calibrateSensors();
  sensorsCalibrated = true;
  trackingResumeAt = millis() + 200UL;
  lastSensorSample = 0UL;
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
void loop() {
  handleSerial();

  if (recalibrateRequested) {
    recalibrateRequested = false;
    calibrateSensors();
    trackingResumeAt = millis() + 200UL;
    lastSensorSample = 0UL;
  }

  if (manualJogRequested) {
    manualJogRequested = false;
    manualJogSequence();
  }

  updateTracking();
}

// ---------------------------------------------------------------------------
// Sensor calibration and reading
// ---------------------------------------------------------------------------
void calibrateSensors() {
  Serial.println(F("Calibrating sensors for 5 seconds..."));

  stopMotor(azimuth);
  stopMotor(elevation);
  azimuth.state = AXIS_IDLE;
  elevation.state = AXIS_IDLE;

  unsigned long start = millis();
  unsigned long count = 0;
  unsigned long sumTop = 0;
  unsigned long sumBottom = 0;
  unsigned long sumLeft = 0;
  unsigned long sumRight = 0;

  while (millis() - start < 5000UL) {
    sumTop += analogRead(PIN_LDR_TOP);
    sumBottom += analogRead(PIN_LDR_BOTTOM);
    sumLeft += analogRead(PIN_LDR_LEFT);
    sumRight += analogRead(PIN_LDR_RIGHT);
    count++;
    delay(20);
  }

  if (count == 0) count = 1;

  baselineTop = sumTop / (float)count;
  baselineBottom = sumBottom / (float)count;
  baselineLeft = sumLeft / (float)count;
  baselineRight = sumRight / (float)count;

  Serial.print(F("Baselines -> T:"));
  Serial.print(baselineTop, 1);
  Serial.print(F(" B:"));
  Serial.print(baselineBottom, 1);
  Serial.print(F(" L:"));
  Serial.print(baselineLeft, 1);
  Serial.print(F(" R:"));
  Serial.println(baselineRight, 1);

  bool lowLight = (baselineTop < NIGHT_THRESHOLD &&
                   baselineBottom < NIGHT_THRESHOLD &&
                   baselineLeft < NIGHT_THRESHOLD &&
                   baselineRight < NIGHT_THRESHOLD);

  if (lowLight) {
    Serial.println(F("Ambient below night threshold. Night mode engaged."));
    nightModeActive = true;
    enterNightMode();
  } else {
    leaveNightMode();
  }
}

int readAveragedAnalog(uint8_t pin) {
  unsigned long total = 0;
  for (uint8_t i = 0; i < SAMPLE_COUNT; i++) {
    total += analogRead(pin);
    delayMicroseconds(SAMPLE_DELAY_US);
  }
  return (int)(total / SAMPLE_COUNT);
}

void readSensors() {
  rawTop = readAveragedAnalog(PIN_LDR_TOP);
  rawBottom = readAveragedAnalog(PIN_LDR_BOTTOM);
  rawLeft = readAveragedAnalog(PIN_LDR_LEFT);
  rawRight = readAveragedAnalog(PIN_LDR_RIGHT);

  avgRaw = (rawTop + rawBottom + rawLeft + rawRight) * 0.25f;

  float correctedTop = max(0.0f, rawTop - baselineTop);
  float correctedBottom = max(0.0f, rawBottom - baselineBottom);
  float correctedLeft = max(0.0f, rawLeft - baselineLeft);
  float correctedRight = max(0.0f, rawRight - baselineRight);

  float maxCorrected = correctedTop;
  if (correctedBottom > maxCorrected) maxCorrected = correctedBottom;
  if (correctedLeft > maxCorrected) maxCorrected = correctedLeft;
  if (correctedRight > maxCorrected) maxCorrected = correctedRight;
  if (maxCorrected < 1.0f) maxCorrected = 1.0f;  // avoid divide-by-zero

  normTop = correctedTop / maxCorrected;
  normBottom = correctedBottom / maxCorrected;
  normLeft = correctedLeft / maxCorrected;
  normRight = correctedRight / maxCorrected;

  errorX = normLeft - normRight;
  errorY = normTop - normBottom;
}

// ---------------------------------------------------------------------------
// Tracking update
// ---------------------------------------------------------------------------
void updateTracking() {
  if (!sensorsCalibrated) {
    return;
  }

  unsigned long now = millis();
  if (now - lastSensorSample < SENSOR_INTERVAL_MS) {
    return;
  }
  lastSensorSample = now;

  readSensors();

  bool lowLight = (rawTop < NIGHT_THRESHOLD &&
                   rawBottom < NIGHT_THRESHOLD &&
                   rawLeft < NIGHT_THRESHOLD &&
                   rawRight < NIGHT_THRESHOLD);

  if (lowLight) {
    if (!nightModeActive) {
      Serial.println(F("Night mode – holding position."));
    }
    nightModeActive = true;
    enterNightMode();
  } else {
    if (nightModeActive) {
      Serial.println(F("Daylight detected – resuming tracking."));
    }
    nightModeActive = false;
    leaveNightMode();
  }

  bool pauseActive = (now < trackingResumeAt);
  bool allowMotion = !nightModeActive && !pauseActive;

  float deadZone = 0.1f * (avgRaw / 1023.0f);
  lastDeadZone = deadZone;

  updateAxis(azimuth, errorX, deadZone, AZIMUTH_POSITIVE_ERROR_DIR, now, allowMotion);
  updateAxis(elevation, errorY, deadZone, ELEVATION_POSITIVE_ERROR_DIR, now, allowMotion);

  printStatus();
}

void updateAxis(AxisControl &axis, float error, float deadZone, int positiveErrorDirection, unsigned long now, bool allowMotion) {
  float absError = fabsf(error);

  if (!allowMotion) {
    if (axis.state != AXIS_IDLE) {
      stopMotor(axis);
      axis.state = AXIS_IDLE;
    }
    axis.lastCommand = 0;
    return;
  }

  bool withinDeadZone = (absError < deadZone);

  switch (axis.state) {
    case AXIS_IDLE:
      axis.lastCommand = 0;
      if (!withinDeadZone) {
        int direction = (error >= 0.0f) ? positiveErrorDirection : -positiveErrorDirection;
        float pwmFloat = KP_GAIN * absError * 255.0f;
        if (pwmFloat > MAX_PWM) pwmFloat = (float)MAX_PWM;
        int pwm = (int)(pwmFloat + 0.5f);
        if (pwm > 0 && pwm < MIN_PWM) {
          pwm = MIN_PWM;
        }
        if (pwm > 0) {
          axis.state = AXIS_MOVING;
          axis.stateStarted = now;
          axis.currentDirection = direction;
          axis.currentPWM = pwm;
          axis.lastCommand = direction * pwm;
          driveMotor(axis, direction, pwm);
        }
      }
      break;

    case AXIS_MOVING:
      if (withinDeadZone || (now - axis.stateStarted >= MOVE_BURST_MS)) {
        stopMotor(axis);
        axis.state = AXIS_SETTLE;
        axis.stateStarted = now;
        axis.currentDirection = 0;
        axis.currentPWM = 0;
        axis.lastCommand = 0;
      } else {
        axis.lastCommand = axis.currentDirection * axis.currentPWM;
      }
      break;

    case AXIS_SETTLE:
      axis.lastCommand = 0;
      if (now - axis.stateStarted >= SETTLE_MS) {
        axis.state = AXIS_IDLE;
      }
      break;
  }
}

void driveMotor(AxisControl &axis, int direction, int pwm) {
  pwm = constrain(pwm, 0, MAX_PWM);
  if (direction > 0) {
    digitalWrite(axis.pinINA, HIGH);
    digitalWrite(axis.pinINB, LOW);
  } else if (direction < 0) {
    digitalWrite(axis.pinINA, LOW);
    digitalWrite(axis.pinINB, HIGH);
  } else {
    digitalWrite(axis.pinINA, LOW);
    digitalWrite(axis.pinINB, LOW);
  }
  analogWrite(axis.pinPWM, pwm);
}

void stopMotor(AxisControl &axis) {
  digitalWrite(axis.pinINA, LOW);
  digitalWrite(axis.pinINB, LOW);
  analogWrite(axis.pinPWM, 0);
  axis.currentPWM = 0;
  axis.currentDirection = 0;
  axis.lastCommand = 0;
}

void enterNightMode() {
  stopMotor(azimuth);
  stopMotor(elevation);
  azimuth.state = AXIS_IDLE;
  elevation.state = AXIS_IDLE;
}

void leaveNightMode() {
  // Ensure states resume cleanly without sudden moves
  azimuth.state = AXIS_IDLE;
  elevation.state = AXIS_IDLE;
}

// ---------------------------------------------------------------------------
// Serial helpers
// ---------------------------------------------------------------------------
void handleSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      continue;
    }

    switch (c) {
      case 'r':
      case 'R':
        recalibrateRequested = true;
        Serial.println(F("Recalibration requested."));
        break;

      case 'p':
      case 'P':
        printDebugInfo();
        break;

      case 'm':
      case 'M':
        manualJogRequested = true;
        Serial.println(F("Manual jog sequence queued."));
        break;

      default:
        Serial.println(F("Commands: r=recalibrate, p=print status, m=manual jog"));
        break;
    }
  }
}

void composeAction(char *buffer, size_t size) {
  if (size == 0) {
    return;
  }

  buffer[0] = '\0';

  if (nightModeActive) {
    snprintf(buffer, size, "Night");
    return;
  }

  bool appended = false;
  if (azimuth.lastCommand != 0) {
    char segment[24];
    const char *label = (azimuth.lastCommand > 0) ? azimuth.labelPositive : azimuth.labelNegative;
    snprintf(segment, sizeof(segment), "Az%s(%d)", label, abs(azimuth.lastCommand));
    snprintf(buffer, size, "%s", segment);
    appended = true;
  }

  if (elevation.lastCommand != 0) {
    char segment[24];
    const char *label = (elevation.lastCommand > 0) ? elevation.labelPositive : elevation.labelNegative;
    snprintf(segment, sizeof(segment), "El%s(%d)", label, abs(elevation.lastCommand));
    if (appended) {
      strncat(buffer, " ", size - strlen(buffer) - 1);
      strncat(buffer, segment, size - strlen(buffer) - 1);
    } else {
      snprintf(buffer, size, "%s", segment);
      appended = true;
    }
  }

  if (!appended) {
    snprintf(buffer, size, "Idle");
  }
}

void printStatus() {
  char action[48];
  composeAction(action, sizeof(action));

  Serial.print(F("T:"));
  Serial.print(rawTop);
  Serial.print(F(" B:"));
  Serial.print(rawBottom);
  Serial.print(F(" L:"));
  Serial.print(rawLeft);
  Serial.print(F(" R:"));
  Serial.print(rawRight);

  Serial.print(F(" | eX:"));
  Serial.print(errorX, 3);
  Serial.print(F(" eY:"));
  Serial.print(errorY, 3);

  Serial.print(F(" | Action:"));
  Serial.println(action);
}

void printDebugInfo() {
  Serial.println();
  Serial.println(F("--- Tracker Status ---"));
  Serial.print(F("Raw -> T:"));
  Serial.print(rawTop);
  Serial.print(F(" B:"));
  Serial.print(rawBottom);
  Serial.print(F(" L:"));
  Serial.print(rawLeft);
  Serial.print(F(" R:"));
  Serial.println(rawRight);

  Serial.print(F("Normalized -> T:"));
  Serial.print(normTop, 3);
  Serial.print(F(" B:"));
  Serial.print(normBottom, 3);
  Serial.print(F(" L:"));
  Serial.print(normLeft, 3);
  Serial.print(F(" R:"));
  Serial.println(normRight, 3);

  Serial.print(F("Errors -> eX:"));
  Serial.print(errorX, 4);
  Serial.print(F(" eY:"));
  Serial.println(errorY, 4);

  Serial.print(F("Dead zone: "));
  Serial.println(lastDeadZone, 4);
  Serial.print(F("Night mode: "));
  Serial.println(nightModeActive ? F("ON") : F("OFF"));

  Serial.print(F("Axis states -> Az:"));
  Serial.print((int)azimuth.state);
  Serial.print(F(" El:"));
  Serial.println((int)elevation.state);

  Serial.print(F("Control -> Kp:"));
  Serial.print(KP_GAIN, 2);
  Serial.print(F(" minPWM:"));
  Serial.print(MIN_PWM);
  Serial.print(F(" maxPWM:"));
  Serial.println(MAX_PWM);
  Serial.println(F("-----------------------"));
  Serial.println();
}

void manualJogSequence() {
  Serial.println(F("[Manual] Jogging both axes for verification."));

  stopMotor(azimuth);
  stopMotor(elevation);
  azimuth.state = AXIS_IDLE;
  elevation.state = AXIS_IDLE;

  const int testPWM = 120;

  driveMotor(azimuth, AZIMUTH_DIR_RIGHT, testPWM);
  delay(300);
  stopMotor(azimuth);
  delay(150);

  driveMotor(azimuth, AZIMUTH_DIR_LEFT, testPWM);
  delay(300);
  stopMotor(azimuth);
  delay(150);

  driveMotor(elevation, ELEVATION_DIR_UP, testPWM);
  delay(300);
  stopMotor(elevation);
  delay(150);

  driveMotor(elevation, ELEVATION_DIR_DOWN, testPWM);
  delay(300);
  stopMotor(elevation);

  trackingResumeAt = millis() + MANUAL_PAUSE_MS;
  lastSensorSample = millis();

  Serial.println(F("[Manual] Jog sequence complete."));
}
