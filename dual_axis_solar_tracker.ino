/*
  Dual-Axis Solar Tracker with PID Control and Relay Autotune
  Hardware:
    - Arduino UNO
    - 4 LDRs (top/bottom/left/right) on voltage dividers
    - 2x VNH2SP30 motor drivers
    - 2x 12V wiper motors (12 V supply, grounds common with Arduino)

  Serial commands:
    - 't' or 'T' → request PID autotune (confirm with 'y', abort with 'q')
    - 's' or 'S' → run startup scan again
    - 'p' or 'P' → print current PID gains and stored LDR baselines
    - 'c' or 'C' → re-run 5 s LDR baseline calibration

  Adjustments:
    - Edit deadZoneRatio, hysteresisRatio, minPWM, and default PID gains below to tune response.
    - Recalibrate baselines with the 'c' command if ambient lighting drifts.
    - Autotune performs a relay (on/off) test using Ziegler–Nichols; ensure bright, steady light.
*/

#include <math.h>
#include <string.h>
#include <stdio.h>

struct AxisControl {
  uint8_t pinINA;
  uint8_t pinINB;
  uint8_t pinPWM;
  uint8_t pinCS;
  float Kp;
  float Ki;
  float Kd;
  float integral;
  float prevError;
  unsigned long lastUpdate;
  int lastDirection;
  float lastCommandError;
  unsigned long lastMoveTime;
};

// LDR analog inputs
const uint8_t PIN_LDR_TOP = A0;
const uint8_t PIN_LDR_BOTTOM = A1;
const uint8_t PIN_LDR_RIGHT = A2;
const uint8_t PIN_LDR_LEFT = A3;

// Motor driver pin mapping (VNH2SP30)
const uint8_t PIN_AZ_CS = A4;  // optional current sense
const uint8_t PIN_AZ_INA = 9;
const uint8_t PIN_AZ_INB = 10;
const uint8_t PIN_AZ_PWM = 5;  // PWM capable

const uint8_t PIN_EL_CS = A5;  // optional current sense
const uint8_t PIN_EL_INA = 3;
const uint8_t PIN_EL_INB = 2;
const uint8_t PIN_EL_PWM = 6;  // PWM capable

AxisControl azimuth = {
  PIN_AZ_INA, PIN_AZ_INB, PIN_AZ_PWM, PIN_AZ_CS,
  0.12f, 0.004f, 0.06f,
  0.0f, 0.0f, 0UL, 0, 0.5f, 0UL
};

AxisControl elevation = {
  PIN_EL_INA, PIN_EL_INB, PIN_EL_PWM, PIN_EL_CS,
  0.12f, 0.004f, 0.06f,
  0.0f, 0.0f, 0UL, 0, 0.5f, 0UL
};

// Tracking and tuning parameters
const uint8_t SMOOTH_SAMPLES = 10;          // >= 8 samples per requirement
const unsigned int SMOOTH_DELAY_US = 4000;  // inter-sample delay
float deadZoneRatio = 0.08f;                // 8% default dead zone (edit to adjust sensitivity)
const float hysteresisRatio = 0.02f;        // require error reduction before reversing direction
const int minPWM = 90;                      // minimum PWM to overcome static friction (edit as needed)
const int MAX_PWM = 220;                    // safety headroom below full scale 255
const float PWM_SCALE = 255.0f;             // scale PID output (ratio) to PWM
const unsigned long SENSOR_SETTLE_MS = 250; // pause after movement for sensors to settle
float nightThreshold = 150.0f;              // average raw ADC threshold for night mode (edit if needed)
const float EPSILON = 0.0001f;

// Autotune parameters
const int AUTOTUNE_RELAY_PWM = 140;
const float AUTOTUNE_MAX_ERROR = 0.7f;
const int AUTOTUNE_MIN_CROSSINGS = 10;
const unsigned long AUTOTUNE_MAX_DURATION = 180000UL; // 3 minutes
const float ZERO_CROSS_THRESHOLD = 0.01f;
const int CURRENT_SENSE_THRESHOLD = 850; // adjust to suit actual current sense scaling

// Sensor baselines
float baseTop = 0.0f;
float baseBottom = 0.0f;
float baseLeft = 0.0f;
float baseRight = 0.0f;

// Sensor readings
float rawTop = 0.0f;
float rawBottom = 0.0f;
float rawLeft = 0.0f;
float rawRight = 0.0f;
float correctedTop = 0.0f;
float correctedBottom = 0.0f;
float correctedLeft = 0.0f;
float correctedRight = 0.0f;
float normTop = 0.0f;
float normBottom = 0.0f;
float normLeft = 0.0f;
float normRight = 0.0f;
float avgRaw = 0.0f;
float avgCorrected = 0.0f;
float errorX = 0.0f;
float errorY = 0.0f;

// Current sense tracking
int lastCSAzi = -1;
int lastCSElev = -1;

// Loop state
char actionMessage[48] = "Idle";
bool nightModeActive = false;
bool awaitingAutotuneConfirm = false;
bool requestAutotune = false;
bool requestStartupScan = false;
bool requestRecalibration = false;
bool isAutoTuning = false;

// Forward declarations
void calibrateLDRs();
void startupScan();
void updateSensors();
int readSmoothLDR(uint8_t pin);
float computePIDAxis(AxisControl &axis, float error);
int applyAxisOutput(AxisControl &axis, float pidOutput, float error, bool isAzimuth);
void setMotor(AxisControl &axis, int direction, int pwm);
void stopMotor(AxisControl &axis);
int readCurrentSense(const AxisControl &axis);
bool checkOverCurrent(AxisControl &axis, const char *axisName);
void handleSerial();
void updateActionMessage(int pwmX, int pwmY);
void printStatus(int pwmX, int pwmY);
void printPIDValues();
void runAutotune();
bool autoTuneAxis(AxisControl &axis, const char *axisName, bool isAzimuth);

void setup() {
  Serial.begin(115200);
  delay(500); // allow serial monitor to connect

  Serial.println();
  Serial.println(F("Dual-axis solar tracker with PID + autotune"));
  Serial.println(F("Commands: t=autotune, s=startup scan, p=print PID, c=calibrate baselines"));
  Serial.println(F("Autotune uses relay oscillation (confirm with 'y', abort with 'q')."));
  Serial.println(F("Adjust deadZoneRatio/minPWM/PID defaults near top of sketch if needed."));
  Serial.println();

  pinMode(azimuth.pinINA, OUTPUT);
  pinMode(azimuth.pinINB, OUTPUT);
  pinMode(azimuth.pinPWM, OUTPUT);
  pinMode(elevation.pinINA, OUTPUT);
  pinMode(elevation.pinINB, OUTPUT);
  pinMode(elevation.pinPWM, OUTPUT);

  stopMotor(azimuth);
  stopMotor(elevation);

  calibrateLDRs();
  updateSensors();
  startupScan();
  updateSensors();

  Serial.println(F("Tracker ready. Entering control loop."));
}

void loop() {
  handleSerial();

  if (requestRecalibration && !isAutoTuning) {
    requestRecalibration = false;
    calibrateLDRs();
    updateSensors();
  }

  if (requestStartupScan && !isAutoTuning) {
    requestStartupScan = false;
    startupScan();
    updateSensors();
  }

  if (requestAutotune && !isAutoTuning) {
    requestAutotune = false;
    runAutotune();
    updateSensors();
  }

  if (isAutoTuning) {
    delay(50);
    return;
  }

  updateSensors();
  lastCSAzi = readCurrentSense(azimuth);
  lastCSElev = readCurrentSense(elevation);

  if (avgRaw < nightThreshold) {
    if (!nightModeActive) {
      Serial.println(F("Night mode – holding position."));
    }
    nightModeActive = true;
    stopMotor(azimuth);
    stopMotor(elevation);
    strncpy(actionMessage, "Night mode", sizeof(actionMessage));
    printStatus(0, 0);
    delay(1000);
    return;
  } else {
    nightModeActive = false;
  }

  float pidX = computePIDAxis(azimuth, errorX);
  float pidY = computePIDAxis(elevation, errorY);

  int pwmX = applyAxisOutput(azimuth, pidX, errorX, true);
  int pwmY = applyAxisOutput(elevation, pidY, errorY, false);

  updateActionMessage(pwmX, pwmY);
  printStatus(pwmX, pwmY);

  if (pwmX != 0 || pwmY != 0) {
    delay(SENSOR_SETTLE_MS);
  } else {
    delay(120);
  }
}

void calibrateLDRs() {
  Serial.println(F("Calibrating LDR baselines for 5 seconds..."));
  stopMotor(azimuth);
  stopMotor(elevation);

  unsigned long start = millis();
  unsigned long elapsed = 0;
  unsigned long count = 0;
  unsigned long sumTop = 0;
  unsigned long sumBottom = 0;
  unsigned long sumLeft = 0;
  unsigned long sumRight = 0;

  while ((elapsed = millis() - start) < 5000UL) {
    sumTop += analogRead(PIN_LDR_TOP);
    sumBottom += analogRead(PIN_LDR_BOTTOM);
    sumLeft += analogRead(PIN_LDR_LEFT);
    sumRight += analogRead(PIN_LDR_RIGHT);
    count++;
    delay(20);
  }

  if (count == 0) count = 1;
  baseTop = sumTop / (float)count;
  baseBottom = sumBottom / (float)count;
  baseLeft = sumLeft / (float)count;
  baseRight = sumRight / (float)count;

  Serial.print(F("Baselines -> T:"));
  Serial.print(baseTop, 1);
  Serial.print(F(" B:"));
  Serial.print(baseBottom, 1);
  Serial.print(F(" L:"));
  Serial.print(baseLeft, 1);
  Serial.print(F(" R:"));
  Serial.println(baseRight, 1);
}

void startupScan() {
  Serial.println(F("Running startup scan..."));
  stopMotor(azimuth);
  stopMotor(elevation);
  delay(150);

  updateSensors();
  float bestBrightness = correctedTop + correctedBottom + correctedLeft + correctedRight;
  enum ScanPos { CENTER_POS, LEFT_POS, RIGHT_POS };
  ScanPos bestAz = CENTER_POS;
  const int scanPWM = minPWM + 30;
  const unsigned long sweepMs = 350;
  const unsigned long waitMs = 220;

  // Test left
  setMotor(azimuth, -1, scanPWM);
  delay(sweepMs);
  stopMotor(azimuth);
  delay(waitMs);
  updateSensors();
  float brightness = correctedTop + correctedBottom + correctedLeft + correctedRight;
  if (brightness > bestBrightness) {
    bestBrightness = brightness;
    bestAz = LEFT_POS;
  }
  setMotor(azimuth, 1, scanPWM);
  delay(sweepMs);
  stopMotor(azimuth);
  delay(waitMs);

  // Test right
  setMotor(azimuth, 1, scanPWM);
  delay(sweepMs);
  stopMotor(azimuth);
  delay(waitMs);
  updateSensors();
  brightness = correctedTop + correctedBottom + correctedLeft + correctedRight;
  if (brightness > bestBrightness) {
    bestBrightness = brightness;
    bestAz = RIGHT_POS;
  }
  setMotor(azimuth, -1, scanPWM);
  delay(sweepMs);
  stopMotor(azimuth);
  delay(waitMs);

  // Move to best azimuth position
  if (bestAz == LEFT_POS) {
    setMotor(azimuth, -1, scanPWM);
    delay(sweepMs);
  } else if (bestAz == RIGHT_POS) {
    setMotor(azimuth, 1, scanPWM);
    delay(sweepMs);
  }
  stopMotor(azimuth);
  delay(waitMs);

  // Elevation sweep (up/down)
  enum ScanElev { MID_POS, UP_POS, DOWN_POS };
  ScanElev bestEl = MID_POS;

  setMotor(elevation, 1, scanPWM);  // assume direction = up
  delay(sweepMs);
  stopMotor(elevation);
  delay(waitMs);
  updateSensors();
  brightness = correctedTop + correctedBottom + correctedLeft + correctedRight;
  if (brightness > bestBrightness) {
    bestBrightness = brightness;
    bestEl = UP_POS;
  }
  setMotor(elevation, -1, scanPWM);
  delay(sweepMs);
  stopMotor(elevation);
  delay(waitMs);

  setMotor(elevation, -1, scanPWM);  // down
  delay(sweepMs);
  stopMotor(elevation);
  delay(waitMs);
  updateSensors();
  brightness = correctedTop + correctedBottom + correctedLeft + correctedRight;
  if (brightness > bestBrightness) {
    bestBrightness = brightness;
    bestEl = DOWN_POS;
  }
  setMotor(elevation, 1, scanPWM);
  delay(sweepMs);
  stopMotor(elevation);
  delay(waitMs);

  if (bestEl == UP_POS) {
    setMotor(elevation, 1, scanPWM);
    delay(sweepMs);
  } else if (bestEl == DOWN_POS) {
    setMotor(elevation, -1, scanPWM);
    delay(sweepMs);
  }
  stopMotor(elevation);
  delay(waitMs);

  azimuth.integral = 0.0f;
  azimuth.prevError = 0.0f;
  elevation.integral = 0.0f;
  elevation.prevError = 0.0f;

  Serial.println(F("Startup scan complete."));
}

void updateSensors() {
  rawTop = readSmoothLDR(PIN_LDR_TOP);
  rawBottom = readSmoothLDR(PIN_LDR_BOTTOM);
  rawLeft = readSmoothLDR(PIN_LDR_LEFT);
  rawRight = readSmoothLDR(PIN_LDR_RIGHT);

  avgRaw = (rawTop + rawBottom + rawLeft + rawRight) * 0.25f;

  correctedTop = max(0.0f, rawTop - baseTop);
  correctedBottom = max(0.0f, rawBottom - baseBottom);
  correctedLeft = max(0.0f, rawLeft - baseLeft);
  correctedRight = max(0.0f, rawRight - baseRight);

  avgCorrected = (correctedTop + correctedBottom + correctedLeft + correctedRight) * 0.25f;

  float maxVal = correctedTop;
  if (correctedBottom > maxVal) maxVal = correctedBottom;
  if (correctedLeft > maxVal) maxVal = correctedLeft;
  if (correctedRight > maxVal) maxVal = correctedRight;
  if (maxVal < EPSILON) maxVal = EPSILON;

  normTop = correctedTop / maxVal;
  normBottom = correctedBottom / maxVal;
  normLeft = correctedLeft / maxVal;
  normRight = correctedRight / maxVal;

  errorX = normLeft - normRight;
  errorY = normTop - normBottom;
}

int readSmoothLDR(uint8_t pin) {
  unsigned long sum = 0;
  for (uint8_t i = 0; i < SMOOTH_SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(SMOOTH_DELAY_US);
  }
  return (int)(sum / SMOOTH_SAMPLES);
}

float computePIDAxis(AxisControl &axis, float error) {
  unsigned long now = millis();
  float dt = 0.0f;
  if (axis.lastUpdate == 0UL) {
    dt = 0.02f; // assume 20 ms for first iteration
  } else {
    dt = (now - axis.lastUpdate) / 1000.0f;
  }
  if (dt < 0.001f) {
    dt = 0.001f;
  } else if (dt > 0.5f) {
    dt = 0.5f;
  }
  axis.lastUpdate = now;

  float absError = fabsf(error);
  if (absError > deadZoneRatio) {
    axis.integral += error * dt;
  } else {
    axis.integral *= 0.9f;
  }
  axis.integral = constrain(axis.integral, -10.0f, 10.0f);

  float derivative = 0.0f;
  if (dt > 0.0f) {
    derivative = (error - axis.prevError) / dt;
  }

  float output = axis.Kp * error + axis.Ki * axis.integral + axis.Kd * derivative;
  axis.prevError = error;
  return output;
}

int applyAxisOutput(AxisControl &axis, float pidOutput, float error, bool isAzimuth) {
  float absError = fabsf(error);
  float previousCommandError = axis.lastCommandError;

  if (absError < deadZoneRatio) {
    axis.lastCommandError = absError;
    stopMotor(axis);
    return 0;
  }

  int desiredDirection = (pidOutput >= 0.0f) ? 1 : -1;
  if (fabsf(pidOutput) < 0.0005f) {
    desiredDirection = (error >= 0.0f) ? 1 : -1;
  }

  if (axis.lastDirection != 0 && desiredDirection != axis.lastDirection) {
    if (previousCommandError > 0.01f) {
      float requiredError = previousCommandError * (1.0f - hysteresisRatio);
      if (absError > requiredError) {
        stopMotor(axis);
        return 0;
      }
    }
  }

  float pwmFloat = fabsf(pidOutput) * PWM_SCALE;
  if (pwmFloat > MAX_PWM) pwmFloat = MAX_PWM;

  int pwm = (int)(pwmFloat + 0.5f);
  if (pwm > 0 && pwm < minPWM) {
    pwm = minPWM;
  }
  if (pwm == 0) {
    stopMotor(axis);
    return 0;
  }

  setMotor(axis, desiredDirection, pwm);
  axis.lastMoveTime = millis();
  axis.lastCommandError = absError;

  const char *axisName = isAzimuth ? "Azimuth" : "Elevation";
  if (checkOverCurrent(axis, axisName)) {
    return 0;
  }

  return desiredDirection > 0 ? pwm : -pwm;
}

void setMotor(AxisControl &axis, int direction, int pwm) {
  pwm = constrain(pwm, 0, MAX_PWM);
  if (direction > 0) {
    digitalWrite(axis.pinINA, HIGH);
    digitalWrite(axis.pinINB, LOW);
    axis.lastDirection = 1;
  } else if (direction < 0) {
    digitalWrite(axis.pinINA, LOW);
    digitalWrite(axis.pinINB, HIGH);
    axis.lastDirection = -1;
  } else {
    digitalWrite(axis.pinINA, LOW);
    digitalWrite(axis.pinINB, LOW);
    axis.lastDirection = 0;
  }
  analogWrite(axis.pinPWM, pwm);
}

void stopMotor(AxisControl &axis) {
  digitalWrite(axis.pinINA, LOW);
  digitalWrite(axis.pinINB, LOW);
  analogWrite(axis.pinPWM, 0);
  axis.lastDirection = 0;
}

int readCurrentSense(const AxisControl &axis) {
  if (axis.pinCS == 0xFF) {
    return -1;
  }
  return analogRead(axis.pinCS);
}

bool checkOverCurrent(AxisControl &axis, const char *axisName) {
  int csValue = readCurrentSense(axis);
  if (&axis == &azimuth) {
    lastCSAzi = csValue;
  } else if (&axis == &elevation) {
    lastCSElev = csValue;
  }

  if (csValue >= 0 && csValue > CURRENT_SENSE_THRESHOLD) {
    Serial.print(axisName);
    Serial.println(F(" overcurrent detected! Stopping axis."));
    stopMotor(axis);
    return true;
  }
  return false;
}

void updateActionMessage(int pwmX, int pwmY) {
  if (pwmX == 0 && pwmY == 0) {
    strncpy(actionMessage, nightModeActive ? "Night mode" : "Idle", sizeof(actionMessage));
    actionMessage[sizeof(actionMessage) - 1] = '\0';
    return;
  }

  actionMessage[0] = '\0';

  if (pwmX != 0) {
    char segment[20];
    snprintf(segment, sizeof(segment), "Az%s(%d)", (pwmX > 0) ? "Right" : "Left", abs(pwmX));
    strncat(actionMessage, segment, sizeof(actionMessage) - strlen(actionMessage) - 1);
  }

  if (pwmY != 0) {
    if (actionMessage[0] != '\0') {
      strncat(actionMessage, " ", sizeof(actionMessage) - strlen(actionMessage) - 1);
    }
    char segment[20];
    snprintf(segment, sizeof(segment), "El%s(%d)", (pwmY > 0) ? "Up" : "Down", abs(pwmY));
    strncat(actionMessage, segment, sizeof(actionMessage) - strlen(actionMessage) - 1);
  }
}

void printStatus(int pwmX, int pwmY) {
  Serial.print(F("T:"));
  Serial.print((int)rawTop);
  Serial.print(F(" B:"));
  Serial.print((int)rawBottom);
  Serial.print(F(" L:"));
  Serial.print((int)rawLeft);
  Serial.print(F(" R:"));
  Serial.print((int)rawRight);

  Serial.print(F(" | eX:"));
  Serial.print(errorX, 3);
  Serial.print(F(" eY:"));
  Serial.print(errorY, 3);

  Serial.print(F(" | oX:"));
  Serial.print(pwmX);
  Serial.print(F(" oY:"));
  Serial.print(pwmY);

  if (lastCSAzi >= 0) {
    Serial.print(F(" | csX:"));
    Serial.print(lastCSAzi);
  }
  if (lastCSElev >= 0) {
    Serial.print(F(" csY:"));
    Serial.print(lastCSElev);
  }

  Serial.print(F(" | Action:"));
  Serial.println(actionMessage);
}

void printPIDValues() {
  Serial.print(F("Azimuth PID -> Kp:"));
  Serial.print(azimuth.Kp, 4);
  Serial.print(F(" Ki:"));
  Serial.print(azimuth.Ki, 4);
  Serial.print(F(" Kd:"));
  Serial.println(azimuth.Kd, 4);

  Serial.print(F("Elevation PID -> Kp:"));
  Serial.print(elevation.Kp, 4);
  Serial.print(F(" Ki:"));
  Serial.print(elevation.Ki, 4);
  Serial.print(F(" Kd:"));
  Serial.println(elevation.Kd, 4);

  Serial.print(F("Baselines T:"));
  Serial.print(baseTop, 1);
  Serial.print(F(" B:"));
  Serial.print(baseBottom, 1);
  Serial.print(F(" L:"));
  Serial.print(baseLeft, 1);
  Serial.print(F(" R:"));
  Serial.println(baseRight, 1);

  Serial.print(F("deadZoneRatio:"));
  Serial.print(deadZoneRatio, 3);
  Serial.print(F(" minPWM:"));
  Serial.print(minPWM);
  Serial.print(F(" MAX_PWM:"));
  Serial.println(MAX_PWM);
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      continue;
    }

    if (awaitingAutotuneConfirm) {
      if (c == 'y' || c == 'Y') {
        awaitingAutotuneConfirm = false;
        requestAutotune = true;
        Serial.println(F("Autotune confirmed. Starting sequence..."));
      } else if (c == 'n' || c == 'N') {
        awaitingAutotuneConfirm = false;
        Serial.println(F("Autotune cancelled."));
      } else {
        Serial.println(F("Confirm autotune with 'y' or cancel with 'n'."));
      }
      continue;
    }

    switch (c) {
      case 't':
      case 'T':
        if (isAutoTuning) {
          Serial.println(F("Autotune already running."));
        } else {
          Serial.println(F("Autotune requested. Motors will oscillate each axis."));
          Serial.println(F("Type 'y' to begin, 'n' to cancel."));
          awaitingAutotuneConfirm = true;
        }
        break;

      case 's':
      case 'S':
        if (isAutoTuning) {
          Serial.println(F("Busy autotuning; cannot run scan now."));
        } else {
          requestStartupScan = true;
          Serial.println(F("Startup scan queued."));
        }
        break;

      case 'p':
      case 'P':
        printPIDValues();
        break;

      case 'c':
      case 'C':
        if (isAutoTuning) {
          Serial.println(F("Busy autotuning; cannot recalibrate now."));
        } else {
          requestRecalibration = true;
          Serial.println(F("Baseline recalibration queued."));
        }
        break;

      default:
        Serial.println(F("Commands: t=autotune, s=startup scan, p=print PID, c=calibrate"));
        break;
    }
  }
}

void runAutotune() {
  isAutoTuning = true;
  stopMotor(azimuth);
  stopMotor(elevation);
  Serial.println();
  Serial.println(F("=== Autotune starting (relay test, Ziegler–Nichols) ==="));
  Serial.println(F("Ensure bright, steady light. Send 'q' to abort at any time."));

  bool azSuccess = autoTuneAxis(azimuth, "Azimuth", true);
  if (azSuccess) {
    bool elSuccess = autoTuneAxis(elevation, "Elevation", false);
    if (!elSuccess) {
      Serial.println(F("[Autotune] Elevation axis failed autotune."));
    }
  } else {
    Serial.println(F("[Autotune] Azimuth axis failed; skipping elevation."));
  }

  Serial.println(F("=== Autotune complete ==="));
  printPIDValues();
  Serial.println(F("Reminder: PID values are volatile; record them if needed after power loss."));

  stopMotor(azimuth);
  stopMotor(elevation);
  isAutoTuning = false;
}

bool autoTuneAxis(AxisControl &axis, const char *axisName, bool isAzimuth) {
  updateSensors();
  if (avgRaw < nightThreshold) {
    Serial.print(F("[Autotune] "));
    Serial.print(axisName);
    Serial.println(F(" axis: too dark to run autotune."));
    return false;
  }

  float backupKp = axis.Kp;
  float backupKi = axis.Ki;
  float backupKd = axis.Kd;

  axis.integral = 0.0f;
  axis.prevError = 0.0f;
  axis.lastUpdate = millis();
  axis.lastDirection = 0;

  axis.Ki = 0.0f;
  axis.Kd = 0.0f;

  float lastError = isAzimuth ? errorX : errorY;
  bool lastErrorValid = true;
  int direction = (lastError >= 0.0f) ? 1 : -1;

  setMotor(axis, direction, AUTOTUNE_RELAY_PWM);
  delay(250);

  unsigned long zeroCrossTimes[20];
  int zeroCrossCount = 0;
  float maxError = -1000.0f;
  float minError = 1000.0f;

  unsigned long start = millis();

  while (millis() - start < AUTOTUNE_MAX_DURATION) {
    updateSensors();
    float currentError = isAzimuth ? errorX : errorY;

    if (currentError > maxError) maxError = currentError;
    if (currentError < minError) minError = currentError;

    if (lastErrorValid) {
      bool crossingPositive = (currentError > ZERO_CROSS_THRESHOLD && lastError < -ZERO_CROSS_THRESHOLD);
      bool crossingNegative = (currentError < -ZERO_CROSS_THRESHOLD && lastError > ZERO_CROSS_THRESHOLD);
      if (crossingPositive || crossingNegative) {
        if (zeroCrossCount < (int)(sizeof(zeroCrossTimes) / sizeof(zeroCrossTimes[0]))) {
          zeroCrossTimes[zeroCrossCount] = millis();
        }
        zeroCrossCount++;
        direction = -direction;
        setMotor(axis, direction, AUTOTUNE_RELAY_PWM);
        Serial.print(F("[Autotune] "));
        Serial.print(axisName);
        Serial.print(F(" zero-cross #"));
        Serial.println(zeroCrossCount);
      }
    }

    int csValue = readCurrentSense(axis);
    if (csValue >= 0 && csValue > CURRENT_SENSE_THRESHOLD) {
      Serial.print(F("[Autotune] "));
      Serial.print(axisName);
      Serial.println(F(" overcurrent. Aborting."));
      stopMotor(axis);
      axis.Kp = backupKp;
      axis.Ki = backupKi;
      axis.Kd = backupKd;
      return false;
    }

    if (fabsf(currentError) > AUTOTUNE_MAX_ERROR) {
      Serial.print(F("[Autotune] "));
      Serial.print(axisName);
      Serial.println(F(" error amplitude exceeded limit. Aborting."));
      stopMotor(axis);
      axis.Kp = backupKp;
      axis.Ki = backupKi;
      axis.Kd = backupKd;
      return false;
    }

    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'q' || c == 'Q') {
        Serial.print(F("[Autotune] "));
        Serial.print(axisName);
        Serial.println(F(" aborted by user."));
        stopMotor(axis);
        axis.Kp = backupKp;
        axis.Ki = backupKi;
        axis.Kd = backupKd;
        return false;
      }
    }

    if (zeroCrossCount >= AUTOTUNE_MIN_CROSSINGS) {
      unsigned long lastCross = zeroCrossTimes[min(zeroCrossCount, (int)(sizeof(zeroCrossTimes) / sizeof(zeroCrossTimes[0]))) - 1];
      if (millis() - lastCross > 1000UL) {
        break;
      }
    }

    lastError = currentError;
    lastErrorValid = true;
    delay(60);
  }

  stopMotor(axis);

  if (zeroCrossCount < AUTOTUNE_MIN_CROSSINGS) {
    Serial.print(F("[Autotune] "));
    Serial.print(axisName);
    Serial.println(F(" insufficient oscillations. Check lighting and mechanics."));
    axis.Kp = backupKp;
    axis.Ki = backupKi;
    axis.Kd = backupKd;
    return false;
  }

  int usableCrossings = min(zeroCrossCount, (int)(sizeof(zeroCrossTimes) / sizeof(zeroCrossTimes[0])));
  float totalPeriodMs = 0.0f;
  int periodSamples = 0;
  for (int i = 2; i < usableCrossings; i++) {
    totalPeriodMs += (float)(zeroCrossTimes[i] - zeroCrossTimes[i - 2]);
    periodSamples++;
  }

  if (periodSamples == 0) {
    Serial.print(F("[Autotune] "));
    Serial.print(axisName);
    Serial.println(F(" failed to compute oscillation period."));
    axis.Kp = backupKp;
    axis.Ki = backupKi;
    axis.Kd = backupKd;
    return false;
  }

  float Pu = (totalPeriodMs / periodSamples) / 1000.0f;
  float oscillationAmplitude = maxError - minError;
  float halfAmplitude = oscillationAmplitude * 0.5f;

  if (halfAmplitude < 0.001f) {
    Serial.print(F("[Autotune] "));
    Serial.print(axisName);
    Serial.println(F(" oscillation amplitude too small."));
    axis.Kp = backupKp;
    axis.Ki = backupKi;
    axis.Kd = backupKd;
    return false;
  }

  float Ku = AUTOTUNE_RELAY_PWM / halfAmplitude;

  Serial.print(F("[Autotune] "));
  Serial.print(axisName);
  Serial.print(F(" Ku="));
  Serial.print(Ku, 3);
  Serial.print(F(" Pu="));
  Serial.print(Pu, 3);
  Serial.println(F(" s"));

  float newKp = 0.6f * Ku;
  float Ti = 0.5f * Pu;
  float Td = 0.125f * Pu;
  float newKi = (Ti > 0.0f) ? (newKp / Ti) : 0.0f;
  float newKd = newKp * Td;

  axis.Kp = newKp;
  axis.Ki = newKi;
  axis.Kd = newKd;
  axis.integral = 0.0f;
  axis.prevError = 0.0f;
  axis.lastUpdate = millis();

  Serial.print(F("[Autotune] "));
  Serial.print(axisName);
  Serial.print(F(" new PID -> Kp:"));
  Serial.print(newKp, 4);
  Serial.print(F(" Ki:"));
  Serial.print(newKi, 4);
  Serial.print(F(" Kd:"));
  Serial.println(newKd, 4);

  Serial.println(F("[Autotune] Store these values manually if you need them after reset."));

  return true;
}
