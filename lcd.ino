#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>

#include "secrets.h"
#include <ThingSpeak.h>

// ---------------- Pins ----------------

const int PAN_SERVO_PIN = 25;   // Bottom, horizontal servo
const int TILT_SERVO_PIN = 26;  // Top, vertical servo

// LDR positions when looking at the front of the panel.
const int LDR_TOP_LEFT_PIN = 36;
const int LDR_TOP_RIGHT_PIN = 39;
const int LDR_BOTTOM_LEFT_PIN = 35;
const int LDR_BOTTOM_RIGHT_PIN = 34;

// 1 means increasing the angle moves toward right/top.
// Change an axis to -1 if that servo moves the wrong way.
const int PAN_DIRECTION = 1;
const int TILT_DIRECTION = 1;

// ---------------- Settings ----------------

// Servo.write() uses normalized command units, not measured shaft degrees.
// 0 and 180 map to the configured 1000 and 2000 microsecond pulses. The real
// physical travel may be 120, 180, 270, or another value depending on servo.
const int PAN_MIN_COMMAND = 0;
const int PAN_MAX_COMMAND = 180;
const int TILT_MIN_COMMAND = 0;
const int TILT_MAX_COMMAND = 180;
const int SERVO_MIN_PULSE_US = 1000;
const int SERVO_MAX_PULSE_US = 2000;

// Command where the panel normal aligns with the bottom servo's pan axis.
const int TILT_AXIS_ALIGNMENT_COMMAND = 90;
const int LIGHT_DEADBAND = 10;
const unsigned long UPDATE_INTERVAL_MS = 20;
const unsigned long PRINT_INTERVAL_MS = 100;
const unsigned long WIFI_RETRY_INTERVAL_MS = 15000;
const unsigned long THINGSPEAK_UPLOAD_INTERVAL_MS = 20000;

// H(s) = 1 / (tau*s + 1), discretized as an exponential moving average.
// At a 20 ms sample interval, tau=80 ms gives alpha=0.20: quick, but much
// less sensitive to ADC noise than one unfiltered conversion.
const float LDR_FILTER_TIME_CONSTANT_MS = 80.0f;

// Joint-limit recovery settings. A limit must be requested continuously before
// a scan starts, so one noisy reading cannot send the panel across its range.
const int LIMIT_TRIGGER_ERROR = 80;
const int MIN_RECOVERY_BRIGHTNESS = 500;
const unsigned long LIMIT_CONFIRM_MS = 400;
const unsigned long RECOVERY_COOLDOWN_MS = 8000;
const int AXIS_ALIGNMENT_PAN_HOLD_COMMANDS = 5;
const int RECOVERY_SLEW_COMMANDS = 6;
const unsigned long SCAN_SETTLE_MS = 100;
const int SCAN_SAMPLE_COUNT = 4;
const unsigned long FINAL_SETTLE_MS = 220;
const int SCAN_GRID_SIZE = 5;
const int SCAN_POINT_COUNT = SCAN_GRID_SIZE * SCAN_GRID_SIZE;
const int FLIP_ACCEPT_PERCENT = 3;

Servo panServo;
Servo tiltServo;
WiFiClient thingSpeakClient;

int panAngle = 90;
int tiltAngle = 90;

unsigned long lastUpdateMs = 0;
unsigned long lastPrintMs = 0;
unsigned long lastWifiAttemptMs = 0;
unsigned long lastThingSpeakUploadMs = 0;

bool wifiAttemptStarted = false;
bool wifiWasConnected = false;
bool thingSpeakUploadAttempted = false;

int topLeft = 0;
int topRight = 0;
int bottomLeft = 0;
int bottomRight = 0;
int horizontalError = 0;
int verticalError = 0;

float filteredTopLeft = 0.0f;
float filteredTopRight = 0.0f;
float filteredBottomLeft = 0.0f;
float filteredBottomRight = 0.0f;
bool ldrFilterInitialized = false;

enum TrackerMode {
  TRACKING,
  FLIP_TO_ZENITH,
  FLIP_MOVE_PAN,
  FLIP_RESTORE_TILT,
  FLIP_SETTLE,
  FLIP_SAMPLE,
  SCAN_MOVE,
  SCAN_SETTLE,
  SCAN_SAMPLE,
  RETURN_TO_BEST,
  FINAL_SETTLE
};

enum LimitReason {
  NO_LIMIT,
  PAN_MIN_LIMIT,
  PAN_MAX_LIMIT,
  TILT_MIN_LIMIT,
  TILT_MAX_LIMIT,
  PAN_ZENITH_SINGULARITY
};

TrackerMode trackerMode = TRACKING;
LimitReason pendingLimitReason = NO_LIMIT;
bool limitTimerRunning = false;
unsigned long limitRequestStartedMs = 0;
bool recoveryHasRun = false;
unsigned long lastRecoveryFinishedMs = 0;

int recoveryOriginPan = 90;
int recoveryOriginTilt = 90;
int recoveryBestPan = 90;
int recoveryBestTilt = 90;
int recoveryTargetPan = 90;
int recoveryTargetTilt = 90;
uint32_t recoveryOriginScore = 0;
uint32_t recoveryBestScore = 0;
uint32_t recoverySampleTotal = 0;
int recoverySampleCount = 0;
int scanPointIndex = 0;
unsigned long recoveryStateStartedMs = 0;

int readLdrRaw(int pin) {
  // Discard the first conversion after changing ADC channels.
  analogRead(pin);

  long total = 0;
  for (int sample = 0; sample < 4; sample++) {
    total += analogRead(pin);
  }
  return total / 4;
}

void updateLdrReadings() {
  const int rawTopLeft = readLdrRaw(LDR_TOP_LEFT_PIN);
  const int rawTopRight = readLdrRaw(LDR_TOP_RIGHT_PIN);
  const int rawBottomLeft = readLdrRaw(LDR_BOTTOM_LEFT_PIN);
  const int rawBottomRight = readLdrRaw(LDR_BOTTOM_RIGHT_PIN);

  if (!ldrFilterInitialized) {
    filteredTopLeft = rawTopLeft;
    filteredTopRight = rawTopRight;
    filteredBottomLeft = rawBottomLeft;
    filteredBottomRight = rawBottomRight;
    ldrFilterInitialized = true;
  } else {
    const float alpha = UPDATE_INTERVAL_MS /
                        (LDR_FILTER_TIME_CONSTANT_MS + UPDATE_INTERVAL_MS);
    filteredTopLeft += alpha * (rawTopLeft - filteredTopLeft);
    filteredTopRight += alpha * (rawTopRight - filteredTopRight);
    filteredBottomLeft += alpha * (rawBottomLeft - filteredBottomLeft);
    filteredBottomRight += alpha * (rawBottomRight - filteredBottomRight);
  }

  topLeft = (int)(filteredTopLeft + 0.5f);
  topRight = (int)(filteredTopRight + 0.5f);
  bottomLeft = (int)(filteredBottomLeft + 0.5f);
  bottomRight = (int)(filteredBottomRight + 0.5f);

  const int leftAdc = (topLeft + bottomLeft) / 2;
  const int rightAdc = (topRight + bottomRight) / 2;
  const int topAdc = (topLeft + topRight) / 2;
  const int bottomAdc = (bottomLeft + bottomRight) / 2;

  // The LDRs pull the ADC pins toward ground, so lower ADC means brighter.
  // Positive horizontal error means the right side is brighter.
  // Positive vertical error means the top side is brighter.
  horizontalError = leftAdc - rightAdc;
  verticalError = bottomAdc - topAdc;
}

int movementStep(int errorSize) {
  if (errorSize > 800) return 6;  // Large correction
  if (errorSize > 300) return 3;  // Medium correction
  if (errorSize > 80) return 2;   // Small correction
  return 1;                       // Fine correction
}

int signOf(int value) {
  return (value > 0) - (value < 0);
}

int effectivePanDirection() {
  // In a stacked pan/tilt mount, panel-left and panel-right swap relative to
  // increasing pan after the panel passes through the pan-axis singularity.
  const int distanceFromAxisAlignment =
      abs(tiltAngle - TILT_AXIS_ALIGNMENT_COMMAND);
  if (distanceFromAxisAlignment <= AXIS_ALIGNMENT_PAN_HOLD_COMMANDS) return 0;
  return PAN_DIRECTION *
      (tiltAngle < TILT_AXIS_ALIGNMENT_COMMAND ? 1 : -1);
}

int totalBrightness() {
  return (4095 - topLeft) +
         (4095 - topRight) +
         (4095 - bottomLeft) +
         (4095 - bottomRight);
}

uint32_t opticalScore() {
  // Higher is better. Total light rejects a dark-but-equal pose; the balance
  // term rejects a pose where only one corner sees the light.
  const int brightness = totalBrightness();
  if (brightness <= 0) return 0;

  const int imbalance =
      2 * (abs(horizontalError) + abs(verticalError));
  const int balancePermille =
      max(0, 1000 - (500 * imbalance) / brightness);
  return ((uint32_t)(brightness / 4) * balancePermille) / 1000;
}

const char *trackerModeName() {
  switch (trackerMode) {
    case TRACKING: return "TRACK";
    case FLIP_TO_ZENITH: return "FLIP-ZENITH";
    case FLIP_MOVE_PAN: return "FLIP-PAN";
    case FLIP_RESTORE_TILT: return "FLIP-TILT";
    case FLIP_SETTLE: return "FLIP-SETTLE";
    case FLIP_SAMPLE: return "FLIP-SAMPLE";
    case SCAN_MOVE: return "SCAN-MOVE";
    case SCAN_SETTLE: return "SCAN-SETTLE";
    case SCAN_SAMPLE: return "SCAN-SAMPLE";
    case RETURN_TO_BEST: return "RETURN-BEST";
    case FINAL_SETTLE: return "FINAL-SETTLE";
  }
  return "UNKNOWN";
}

void enterRecoveryState(TrackerMode newMode, unsigned long now) {
  trackerMode = newMode;
  recoveryStateStartedMs = now;
}

int approachCommand(int current, int target, int maximumStep) {
  if (current < target) return min(current + maximumStep, target);
  if (current > target) return max(current - maximumStep, target);
  return current;
}

bool slewRecoveryServos() {
  const int nextPan =
      approachCommand(panAngle, recoveryTargetPan, RECOVERY_SLEW_COMMANDS);
  const int nextTilt =
      approachCommand(tiltAngle, recoveryTargetTilt, RECOVERY_SLEW_COMMANDS);

  if (nextPan != panAngle) {
    panAngle = nextPan;
    panServo.write(panAngle);
  }
  if (nextTilt != tiltAngle) {
    tiltAngle = nextTilt;
    tiltServo.write(tiltAngle);
  }

  return panAngle == recoveryTargetPan && tiltAngle == recoveryTargetTilt;
}

void resetRecoverySamples() {
  recoverySampleTotal = 0;
  recoverySampleCount = 0;
}

void setScanTarget(int index) {
  const int row = index / SCAN_GRID_SIZE;
  int column = index % SCAN_GRID_SIZE;
  if (row % 2 == 1) column = SCAN_GRID_SIZE - 1 - column;

  recoveryTargetPan = PAN_MIN_COMMAND +
      (column * (PAN_MAX_COMMAND - PAN_MIN_COMMAND)) /
          (SCAN_GRID_SIZE - 1);
  recoveryTargetTilt = TILT_MIN_COMMAND +
      (row * (TILT_MAX_COMMAND - TILT_MIN_COMMAND)) /
          (SCAN_GRID_SIZE - 1);
}

void beginGridScan(unsigned long now) {
  scanPointIndex = 0;
  setScanTarget(scanPointIndex);
  enterRecoveryState(SCAN_MOVE, now);
  Serial.println("Limit recovery: starting bounded 5x5 light search.");
}

void finishRecovery(unsigned long now) {
  trackerMode = TRACKING;
  pendingLimitReason = NO_LIMIT;
  limitTimerRunning = false;
  recoveryHasRun = true;
  lastRecoveryFinishedMs = now;

  Serial.print("Limit recovery complete. Best score/pose: ");
  Serial.print(recoveryBestScore);
  Serial.print(" at ");
  Serial.print(panAngle);
  Serial.print('/');
  Serial.println(tiltAngle);
}

void startRecovery(LimitReason reason, unsigned long now) {
  pendingLimitReason = reason;
  recoveryOriginPan = panAngle;
  recoveryOriginTilt = tiltAngle;
  recoveryOriginScore = opticalScore();
  recoveryBestPan = panAngle;
  recoveryBestTilt = tiltAngle;
  recoveryBestScore = recoveryOriginScore;

  Serial.print("Persistent joint limit detected. Origin score/pose: ");
  Serial.print(recoveryOriginScore);
  Serial.print(" at ");
  Serial.print(recoveryOriginPan);
  Serial.print('/');
  Serial.println(recoveryOriginTilt);

  // A blocked pan gets a fast, geometry-aware over-the-top probe first. Servo
  // commands are not measured angles, so this remains only a candidate: real
  // LDR measurements decide whether it is kept.
  if (reason == PAN_MIN_LIMIT || reason == PAN_MAX_LIMIT) {
    const int mirroredTilt =
        2 * TILT_AXIS_ALIGNMENT_COMMAND - recoveryOriginTilt;
    if (mirroredTilt >= TILT_MIN_COMMAND &&
        mirroredTilt <= TILT_MAX_COMMAND) {
      recoveryTargetPan = recoveryOriginPan;
      recoveryTargetTilt = TILT_AXIS_ALIGNMENT_COMMAND;
      enterRecoveryState(FLIP_TO_ZENITH, now);
      Serial.println("Limit recovery: trying measured over-the-top reindex.");
      return;
    }
  }

  beginGridScan(now);
}

bool recoveryAllowed(unsigned long now) {
  return !recoveryHasRun ||
         now - lastRecoveryFinishedMs >= RECOVERY_COOLDOWN_MS;
}

void updateLimitRequest(LimitReason reason, unsigned long now) {
  if (reason == NO_LIMIT) {
    pendingLimitReason = NO_LIMIT;
    limitTimerRunning = false;
    return;
  }

  if (!limitTimerRunning || reason != pendingLimitReason) {
    pendingLimitReason = reason;
    limitRequestStartedMs = now;
    limitTimerRunning = true;
    return;
  }

  if (now - limitRequestStartedMs >= LIMIT_CONFIRM_MS &&
      recoveryAllowed(now) &&
      totalBrightness() >= MIN_RECOVERY_BRIGHTNESS) {
    startRecovery(reason, now);
  }
}

void serviceRecovery(unsigned long now) {
  switch (trackerMode) {
    case TRACKING:
      return;

    case FLIP_TO_ZENITH:
      if (slewRecoveryServos()) {
        recoveryTargetPan =
            pendingLimitReason == PAN_MAX_LIMIT
                ? PAN_MIN_COMMAND
                : PAN_MAX_COMMAND;
        recoveryTargetTilt = TILT_AXIS_ALIGNMENT_COMMAND;
        enterRecoveryState(FLIP_MOVE_PAN, now);
      }
      return;

    case FLIP_MOVE_PAN:
      if (slewRecoveryServos()) {
        recoveryTargetTilt =
            2 * TILT_AXIS_ALIGNMENT_COMMAND - recoveryOriginTilt;
        enterRecoveryState(FLIP_RESTORE_TILT, now);
      }
      return;

    case FLIP_RESTORE_TILT:
      if (slewRecoveryServos()) {
        enterRecoveryState(FLIP_SETTLE, now);
      }
      return;

    case FLIP_SETTLE:
      if (now - recoveryStateStartedMs >= SCAN_SETTLE_MS) {
        resetRecoverySamples();
        enterRecoveryState(FLIP_SAMPLE, now);
      }
      return;

    case FLIP_SAMPLE:
      recoverySampleTotal += opticalScore();
      recoverySampleCount++;
      if (recoverySampleCount >= SCAN_SAMPLE_COUNT) {
        const uint32_t candidateScore =
            recoverySampleTotal / recoverySampleCount;
        if (candidateScore > recoveryBestScore) {
          recoveryBestScore = candidateScore;
          recoveryBestPan = panAngle;
          recoveryBestTilt = tiltAngle;
        }

        const uint32_t requiredImprovement =
            max((uint32_t)15,
                (recoveryOriginScore * FLIP_ACCEPT_PERCENT) / 100);
        if (candidateScore >= recoveryOriginScore + requiredImprovement) {
          Serial.println("Over-the-top pose improved the measured light.");
          recoveryTargetPan = recoveryBestPan;
          recoveryTargetTilt = recoveryBestTilt;
          enterRecoveryState(FINAL_SETTLE, now);
        } else {
          Serial.println("Over-the-top pose was not clearly better.");
          beginGridScan(now);
        }
      }
      return;

    case SCAN_MOVE:
      if (slewRecoveryServos()) {
        enterRecoveryState(SCAN_SETTLE, now);
      }
      return;

    case SCAN_SETTLE:
      if (now - recoveryStateStartedMs >= SCAN_SETTLE_MS) {
        resetRecoverySamples();
        enterRecoveryState(SCAN_SAMPLE, now);
      }
      return;

    case SCAN_SAMPLE:
      recoverySampleTotal += opticalScore();
      recoverySampleCount++;
      if (recoverySampleCount >= SCAN_SAMPLE_COUNT) {
        const uint32_t candidateScore =
            recoverySampleTotal / recoverySampleCount;
        if (candidateScore > recoveryBestScore) {
          recoveryBestScore = candidateScore;
          recoveryBestPan = panAngle;
          recoveryBestTilt = tiltAngle;
        }

        scanPointIndex++;
        if (scanPointIndex < SCAN_POINT_COUNT) {
          setScanTarget(scanPointIndex);
          enterRecoveryState(SCAN_MOVE, now);
        } else {
          recoveryTargetPan = recoveryBestPan;
          recoveryTargetTilt = recoveryBestTilt;
          enterRecoveryState(RETURN_TO_BEST, now);
        }
      }
      return;

    case RETURN_TO_BEST:
      if (slewRecoveryServos()) {
        enterRecoveryState(FINAL_SETTLE, now);
      }
      return;

    case FINAL_SETTLE:
      if (now - recoveryStateStartedMs >= FINAL_SETTLE_MS) {
        finishRecovery(now);
      }
      return;
  }
}

bool cloudConfigured() {
  return SECRET_SSID[0] != '\0' &&
         SECRET_CH_ID != 0 &&
         SECRET_WRITE_APIKEY[0] != '\0';
}

void beginWifiAttempt(unsigned long now) {
  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(SECRET_SSID);

  WiFi.begin(SECRET_SSID, SECRET_PASS);
  lastWifiAttemptMs = now;
  wifiAttemptStarted = true;
}

void serviceWifi(unsigned long now) {
  if (!cloudConfigured()) return;

  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiWasConnected) {
      wifiWasConnected = true;
      Serial.print("Wi-Fi connected. IP: ");
      Serial.println(WiFi.localIP());
    }
    return;
  }

  if (wifiWasConnected) {
    wifiWasConnected = false;
    Serial.println("Wi-Fi disconnected; reconnecting automatically.");
  }

  if (!wifiAttemptStarted ||
      now - lastWifiAttemptMs >= WIFI_RETRY_INTERVAL_MS) {
    beginWifiAttempt(now);
  }
}

void serviceThingSpeak(unsigned long now) {
  if (!cloudConfigured() || WiFi.status() != WL_CONNECTED) return;

  if (thingSpeakUploadAttempted &&
      now - lastThingSpeakUploadMs < THINGSPEAK_UPLOAD_INTERVAL_MS) {
    return;
  }

  // One write sends all six values to the same ThingSpeak entry.
  ThingSpeak.setField(1, topLeft);
  ThingSpeak.setField(2, topRight);
  ThingSpeak.setField(3, bottomLeft);
  ThingSpeak.setField(4, bottomRight);
  ThingSpeak.setField(5, panAngle);
  ThingSpeak.setField(6, tiltAngle);

  lastThingSpeakUploadMs = now;
  thingSpeakUploadAttempted = true;

  const int statusCode =
      ThingSpeak.writeFields(SECRET_CH_ID, SECRET_WRITE_APIKEY);
  if (statusCode == 200) {
    Serial.println("ThingSpeak update successful.");
  } else {
    Serial.print("ThingSpeak update failed, code: ");
    Serial.println(statusCode);
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  Serial.print("ESP32 Wi-Fi MAC: ");
  Serial.println(WiFi.macAddress());
  ThingSpeak.begin(thingSpeakClient);

  if (cloudConfigured()) {
    beginWifiAttempt(millis());
  } else {
    Serial.println("ThingSpeak disabled: fill in secrets.h first.");
  }

  analogReadResolution(12);
  analogSetPinAttenuation(LDR_TOP_LEFT_PIN, ADC_11db);
  analogSetPinAttenuation(LDR_TOP_RIGHT_PIN, ADC_11db);
  analogSetPinAttenuation(LDR_BOTTOM_LEFT_PIN, ADC_11db);
  analogSetPinAttenuation(LDR_BOTTOM_RIGHT_PIN, ADC_11db);

  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);
  panServo.attach(
      PAN_SERVO_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  tiltServo.attach(
      TILT_SERVO_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);

  panServo.write(panAngle);
  tiltServo.write(tiltAngle);

  Serial.println("Adaptive four-LDR solar tracker started.");
  Serial.println("Lower ADC value is treated as brighter.");
  Serial.println("Persistent joint limits trigger automatic light reacquisition.");
}

void loop() {
  const unsigned long now = millis();

  if (now - lastUpdateMs >= UPDATE_INTERVAL_MS) {
    lastUpdateMs = now;

    updateLdrReadings();

    if (trackerMode == TRACKING) {
      int requestedPanDelta = 0;
      int requestedTiltDelta = 0;

      if (abs(horizontalError) > LIGHT_DEADBAND) {
        requestedPanDelta =
            signOf(horizontalError) *
            effectivePanDirection() *
            movementStep(abs(horizontalError));
      }
      if (abs(verticalError) > LIGHT_DEADBAND) {
        requestedTiltDelta =
            signOf(verticalError) *
            TILT_DIRECTION *
            movementStep(abs(verticalError));
      }

      const bool panBlockedAtMin =
          panAngle <= PAN_MIN_COMMAND && requestedPanDelta < 0 &&
          abs(horizontalError) >= LIMIT_TRIGGER_ERROR;
      const bool panBlockedAtMax =
          panAngle >= PAN_MAX_COMMAND && requestedPanDelta > 0 &&
          abs(horizontalError) >= LIMIT_TRIGGER_ERROR;
      const bool tiltBlockedAtMin =
          tiltAngle <= TILT_MIN_COMMAND && requestedTiltDelta < 0 &&
          abs(verticalError) >= LIMIT_TRIGGER_ERROR;
      const bool tiltBlockedAtMax =
          tiltAngle >= TILT_MAX_COMMAND && requestedTiltDelta > 0 &&
          abs(verticalError) >= LIMIT_TRIGGER_ERROR;
      const bool panBlockedAtZenith =
          effectivePanDirection() == 0 &&
          abs(horizontalError) >= LIMIT_TRIGGER_ERROR;

      LimitReason limitReason = NO_LIMIT;
      if (panBlockedAtZenith) limitReason = PAN_ZENITH_SINGULARITY;
      if (panBlockedAtMin) limitReason = PAN_MIN_LIMIT;
      if (panBlockedAtMax) limitReason = PAN_MAX_LIMIT;
      if (tiltBlockedAtMin) limitReason = TILT_MIN_LIMIT;
      if (tiltBlockedAtMax) limitReason = TILT_MAX_LIMIT;
      updateLimitRequest(limitReason, now);

      // Once recovery starts, it owns both servos until it has measured and
      // returned to the best reachable pose.
      if (trackerMode == TRACKING) {
        if (requestedPanDelta != 0 && !panBlockedAtMin && !panBlockedAtMax) {
          panAngle = constrain(
              panAngle + requestedPanDelta,
              PAN_MIN_COMMAND,
              PAN_MAX_COMMAND);
          panServo.write(panAngle);
        }

        if (requestedTiltDelta != 0 && !tiltBlockedAtMin && !tiltBlockedAtMax) {
          tiltAngle = constrain(
              tiltAngle + requestedTiltDelta,
              TILT_MIN_COMMAND,
              TILT_MAX_COMMAND);
          tiltServo.write(tiltAngle);
        }
      }
    } else {
      serviceRecovery(now);
    }
  }

  if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;

    Serial.print("TL/TR/BL/BR=");
    Serial.print(topLeft);
    Serial.print('/');
    Serial.print(topRight);
    Serial.print('/');
    Serial.print(bottomLeft);
    Serial.print('/');
    Serial.print(bottomRight);
    Serial.print("  error H/V=");
    Serial.print(horizontalError);
    Serial.print('/');
    Serial.print(verticalError);
    Serial.print("  command pan/tilt=");
    Serial.print(panAngle);
    Serial.print('/');
    Serial.print(tiltAngle);
    Serial.print("  score=");
    Serial.print(opticalScore());
    Serial.print("  mode=");
    Serial.println(trackerModeName());
  }

  serviceWifi(now);
  serviceThingSpeak(now);

  // Yield to the ESP32 Wi-Fi stack without slowing the 20 ms tracker loop.
  delay(1);
}
