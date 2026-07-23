#include <Arduino.h>
#include <ESP32Servo.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <math.h>
#include <time.h>

#include "secrets.h"
#include <ThingSpeak.h>

// Optional exact coordinates. If these are not present in secrets.h, the
// firmware estimates location from the Wi-Fi network's public IP address.
#ifndef SECRET_LATITUDE
#define SECRET_LATITUDE 999.0
#endif
#ifndef SECRET_LONGITUDE
#define SECRET_LONGITUDE 999.0
#endif

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
// 0 and 180 map to the configured pulse endpoints below. The real
// physical travel may be 120, 180, 270, or another value depending on servo.
const int PAN_MIN_COMMAND = 0;
const int PAN_MAX_COMMAND = 180;
const int TILT_MIN_COMMAND = 0;
const int TILT_MAX_COMMAND = 180;

// Experimental maximum pulse span for additional SG90 travel. Individual
// servos vary: reduce an endpoint immediately if the servo buzzes, stalls, or
// stops moving before the command reaches 0 or 180.
const int PAN_SERVO_MIN_PULSE_US = 500;
const int PAN_SERVO_MAX_PULSE_US = 2500;
const int TILT_SERVO_MIN_PULSE_US = 500;
const int TILT_SERVO_MAX_PULSE_US = 2500;

// Command where the panel normal aligns with the bottom servo's pan axis.
const int TILT_AXIS_ALIGNMENT_COMMAND = 90;
const int LIGHT_DEADBAND = 30;
const unsigned long UPDATE_INTERVAL_MS = 20;
const unsigned long PRINT_INTERVAL_MS = 100;
const unsigned long WIFI_RETRY_INTERVAL_MS = 15000;
const unsigned long THINGSPEAK_UPLOAD_INTERVAL_MS = 20000;

// Automatic time, location, and solar feed-forward settings.
const char NTP_SERVER_1[] = "pool.ntp.org";
const char NTP_SERVER_2[] = "time.nist.gov";
const char GEOLOCATION_URL[] =
    "https://ipwho.is/?fields=latitude,longitude&output=csv";
const unsigned long GEOLOCATION_RETRY_INTERVAL_MS = 10UL * 60UL * 1000UL;
const unsigned long SUN_UPDATE_INTERVAL_MS = 30000;
const unsigned long SUN_LOCK_CONFIRM_MS = 2000;
const unsigned long PASSIVE_SUN_LOCK_WAIT_MS = 5000;
const unsigned long ORIENTATION_NO_PROGRESS_MS = 3000;
const unsigned long SUN_GUIDANCE_STEP_INTERVAL_MS = 100;
const int MIN_LDR_TRACKING_BRIGHTNESS = 800;
const int LDR_TRACKING_ENTER_BRIGHTNESS = 1000;
const int LDR_TRACKING_EXIT_BRIGHTNESS = 600;
const int MIN_SUN_LOCK_BRIGHTNESS = 4000;
const int MIN_SUN_LOCK_SCORE = 1000;
const int ORIENTATION_PROGRESS_MARGIN = 20;
const int LDR_SATURATION_MARGIN = 20;
const float MIN_CALIBRATION_SUN_ELEVATION_DEG = 5.0f;
const float MAX_GUIDANCE_SUN_ELEVATION_DEG = 85.0f;
const float MAX_ANCHOR_GUIDANCE_CHANGE_DEG = 20.0f;
const float PAN_DEGREES_PER_COMMAND = 1.0f;
const float TILT_DEGREES_PER_COMMAND = 1.0f;
const double UNSET_COORDINATE = 999.0;

// H(s) = 1 / (tau*s + 1), discretized as an exponential moving average.
// At a 20 ms sample interval, tau=80 ms gives alpha=0.20: quick, but much
// less sensitive to ADC noise than one unfiltered conversion.
const float LDR_FILTER_TIME_CONSTANT_MS = 80.0f;

// Joint-limit recovery settings. A limit must be requested continuously before
// a scan starts, so one noisy reading cannot send the panel across its range.
const int LIMIT_TRIGGER_ERROR = 80;
const int MIN_RECOVERY_BRIGHTNESS = 500;
const unsigned long LIMIT_CONFIRM_MS = 250;
const int AXIS_ALIGNMENT_PAN_HOLD_COMMANDS = 5;
const int RECOVERY_SLEW_COMMANDS = 6;
const unsigned long SCAN_SETTLE_MS = 400;
const int SCAN_SAMPLE_COUNT = 4;
const unsigned long FINAL_SETTLE_MS = 500;
const int SCAN_GRID_SIZE = 5;
const int SCAN_POINT_COUNT = SCAN_GRID_SIZE * SCAN_GRID_SIZE;
const int FLIP_ACCEPT_PERCENT = 3;

// Scan-and-hold behavior. After a full scan, no servo commands are issued
// until a very large directional light-pattern change persists.
const unsigned long STARTUP_SCAN_DELAY_MS = 1000;
const unsigned long MIN_LIGHT_POSE_HOLD_MS = 30000;
const unsigned long RESCAN_CONFIRM_MS = 5000;
const int RESCAN_ERROR_CHANGE = 30;

Servo panServo;
Servo tiltServo;
WiFiClient thingSpeakClient;
Preferences solarPreferences;

struct SunPosition {
  double azimuthDeg;
  double elevationDeg;
  bool aboveHorizon;
};

struct GeolocationResult {
  bool taskRunning;
  bool resultReady;
  bool succeeded;
  double latitudeDeg;
  double longitudeDeg;
};

struct ThingSpeakSnapshot {
  int ldrTopLeft;
  int ldrTopRight;
  int ldrBottomLeft;
  int ldrBottomRight;
  int panCommand;
  int tiltCommand;
  bool sunValid;
  float sunAzimuthDeg;
  float sunElevationDeg;
};

struct ThingSpeakTaskState {
  bool taskRunning;
  bool resultReady;
  int statusCode;
  ThingSpeakSnapshot snapshot;
};

portMUX_TYPE geolocationMux = portMUX_INITIALIZER_UNLOCKED;
GeolocationResult geolocationResult = {false, false, false, 0.0, 0.0};
portMUX_TYPE thingSpeakMux = portMUX_INITIALIZER_UNLOCKED;
ThingSpeakTaskState thingSpeakTaskState = {
    false, false, 0, {0, 0, 0, 0, 90, 90, false, 0.0f, 0.0f}};

double latitudeDeg = 0.0;
double longitudeDeg = 0.0;
bool locationValid = false;
bool locationIsManual = false;
bool automaticLocationResolved = false;
String locationSource = "none";

bool ntpStarted = false;
bool timeValid = false;
bool sunPositionValid = false;
bool sunCalibratable = false;
SunPosition sunPosition = {0.0, -90.0, false};
unsigned long sunBecameCalibratableMs = 0;
unsigned long lastSunUpdateMs = 0;
unsigned long lastGeolocationAttemptMs = 0;
bool geolocationAttempted = false;

bool orientationAnchorValid = false;
bool orientationScanAttempted = false;
bool lightPoseLocked = false;
bool rescanTimerRunning = false;
bool sunLockTimerRunning = false;
bool orientationProgressInitialized = false;
bool ldrTrackingActive = true;
bool singularityLockReported = false;
unsigned long sunLockStartedMs = 0;
unsigned long lightPoseLockedMs = 0;
unsigned long rescanRequestStartedMs = 0;
unsigned long lastOrientationProgressMs = 0;
int bestOrientationError = 32767;
int lockedHorizontalError = 0;
int lockedVerticalError = 0;
double anchorSunAzimuthDeg = 0.0;
double anchorSunElevationDeg = 0.0;
int anchorPanCommand = 90;
int anchorTiltCommand = 90;
int anchorPanParity = 0;
unsigned long lastOrientationAnchorMs = 0;
unsigned long lastSunGuidanceStepMs = 0;

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
bool recoveryHoldActive = false;
int recoveryHoldHorizontalError = 0;
int recoveryHoldVerticalError = 0;

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

bool coordinatesValid(double latitude, double longitude) {
  return isfinite(latitude) && isfinite(longitude) &&
         latitude >= -90.0 && latitude <= 90.0 &&
         longitude >= -180.0 && longitude <= 180.0;
}

double wrapDegrees(double angle) {
  angle = fmod(angle, 360.0);
  return angle < 0.0 ? angle + 360.0 : angle;
}

double shortestAngleDifference(double target, double origin) {
  double difference = wrapDegrees(target - origin + 180.0) - 180.0;
  return difference <= -180.0 ? difference + 360.0 : difference;
}

bool isLeapYear(int year) {
  return (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
}

bool estimateSunPosition(
    time_t utcEpoch,
    double latitude,
    double longitudeEast,
    SunPosition &result) {
  struct tm utc;
  if (gmtime_r(&utcEpoch, &utc) == nullptr) return false;

  const double pi = 3.14159265358979323846;
  const double degreesToRadians = pi / 180.0;
  const double radiansToDegrees = 180.0 / pi;
  const int year = utc.tm_year + 1900;
  const int daysInYear = isLeapYear(year) ? 366 : 365;
  const double utcMinutes =
      utc.tm_hour * 60.0 + utc.tm_min + utc.tm_sec / 60.0;
  const double utcHour = utcMinutes / 60.0;

  // NOAA fractional-year approximation. tm_yday is already zero-based.
  const double gamma =
      2.0 * pi / daysInYear *
      (utc.tm_yday + (utcHour - 12.0) / 24.0);
  const double equationOfTimeMinutes =
      229.18 *
      (0.000075 +
       0.001868 * cos(gamma) -
       0.032077 * sin(gamma) -
       0.014615 * cos(2.0 * gamma) -
       0.040849 * sin(2.0 * gamma));
  const double declination =
      0.006918 -
      0.399912 * cos(gamma) +
      0.070257 * sin(gamma) -
      0.006758 * cos(2.0 * gamma) +
      0.000907 * sin(2.0 * gamma) -
      0.002697 * cos(3.0 * gamma) +
      0.001480 * sin(3.0 * gamma);

  double trueSolarMinutes =
      fmod(utcMinutes + equationOfTimeMinutes + 4.0 * longitudeEast, 1440.0);
  if (trueSolarMinutes < 0.0) trueSolarMinutes += 1440.0;

  const double hourAngle =
      (trueSolarMinutes / 4.0 - 180.0) * degreesToRadians;
  const double latitudeRadians = latitude * degreesToRadians;
  double sinElevation =
      sin(latitudeRadians) * sin(declination) +
      cos(latitudeRadians) * cos(declination) * cos(hourAngle);
  sinElevation = max(-1.0, min(1.0, sinElevation));

  const double elevation = asin(sinElevation);
  const double azimuthY = sin(hourAngle);
  const double azimuthX =
      cos(hourAngle) * sin(latitudeRadians) -
      tan(declination) * cos(latitudeRadians);

  result.elevationDeg = elevation * radiansToDegrees;
  result.azimuthDeg = wrapDegrees(
      atan2(azimuthY, azimuthX) * radiansToDegrees + 180.0);
  result.aboveHorizon = result.elevationDeg > 0.0;
  return true;
}

void applyLocation(
    double latitude,
    double longitude,
    const char *source,
    bool persist) {
  if (!coordinatesValid(latitude, longitude)) return;

  const bool locationChangedMaterially =
      locationValid &&
      (fabs(latitude - latitudeDeg) > 0.01 ||
       fabs(longitude - longitudeDeg) > 0.01);

  latitudeDeg = latitude;
  longitudeDeg = longitude;
  locationValid = true;
  locationSource = source;
  sunPositionValid = false;
  sunCalibratable = false;

  if (locationChangedMaterially) {
    orientationAnchorValid = false;
    orientationScanAttempted = false;
    sunLockTimerRunning = false;
    orientationProgressInitialized = false;
    Serial.println(
        "Location changed: clearing the previous sun orientation anchor.");
  }

  if (persist) {
    solarPreferences.putDouble("latitude", latitude);
    solarPreferences.putDouble("longitude", longitude);
    solarPreferences.putBool("locationValid", true);
  }

  Serial.print("Location ready [");
  Serial.print(source);
  Serial.print("]: ");
  Serial.print(latitudeDeg, 5);
  Serial.print(", ");
  Serial.println(longitudeDeg, 5);
}

void loadLocationConfiguration() {
  const double configuredLatitude = (double)SECRET_LATITUDE;
  const double configuredLongitude = (double)SECRET_LONGITUDE;
  if (configuredLatitude != UNSET_COORDINATE &&
      configuredLongitude != UNSET_COORDINATE &&
      coordinatesValid(configuredLatitude, configuredLongitude)) {
    locationIsManual = true;
    automaticLocationResolved = true;
    applyLocation(
        configuredLatitude, configuredLongitude, "secrets.h exact", false);
    return;
  }

  if (solarPreferences.getBool("locationValid", false)) {
    const double cachedLatitude = solarPreferences.getDouble("latitude", 0.0);
    const double cachedLongitude = solarPreferences.getDouble("longitude", 0.0);
    if (coordinatesValid(cachedLatitude, cachedLongitude)) {
      applyLocation(cachedLatitude, cachedLongitude, "cached IP estimate", false);
    }
  }
}

void geolocationTask(void *parameter) {
  bool succeeded = false;
  double latitude = 0.0;
  double longitude = 0.0;

  if (WiFi.status() == WL_CONNECTED) {
    WiFiClientSecure secureClient;
    secureClient.setInsecure();

    HTTPClient http;
    http.setConnectTimeout(5000);
    http.setTimeout(5000);
    http.setUserAgent("COSMOS-ESP32-Solar-Tracker/1.0");

    if (http.begin(secureClient, GEOLOCATION_URL)) {
      const int statusCode = http.GET();
      const int contentLength = http.getSize();
      if (statusCode == HTTP_CODE_OK &&
          contentLength > 0 &&
          contentLength <= 64) {
        String body = http.getString();
        body.trim();
        const char *latitudeStart = body.c_str();
        char *latitudeEnd = nullptr;
        const double parsedLatitude =
            strtod(latitudeStart, &latitudeEnd);
        if (latitudeEnd != latitudeStart &&
            latitudeEnd != nullptr &&
            *latitudeEnd == ',') {
          const char *longitudeStart = latitudeEnd + 1;
          char *longitudeEnd = nullptr;
          const double parsedLongitude =
              strtod(longitudeStart, &longitudeEnd);
          latitude = parsedLatitude;
          longitude = parsedLongitude;
          succeeded =
              longitudeEnd != longitudeStart &&
              *longitudeEnd == '\0' &&
              coordinatesValid(latitude, longitude);
        }
      }
      http.end();
    }
  }

  portENTER_CRITICAL(&geolocationMux);
  geolocationResult.succeeded = succeeded;
  geolocationResult.latitudeDeg = latitude;
  geolocationResult.longitudeDeg = longitude;
  geolocationResult.resultReady = true;
  geolocationResult.taskRunning = false;
  portEXIT_CRITICAL(&geolocationMux);

  vTaskDelete(nullptr);
}

bool geolocationTaskIsRunning() {
  portENTER_CRITICAL(&geolocationMux);
  const bool running = geolocationResult.taskRunning;
  portEXIT_CRITICAL(&geolocationMux);
  return running;
}

bool startGeolocationTask(unsigned long now) {
  portENTER_CRITICAL(&geolocationMux);
  if (geolocationResult.taskRunning) {
    portEXIT_CRITICAL(&geolocationMux);
    return false;
  }
  geolocationResult.taskRunning = true;
  geolocationResult.resultReady = false;
  portEXIT_CRITICAL(&geolocationMux);

  geolocationAttempted = true;
  lastGeolocationAttemptMs = now;
  const BaseType_t created = xTaskCreatePinnedToCore(
      geolocationTask,
      "ip-geolocation",
      8192,
      nullptr,
      1,
      nullptr,
      0);
  if (created != pdPASS) {
    portENTER_CRITICAL(&geolocationMux);
    geolocationResult.taskRunning = false;
    portEXIT_CRITICAL(&geolocationMux);
    Serial.println("Could not start the background geolocation task.");
    return false;
  }

  Serial.println("Estimating location from public IP in the background...");
  return true;
}

void thingSpeakUploadTask(void *parameter) {
  ThingSpeakSnapshot snapshot;
  portENTER_CRITICAL(&thingSpeakMux);
  snapshot = thingSpeakTaskState.snapshot;
  portEXIT_CRITICAL(&thingSpeakMux);

  ThingSpeak.setField(1, snapshot.ldrTopLeft);
  ThingSpeak.setField(2, snapshot.ldrTopRight);
  ThingSpeak.setField(3, snapshot.ldrBottomLeft);
  ThingSpeak.setField(4, snapshot.ldrBottomRight);
  ThingSpeak.setField(5, snapshot.panCommand);
  ThingSpeak.setField(6, snapshot.tiltCommand);
  if (snapshot.sunValid) {
    ThingSpeak.setField(7, snapshot.sunAzimuthDeg);
    ThingSpeak.setField(8, snapshot.sunElevationDeg);
  }

  const int statusCode =
      ThingSpeak.writeFields(SECRET_CH_ID, SECRET_WRITE_APIKEY);

  portENTER_CRITICAL(&thingSpeakMux);
  thingSpeakTaskState.statusCode = statusCode;
  thingSpeakTaskState.resultReady = true;
  thingSpeakTaskState.taskRunning = false;
  portEXIT_CRITICAL(&thingSpeakMux);

  vTaskDelete(nullptr);
}

bool thingSpeakTaskIsRunning() {
  portENTER_CRITICAL(&thingSpeakMux);
  const bool running = thingSpeakTaskState.taskRunning;
  portEXIT_CRITICAL(&thingSpeakMux);
  return running;
}

bool startThingSpeakUploadTask(unsigned long now) {
  portENTER_CRITICAL(&thingSpeakMux);
  if (thingSpeakTaskState.taskRunning) {
    portEXIT_CRITICAL(&thingSpeakMux);
    return false;
  }

  thingSpeakTaskState.snapshot.ldrTopLeft = topLeft;
  thingSpeakTaskState.snapshot.ldrTopRight = topRight;
  thingSpeakTaskState.snapshot.ldrBottomLeft = bottomLeft;
  thingSpeakTaskState.snapshot.ldrBottomRight = bottomRight;
  thingSpeakTaskState.snapshot.panCommand = panAngle;
  thingSpeakTaskState.snapshot.tiltCommand = tiltAngle;
  thingSpeakTaskState.snapshot.sunValid = sunPositionValid;
  thingSpeakTaskState.snapshot.sunAzimuthDeg =
      (float)sunPosition.azimuthDeg;
  thingSpeakTaskState.snapshot.sunElevationDeg =
      (float)sunPosition.elevationDeg;
  thingSpeakTaskState.taskRunning = true;
  thingSpeakTaskState.resultReady = false;
  portEXIT_CRITICAL(&thingSpeakMux);

  lastThingSpeakUploadMs = now;
  thingSpeakUploadAttempted = true;
  const BaseType_t created = xTaskCreatePinnedToCore(
      thingSpeakUploadTask,
      "thingspeak-upload",
      8192,
      nullptr,
      1,
      nullptr,
      0);
  if (created != pdPASS) {
    portENTER_CRITICAL(&thingSpeakMux);
    thingSpeakTaskState.taskRunning = false;
    portEXIT_CRITICAL(&thingSpeakMux);
    Serial.println("Could not start the background ThingSpeak task.");
    return false;
  }
  return true;
}

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

uint32_t scanLightScore() {
  // The scan target is the pose receiving the greatest total measured light.
  return (uint32_t)max(0, totalBrightness());
}

const char *trackerModeName() {
  if (trackerMode == TRACKING && lightPoseLocked) return "LIGHT-LOCK";
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
  recoveryBestPan = panAngle;
  recoveryBestTilt = tiltAngle;
  recoveryBestScore = scanLightScore();
  scanPointIndex = 0;
  setScanTarget(scanPointIndex);
  enterRecoveryState(SCAN_MOVE, now);
  Serial.println("Starting one bounded 5x5 brightest-light scan.");
}

void finishRecovery(unsigned long now) {
  trackerMode = TRACKING;
  pendingLimitReason = NO_LIMIT;
  limitTimerRunning = false;
  lightPoseLocked = true;
  lightPoseLockedMs = now;
  rescanTimerRunning = false;
  lockedHorizontalError = horizontalError;
  lockedVerticalError = verticalError;
  recoveryHoldActive = true;
  recoveryHoldHorizontalError = horizontalError;
  recoveryHoldVerticalError = verticalError;

  Serial.print("Light scan complete. Brightest score/pose: ");
  Serial.print(recoveryBestScore);
  Serial.print(" at ");
  Serial.print(panAngle);
  Serial.print('/');
  Serial.println(tiltAngle);
  Serial.println(
      "Servo commands are now locked; normal sensor updates cannot move them.");
}

void startRecovery(LimitReason reason, unsigned long now) {
  if (orientationAnchorValid) {
    orientationAnchorValid = false;
    sunLockTimerRunning = false;
    orientationProgressInitialized = false;
    Serial.println(
        "Recovery changed servo geometry: clearing the sun anchor.");
  }

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

void beginOrientationScan(unsigned long now) {
  orientationScanAttempted = true;
  lightPoseLocked = false;
  rescanTimerRunning = false;
  sunLockTimerRunning = false;
  orientationProgressInitialized = false;
  pendingLimitReason = NO_LIMIT;
  limitTimerRunning = false;
  recoveryHoldActive = false;
  recoveryOriginPan = panAngle;
  recoveryOriginTilt = tiltAngle;
  recoveryOriginScore = scanLightScore();
  recoveryBestPan = panAngle;
  recoveryBestTilt = tiltAngle;
  recoveryBestScore = recoveryOriginScore;

  Serial.println("Starting automatic full-range light acquisition.");
  beginGridScan(now);
}

void captureOrientationAnchor(unsigned long now) {
  const int panParity = effectivePanDirection();
  if (panParity == 0) {
    orientationScanAttempted = true;
    if (!singularityLockReported) {
      Serial.println(
          "Sun lock is near zenith; elevation is known but azimuth anchoring "
          "waits until the panel leaves the pan singularity.");
      singularityLockReported = true;
    }
    return;
  }

  anchorSunAzimuthDeg = sunPosition.azimuthDeg;
  anchorSunElevationDeg = sunPosition.elevationDeg;
  anchorPanCommand = panAngle;
  anchorTiltCommand = tiltAngle;
  anchorPanParity = panParity;
  orientationAnchorValid = true;
  orientationScanAttempted = true;
  singularityLockReported = false;
  lastOrientationAnchorMs = now;

  Serial.print("Panel direction detected from stable sun lock: az/el=");
  Serial.print(anchorSunAzimuthDeg, 1);
  Serial.print('/');
  Serial.print(anchorSunElevationDeg, 1);
  Serial.print(" deg at command ");
  Serial.print(anchorPanCommand);
  Serial.print('/');
  Serial.println(anchorTiltCommand);
}

bool estimatePanelDirection(double &azimuth, double &elevation) {
  if (!orientationAnchorValid) return false;

  if (panAngle == anchorPanCommand && tiltAngle == anchorTiltCommand) {
    azimuth = anchorSunAzimuthDeg;
    elevation = anchorSunElevationDeg;
    return true;
  }

  const int currentPanParity = effectivePanDirection();
  if (anchorPanParity == 0 || currentPanParity != anchorPanParity) return false;

  azimuth = wrapDegrees(
      anchorSunAzimuthDeg +
      (panAngle - anchorPanCommand) *
          anchorPanParity * PAN_DEGREES_PER_COMMAND);
  elevation =
      anchorSunElevationDeg +
      (tiltAngle - anchorTiltCommand) *
          TILT_DIRECTION * TILT_DEGREES_PER_COMMAND;
  return elevation >= -90.0 && elevation <= 90.0;
}

void serviceOrientationDetection(unsigned long now) {
  const bool sensorsNotSaturated =
      topLeft > LDR_SATURATION_MARGIN &&
      topRight > LDR_SATURATION_MARGIN &&
      bottomLeft > LDR_SATURATION_MARGIN &&
      bottomRight > LDR_SATURATION_MARGIN &&
      topLeft < 4095 - LDR_SATURATION_MARGIN &&
      topRight < 4095 - LDR_SATURATION_MARGIN &&
      bottomLeft < 4095 - LDR_SATURATION_MARGIN &&
      bottomRight < 4095 - LDR_SATURATION_MARGIN;
  const bool stableSunLock =
      trackerMode == TRACKING &&
      sunPositionValid &&
      sunPosition.elevationDeg >= MIN_CALIBRATION_SUN_ELEVATION_DEG &&
      totalBrightness() >= MIN_SUN_LOCK_BRIGHTNESS &&
      opticalScore() >= MIN_SUN_LOCK_SCORE &&
      sensorsNotSaturated &&
      abs(horizontalError) <= LIGHT_DEADBAND &&
      abs(verticalError) <= LIGHT_DEADBAND;

  if (stableSunLock) {
    if (!sunLockTimerRunning) {
      sunLockTimerRunning = true;
      sunLockStartedMs = now;
    } else if (now - sunLockStartedMs >= SUN_LOCK_CONFIRM_MS &&
               (!orientationAnchorValid ||
                now - lastOrientationAnchorMs >= SUN_UPDATE_INTERVAL_MS ||
                abs(panAngle - anchorPanCommand) >= 2 ||
                abs(tiltAngle - anchorTiltCommand) >= 2)) {
      captureOrientationAnchor(now);
      sunLockStartedMs = now;
    }
  } else {
    sunLockTimerRunning = false;
  }

  const bool orientationAcquisitionActive =
      !orientationAnchorValid &&
      !orientationScanAttempted &&
      !lightPoseLocked &&
      trackerMode == TRACKING &&
      sunCalibratable &&
      totalBrightness() >= MIN_LDR_TRACKING_BRIGHTNESS;
  if (orientationAcquisitionActive) {
    const int currentError =
        abs(horizontalError) + abs(verticalError);
    if (!orientationProgressInitialized) {
      orientationProgressInitialized = true;
      bestOrientationError = currentError;
      lastOrientationProgressMs = now;
    } else if (currentError + ORIENTATION_PROGRESS_MARGIN <
               bestOrientationError) {
      bestOrientationError = currentError;
      lastOrientationProgressMs = now;
    }
  } else if (!sunCalibratable || orientationAnchorValid) {
    orientationProgressInitialized = false;
  }

  // Give ordinary LDR tracking time to converge. Only a persistent lack of
  // improvement can start the single measured orientation scan.
  if (!orientationAnchorValid &&
      !orientationScanAttempted &&
      !lightPoseLocked &&
      trackerMode == TRACKING &&
      sunCalibratable &&
      orientationProgressInitialized &&
      now - sunBecameCalibratableMs >= PASSIVE_SUN_LOCK_WAIT_MS &&
      now - lastOrientationProgressMs >= ORIENTATION_NO_PROGRESS_MS) {
    beginOrientationScan(now);
  }
}

void serviceSunGuidance(unsigned long now) {
  if (!orientationAnchorValid ||
      anchorPanParity == 0 ||
      effectivePanDirection() != anchorPanParity ||
      !sunPositionValid ||
      sunPosition.elevationDeg < MIN_CALIBRATION_SUN_ELEVATION_DEG ||
      sunPosition.elevationDeg > MAX_GUIDANCE_SUN_ELEVATION_DEG ||
      trackerMode != TRACKING ||
      now - lastSunGuidanceStepMs < SUN_GUIDANCE_STEP_INTERVAL_MS) {
    return;
  }

  const double azimuthChange = shortestAngleDifference(
      sunPosition.azimuthDeg, anchorSunAzimuthDeg);
  const double elevationChange =
      sunPosition.elevationDeg - anchorSunElevationDeg;
  if (fabs(azimuthChange) > MAX_ANCHOR_GUIDANCE_CHANGE_DEG ||
      fabs(elevationChange) > MAX_ANCHOR_GUIDANCE_CHANGE_DEG) {
    return;
  }

  const int targetPan = constrain(
      anchorPanCommand +
          (int)lround(
              azimuthChange * anchorPanParity / PAN_DEGREES_PER_COMMAND),
      PAN_MIN_COMMAND,
      PAN_MAX_COMMAND);
  const int targetTilt = constrain(
      anchorTiltCommand +
          (int)lround(
              elevationChange * TILT_DIRECTION /
              TILT_DEGREES_PER_COMMAND),
      TILT_MIN_COMMAND,
      TILT_MAX_COMMAND);

  // A local one-point calibration is not trustworthy across the pan-axis
  // singularity. Let measured LDR recovery handle that geometry instead.
  const bool crossesTiltAxis =
      (anchorTiltCommand < TILT_AXIS_ALIGNMENT_COMMAND &&
       targetTilt >= TILT_AXIS_ALIGNMENT_COMMAND) ||
      (anchorTiltCommand > TILT_AXIS_ALIGNMENT_COMMAND &&
       targetTilt <= TILT_AXIS_ALIGNMENT_COMMAND);
  if (crossesTiltAxis) return;

  const int nextPan = approachCommand(panAngle, targetPan, 1);
  const int nextTilt = approachCommand(tiltAngle, targetTilt, 1);
  if (nextPan != panAngle) {
    panAngle = nextPan;
    panServo.write(panAngle);
  }
  if (nextTilt != tiltAngle) {
    tiltAngle = nextTilt;
    tiltServo.write(tiltAngle);
  }
  lastSunGuidanceStepMs = now;
}

void serviceLightPoseLock(unsigned long now) {
  if (!lightPoseLocked ||
      !ldrTrackingActive ||
      now - lightPoseLockedMs < MIN_LIGHT_POSE_HOLD_MS) {
    rescanTimerRunning = false;
    return;
  }

  const bool patternChanged =
      abs(horizontalError - lockedHorizontalError) >= RESCAN_ERROR_CHANGE ||
      abs(verticalError - lockedVerticalError) >= RESCAN_ERROR_CHANGE;
  if (!patternChanged) {
    rescanTimerRunning = false;
    return;
  }

  if (!rescanTimerRunning) {
    rescanTimerRunning = true;
    rescanRequestStartedMs = now;
    return;
  }

  if (now - rescanRequestStartedMs < RESCAN_CONFIRM_MS) return;

  Serial.println(
      "Large sustained light-direction change detected: rescanning once.");
  lightPoseLocked = false;
  orientationAnchorValid = false;
  orientationScanAttempted = false;
  beginOrientationScan(now);
}

void updateLimitRequest(LimitReason reason, unsigned long now) {
  if (reason == NO_LIMIT) {
    pendingLimitReason = NO_LIMIT;
    limitTimerRunning = false;
    const bool strongPatternChange =
        ldrTrackingActive &&
        (abs(horizontalError - recoveryHoldHorizontalError) >
             LIGHT_DEADBAND ||
         abs(verticalError - recoveryHoldVerticalError) >
             LIGHT_DEADBAND);
    if (recoveryHoldActive && strongPatternChange) {
      recoveryHoldActive = false;
      Serial.println("LDR pattern changed: recovered pose re-armed.");
    }
    return;
  }

  if (recoveryHoldActive) {
    const bool lightChanged =
        ldrTrackingActive &&
        (abs(horizontalError - recoveryHoldHorizontalError) >
             LIGHT_DEADBAND ||
         abs(verticalError - recoveryHoldVerticalError) >
             LIGHT_DEADBAND);
    if (!lightChanged) {
      pendingLimitReason = reason;
      limitTimerRunning = false;
      return;
    }

    // Require the changed LDR pattern and the blocked direction to persist
    // together. A single filtered spike cannot launch another full scan.
    if (!limitTimerRunning || reason != pendingLimitReason) {
      pendingLimitReason = reason;
      limitRequestStartedMs = now;
      limitTimerRunning = true;
      return;
    }

    if (now - limitRequestStartedMs >= LIMIT_CONFIRM_MS &&
        totalBrightness() >= MIN_RECOVERY_BRIGHTNESS) {
      recoveryHoldActive = false;
      Serial.println("Persistent LDR change: joint-limit recovery re-armed.");
      startRecovery(reason, now);
    }
    return;
  }

  if (!limitTimerRunning || reason != pendingLimitReason) {
    pendingLimitReason = reason;
    limitRequestStartedMs = now;
    limitTimerRunning = true;
    return;
  }

  if (now - limitRequestStartedMs >= LIMIT_CONFIRM_MS &&
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
      recoverySampleTotal += scanLightScore();
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

bool wifiConfigured() {
  return SECRET_SSID[0] != '\0';
}

bool thingSpeakConfigured() {
  return wifiConfigured() &&
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
  if (!wifiConfigured()) return;

  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiWasConnected) {
      wifiWasConnected = true;
      Serial.print("Wi-Fi connected. IP: ");
      Serial.println(WiFi.localIP());

      if (!ntpStarted) {
        configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2);
        ntpStarted = true;
        Serial.println("NTP synchronization requested (UTC).");
      }
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

void serviceGeolocation(unsigned long now) {
  bool resultReady = false;
  bool succeeded = false;
  double resultLatitude = 0.0;
  double resultLongitude = 0.0;

  portENTER_CRITICAL(&geolocationMux);
  if (geolocationResult.resultReady) {
    resultReady = true;
    succeeded = geolocationResult.succeeded;
    resultLatitude = geolocationResult.latitudeDeg;
    resultLongitude = geolocationResult.longitudeDeg;
    geolocationResult.resultReady = false;
  }
  portEXIT_CRITICAL(&geolocationMux);

  if (resultReady) {
    if (succeeded) {
      automaticLocationResolved = true;
      applyLocation(
          resultLatitude, resultLongitude, "public-IP estimate", true);
    } else {
      Serial.println(
          "Automatic IP location failed; cached location remains active.");
    }
  }

  if (locationIsManual || automaticLocationResolved ||
      WiFi.status() != WL_CONNECTED ||
      geolocationTaskIsRunning() ||
      thingSpeakTaskIsRunning()) {
    return;
  }

  if (!geolocationAttempted ||
      now - lastGeolocationAttemptMs >= GEOLOCATION_RETRY_INTERVAL_MS) {
    startGeolocationTask(now);
  }
}

void serviceSunEstimator(unsigned long now) {
  const time_t utcEpoch = time(nullptr);
  const bool synchronized = utcEpoch >= 1700000000;
  if (!synchronized || !locationValid) return;

  if (!timeValid) {
    timeValid = true;
    Serial.println("UTC date/time synchronized automatically.");
  }

  if (sunPositionValid &&
      now - lastSunUpdateMs < SUN_UPDATE_INTERVAL_MS) {
    return;
  }

  SunPosition estimate;
  if (!estimateSunPosition(
          utcEpoch, latitudeDeg, longitudeDeg, estimate)) {
    return;
  }

  sunPosition = estimate;
  sunPositionValid = true;
  lastSunUpdateMs = now;
  const bool nowCalibratable =
      sunPosition.elevationDeg >= MIN_CALIBRATION_SUN_ELEVATION_DEG;
  if (nowCalibratable && !sunCalibratable) {
    sunBecameCalibratableMs = now;
    orientationProgressInitialized = false;
  }
  sunCalibratable = nowCalibratable;

  struct tm utc;
  gmtime_r(&utcEpoch, &utc);
  char utcText[24];
  strftime(utcText, sizeof(utcText), "%Y-%m-%d %H:%M:%S", &utc);
  Serial.print("Sun estimate UTC=");
  Serial.print(utcText);
  Serial.print(" az/el=");
  Serial.print(sunPosition.azimuthDeg, 1);
  Serial.print('/');
  Serial.print(sunPosition.elevationDeg, 1);
  Serial.println(" deg");
}

void serviceThingSpeak(unsigned long now) {
  bool resultReady = false;
  int completedStatusCode = 0;
  portENTER_CRITICAL(&thingSpeakMux);
  if (thingSpeakTaskState.resultReady) {
    resultReady = true;
    completedStatusCode = thingSpeakTaskState.statusCode;
    thingSpeakTaskState.resultReady = false;
  }
  portEXIT_CRITICAL(&thingSpeakMux);

  if (resultReady) {
    if (completedStatusCode == 200) {
      Serial.println("ThingSpeak update successful.");
    } else {
      Serial.print("ThingSpeak update failed, code: ");
      Serial.println(completedStatusCode);
    }
  }

  if (!thingSpeakConfigured() ||
      WiFi.status() != WL_CONNECTED ||
      geolocationTaskIsRunning() ||
      thingSpeakTaskIsRunning()) {
    return;
  }

  if (thingSpeakUploadAttempted &&
      now - lastThingSpeakUploadMs < THINGSPEAK_UPLOAD_INTERVAL_MS) {
    return;
  }

  startThingSpeakUploadTask(now);
}

void setup() {
  Serial.begin(115200);

  solarPreferences.begin("solartrack", false);
  loadLocationConfiguration();

  WiFi.mode(WIFI_STA);
  Serial.print("ESP32 Wi-Fi MAC: ");
  Serial.println(WiFi.macAddress());
  ThingSpeak.begin(thingSpeakClient);

  if (wifiConfigured()) {
    beginWifiAttempt(millis());
  } else {
    Serial.println(
        "Wi-Fi time/location and ThingSpeak disabled: set SECRET_SSID.");
  }
  if (!thingSpeakConfigured()) {
    Serial.println("ThingSpeak disabled: set channel ID and Write API key.");
  }

  analogReadResolution(12);
  analogSetPinAttenuation(LDR_TOP_LEFT_PIN, ADC_11db);
  analogSetPinAttenuation(LDR_TOP_RIGHT_PIN, ADC_11db);
  analogSetPinAttenuation(LDR_BOTTOM_LEFT_PIN, ADC_11db);
  analogSetPinAttenuation(LDR_BOTTOM_RIGHT_PIN, ADC_11db);

  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);
  panServo.attach(
      PAN_SERVO_PIN, PAN_SERVO_MIN_PULSE_US, PAN_SERVO_MAX_PULSE_US);
  tiltServo.attach(
      TILT_SERVO_PIN, TILT_SERVO_MIN_PULSE_US, TILT_SERVO_MAX_PULSE_US);

  panServo.write(panAngle);
  tiltServo.write(tiltAngle);

  Serial.println("Four-LDR brightest-light scan-and-hold tracker started.");
  Serial.println("Lower ADC value is treated as brighter.");
  Serial.print("LDR balance deadband: +/-");
  Serial.print(LIGHT_DEADBAND);
  Serial.println(" ADC counts.");
  Serial.println(
      "One full-range scan starts after 1 second, then mode=LIGHT-LOCK.");
  Serial.println(
      "A new scan requires a >=30 ADC pattern change sustained for 5 seconds.");
}

void loop() {
  const unsigned long now = millis();

  if (now - lastUpdateMs >= UPDATE_INTERVAL_MS) {
    lastUpdateMs = now;

    updateLdrReadings();
    const int currentBrightness = totalBrightness();
    if (ldrTrackingActive &&
        currentBrightness < LDR_TRACKING_EXIT_BRIGHTNESS) {
      ldrTrackingActive = false;
    } else if (!ldrTrackingActive &&
               currentBrightness > LDR_TRACKING_ENTER_BRIGHTNESS) {
      ldrTrackingActive = true;
    }
    serviceOrientationDetection(now);

    if (trackerMode != TRACKING) {
      serviceRecovery(now);
    } else if (lightPoseLocked) {
      // Strict hold: this path never writes either servo.
      serviceLightPoseLock(now);
    } else if (!orientationScanAttempted &&
               now >= STARTUP_SCAN_DELAY_MS) {
      beginOrientationScan(now);
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
    Serial.print("  total light=");
    Serial.print(scanLightScore());
    Serial.print("  mode=");
    Serial.print(trackerModeName());
    if (sunPositionValid) {
      Serial.print("  sun az/el=");
      Serial.print(sunPosition.azimuthDeg, 1);
      Serial.print('/');
      Serial.print(sunPosition.elevationDeg, 1);
    }
    double panelAzimuth = 0.0;
    double panelElevation = 0.0;
    if (estimatePanelDirection(panelAzimuth, panelElevation)) {
      Serial.print("  panel az/el=");
      Serial.print(panelAzimuth, 1);
      Serial.print('/');
      Serial.print(panelElevation, 1);
    } else if (sunPositionValid) {
      Serial.print("  panel direction=acquiring");
    }
    Serial.println();
  }

  serviceWifi(now);
  serviceGeolocation(now);
  serviceSunEstimator(now);
  serviceThingSpeak(now);

  // Yield to the ESP32 Wi-Fi stack without slowing the 20 ms tracker loop.
  delay(1);
}
