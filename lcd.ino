#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <math.h>

/*
  Dual-axis solar tracker

  Coarse pointing:
    A two-pass pan/tilt light scan finds the brightest reachable direction, so
    the level base may face any compass heading. Date/time and location still
    determine day/night when a trusted clock is available.

  Fine pointing:
    Four LDRs arranged around an opaque cross make gain-scheduled corrections
    until the left/right and top/bottom light levels are approximately equal.

  ESP32-WROVER breadboard module wiring (change constants below if needed):
    GPIO25 -> bottom pan/azimuth positional-servo signal
    GPIO26 -> top tilt/elevation positional-servo signal
    GPIO34 -> top-left LDR divider
    GPIO35 -> top-right LDR divider
    GPIO36/VP -> bottom-left LDR divider
    GPIO39/VN -> bottom-right LDR divider
    GPIO21 -> DS3231 SDA (optional, but strongly recommended)
    GPIO22 -> DS3231 SCL

  IMPORTANT:
    - ESP32 GPIO is NOT 5 V tolerant. Power every LDR divider from 3.3 V, so
      its midpoint can never exceed 3.3 V.
    - Make sure any DS3231 module pulls SDA/SCL up to 3.3 V, not 5 V.
    - Power the servos from a separate regulated 5-6 V supply and connect its
      ground to ESP32 GND. Do not run two loaded servos from the dev board.
    - The pan actuator must be a positional servo. A continuous-rotation servo
      cannot point to an angle unless an encoder is added.
    - A hobby servo gives the ESP32 no position feedback. On first power-up,
      place the mechanism at its configured park angles; a truly unattended
      mount needs homing switches or encoders to prevent an unknown boot move.
    - Start with conservative mechanical limits, then calibrate them.
*/

// ---------------------------------------------------------------------------
// Site and clock configuration
// ---------------------------------------------------------------------------

// Default example: Los Angeles. Latitude is +north/-south; longitude is
// +east/-west. Replace these with the installation's actual coordinates.
// A fixed ESP32 has no reliable way to discover location without GPS or an
// external network service, so these site coordinates are intentionally
// explicit and remain available when the tracker is offline.
const float LATITUDE_DEG = 34.0522f;
const float LONGITUDE_DEG = -118.2437f;

// Recommended: keep the DS3231 in UTC and enter UTC with the SET command. That
// avoids daylight-saving changes, so the offset stays zero all year. If you
// deliberately store local civil time instead, set its effective UTC offset
// here (PDT=-7, PST=-8, India=+5.5, etc.) and update it when DST changes.
const float UTC_OFFSET_HOURS = 0.0f;

const uint8_t RTC_I2C_ADDRESS = 0x68;

// ---------------------------------------------------------------------------
// Pin configuration
// ---------------------------------------------------------------------------

const uint8_t I2C_SDA_PIN = 21;
const uint8_t I2C_SCL_PIN = 22;

const uint8_t PAN_SERVO_PIN = 25;
const uint8_t TILT_SERVO_PIN = 26;

// LDR positions are named while looking at the sun-facing side of the panel.
// These are ADC1 pins, which remain usable if Wi-Fi is added later. GPIOs
// 34-39 are input-only, making them a good fit for the four analog sensors.
const uint8_t LDR_TOP_LEFT_PIN = 34;
const uint8_t LDR_TOP_RIGHT_PIN = 35;
const uint8_t LDR_BOTTOM_LEFT_PIN = 36;
const uint8_t LDR_BOTTOM_RIGHT_PIN = 39;

const uint16_t ADC_MAX_READING = 4095;  // ESP32 12-bit ADC.

// ---------------------------------------------------------------------------
// Mechanical calibration
// ---------------------------------------------------------------------------

// Conservative endpoints protect the mechanism during initial testing.
const int PAN_SERVO_MIN_DEG = 10;
const int PAN_SERVO_MAX_DEG = 170;
const int TILT_SERVO_MIN_DEG = 15;
const int TILT_SERVO_MAX_DEG = 165;

// ESP32Servo maps 0-180 degrees onto this pulse range. These initial values
// avoid extreme pulses; widen them only after checking the servo datasheet and
// the mechanism by hand.
const int SERVO_MIN_PULSE_US = 1000;
const int SERVO_MAX_PULSE_US = 2000;

// At PAN_SERVO_CENTER_DEG the panel normal points toward
// PAN_CENTER_AZIMUTH_DEG. Azimuth is true bearing: N=0, E=90, S=180, W=270.
// A northern-hemisphere installation commonly centers on true south.
const int PAN_SERVO_CENTER_DEG = 90;
const float PAN_CENTER_AZIMUTH_DEG = 180.0f;
const float PAN_SERVO_DEG_PER_AZIMUTH_DEG = 1.0f;
const int PAN_ASTRONOMY_DIRECTION = 1;  // Change to -1 if coarse pan is reversed.

// For orientation-independent scanning, install the top-servo horn so the
// PANEL NORMAL faces straight upward at 90 degrees. The two safe endpoints
// then approach opposite horizons. Calibrate these if the linkage differs.
const int TILT_SERVO_AT_HORIZON_DEG = 15;
const int TILT_SERVO_AT_ZENITH_DEG = 90;
const int TILT_SERVO_AT_OPPOSITE_HORIZON_DEG = 165;

// Fine-control polarity is independent from the astronomical mapping.
// If brighter right-hand LDRs move the panel left, reverse PAN_FINE_DIRECTION.
// If brighter top LDRs move the panel down, reverse TILT_FINE_DIRECTION.
const int PAN_FINE_DIRECTION = 1;
const int TILT_FINE_DIRECTION = 1;

// Park should be adjusted for the safest low-wind position of the real frame.
const int PAN_PARK_DEG = 90;
const int TILT_PARK_DEG = 20;

// Allow a substantial optical search while the absolute servo endpoints still
// protect the mechanism. Astronomy supplies the starting direction when time
// is available; the LDR controller then closes the remaining error.
const int PAN_FINE_TRIM_LIMIT_DEG = 60;
const int TILT_FINE_TRIM_LIMIT_DEG = 50;

// ---------------------------------------------------------------------------
// LDR calibration and controller tuning
// ---------------------------------------------------------------------------

// This prototype has the fixed resistor to 3.3 V and the LDR to GND, so more
// light lowers the ADC reading. The controller converts that reading to an
// internal brightness value before comparing left/right and top/bottom.
const bool LDR_BRIGHTER_IS_HIGHER = false;

// Use these gains to compensate for small sensor/resistor mismatches.
const float LDR_TOP_LEFT_GAIN = 1.0f;
const float LDR_TOP_RIGHT_GAIN = 1.0f;
const float LDR_BOTTOM_LEFT_GAIN = 1.0f;
const float LDR_BOTTOM_RIGHT_GAIN = 1.0f;

const uint8_t LDR_SAMPLES_PER_READING = 8;

// Two cascaded first-order low-pass stages implement
//
//                  1
//   H(s) = -------------------
//          (tau*s + 1)^2
//
// Each pole is converted to a real-time difference equation with backward
// Euler: y[k] = y[k-1] + alpha*(x[k]-y[k-1]),
// where alpha = dt/(tau+dt). This suppresses ADC/servo noise while retaining
// a predictable physical time constant. Increase tau for smoother/slower data.
const float LDR_LOWPASS_TIME_CONSTANT_SECONDS = 0.10f;

const float LDR_MIN_AVERAGE_BRIGHTNESS = 320.0f;
const float LDR_SATURATED_AVERAGE_BRIGHTNESS = 4040.0f;
const float LDR_MIN_PAIR_DIFFERENCE = 24.0f;
const float LDR_RAIL_MARGIN = 12.0f;
const uint8_t LDR_RAIL_FAULT_SAMPLES = 40;  // 2 seconds at 20 samples/second.

// Error is 1000 * (positiveSide-negativeSide)/(positiveSide+negativeSide).
// 30 therefore means a 3% normalized mismatch.
const int LDR_DEADBAND_PERMILLE = 30;
const int LDR_COARSE_ERROR_PERMILLE = 300;
const int LDR_MEDIUM_ERROR_PERMILLE = 150;
const int LDR_SMALL_ERROR_PERMILLE = 70;
const int LDR_COARSE_STEP_DEG = 8;
const int LDR_MEDIUM_STEP_DEG = 5;
const int LDR_SMALL_STEP_DEG = 2;
const int LDR_FINE_STEP_DEG = 1;
const uint8_t LDR_REQUIRED_SAME_DIRECTION_SAMPLES = 2;

// ---------------------------------------------------------------------------
// Timing and day/night behavior
// ---------------------------------------------------------------------------

const unsigned long ASTRONOMY_UPDATE_MS = 60000UL;
const unsigned long LDR_UPDATE_MS = 50UL;
const unsigned long SERVO_STEP_INTERVAL_MS = 20UL;
const unsigned long SERVO_SETTLE_MS = 250UL;
const unsigned long STATUS_INTERVAL_MS = 100UL;

// A two-pass optical raster makes the tracker independent of its compass
// heading. Astronomy is still used for day/night gating when time is trusted,
// but it does not overwrite the angles found by this search.
const bool ORIENTATION_INDEPENDENT_OPTICAL_MODE = true;
const int SEARCH_COARSE_STEP_DEG = 40;
const int SEARCH_REFINE_RADIUS_DEG = 20;
const int SEARCH_REFINE_STEP_DEG = 10;
const uint8_t SEARCH_POINT_SAMPLES = 4;
const uint8_t SEARCH_MIN_VALID_POINT_SAMPLES = 2;
const uint8_t SEARCH_START_VALID_SAMPLES = 10;
const uint8_t SEARCH_CONFIRM_SAMPLES = 6;
const float SEARCH_MIN_START_BRIGHTNESS = 50.0f;
const float SEARCH_MIN_SCORE_SPREAD = 80.0f;
const float SEARCH_CONFIRM_RATIO = 0.55f;
const int SEARCH_SERVO_STEP_DEG = 2;
const int PAN_ZENITH_DEADBAND_DEG = 4;

// Hysteresis prevents repeated day/night switching around the horizon.
const float ENTER_DAY_ELEVATION_DEG = 2.0f;
const float ENTER_NIGHT_ELEVATION_DEG = -3.0f;
const float MIN_FINE_TRACK_ELEVATION_DEG = 3.0f;

// ---------------------------------------------------------------------------
// Data types and state
// ---------------------------------------------------------------------------

struct DateTimeValue {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};

struct SunPosition {
  float azimuthDeg;
  float elevationDeg;
  bool azimuthStable;
};

struct LdrValues {
  float topLeft;
  float topRight;
  float bottomLeft;
  float bottomRight;
  float average;
  float left;
  float right;
  float top;
  float bottom;
  int horizontalErrorPermille;  // Positive means right is brighter.
  int verticalErrorPermille;    // Positive means top is brighter.
};

struct RawLdrValues {
  uint16_t topLeft;
  uint16_t topRight;
  uint16_t bottomLeft;
  uint16_t bottomRight;
};

struct TwoPoleLowPass {
  float pole1;
  float pole2;
  bool initialized;
};

enum SearchPhase : uint8_t {
  SEARCH_WAITING_FOR_LIGHT,
  SEARCH_COARSE_GRID,
  SEARCH_REFINE_GRID,
  SEARCH_MOVE_TO_BEST,
  SEARCH_LOCKED,
  SEARCH_FAILED,
  SEARCH_STOPPED
};

Servo panServo;
Servo tiltServo;

DateTimeValue softwareClock;
unsigned long softwareClockReferenceMs = 0;
bool rtcAvailable = false;
bool timeTrusted = false;
bool servosReady = false;

SunPosition latestSun = {180.0f, -90.0f, true};
LdrValues latestLdr = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
RawLdrValues latestRawLdr = {0, 0, 0, 0};
TwoPoleLowPass topLeftFilter = {0, 0, false};
TwoPoleLowPass topRightFilter = {0, 0, false};
TwoPoleLowPass bottomLeftFilter = {0, 0, false};
TwoPoleLowPass bottomRightFilter = {0, 0, false};
unsigned long previousLdrFilterMs = 0;
bool ldrRailFault = false;
uint8_t ldrRailSuspicionCount = 0;

SearchPhase searchPhase = SEARCH_WAITING_FOR_LIGHT;
int searchPanMinDeg = PAN_SERVO_MIN_DEG;
int searchPanMaxDeg = PAN_SERVO_MAX_DEG;
int searchTiltMinDeg = TILT_SERVO_MIN_DEG;
int searchTiltMaxDeg = TILT_SERVO_MAX_DEG;
int searchStepDeg = SEARCH_COARSE_STEP_DEG;
int searchPanDeg = PAN_SERVO_MIN_DEG;
int searchTiltDeg = TILT_SERVO_MIN_DEG;
int8_t searchPanDirection = 1;
int searchBestPanDeg = PAN_PARK_DEG;
int searchBestTiltDeg = TILT_PARK_DEG;
float searchBestScore = -1.0f;
float searchLowestScore = 1000000.0f;
float searchBestBalanceError = 1000000.0f;
float searchPointScoreSum = 0.0f;
float searchPointBalanceSum = 0.0f;
uint8_t searchPointSampleCount = 0;
uint8_t searchPointValidCount = 0;
uint8_t searchStartValidCount = 0;
uint16_t searchPointsVisited = 0;

bool daylightStateInitialized = false;
bool daylight = false;
bool basePanInitialized = false;

float basePanDeg = PAN_SERVO_CENTER_DEG;
float baseTiltDeg = TILT_SERVO_AT_HORIZON_DEG;
int panFineTrimDeg = 0;
int tiltFineTrimDeg = 0;

int desiredPanDeg = PAN_PARK_DEG;
int desiredTiltDeg = TILT_PARK_DEG;
int actualPanDeg = PAN_PARK_DEG;
int actualTiltDeg = TILT_PARK_DEG;

int8_t previousHorizontalErrorSign = 0;
int8_t previousVerticalErrorSign = 0;
uint8_t horizontalErrorPersistence = 0;
uint8_t verticalErrorPersistence = 0;

unsigned long lastAstronomyMs = 0;
unsigned long lastLdrMs = 0;
unsigned long lastServoStepMs = 0;
unsigned long lastServoMoveMs = 0;
unsigned long lastStatusMs = 0;

char serialLine[36];
uint8_t serialLineLength = 0;

// ---------------------------------------------------------------------------
// General helpers
// ---------------------------------------------------------------------------

bool isLeapYear(uint16_t year) {
  return (year % 4 == 0) && ((year % 100 != 0) || (year % 400 == 0));
}

uint8_t daysInMonth(uint16_t year, uint8_t month) {
  static const uint8_t DAYS[] = {31, 28, 31, 30, 31, 30,
                                 31, 31, 30, 31, 30, 31};
  if (month < 1 || month > 12) return 0;
  if (month == 2 && isLeapYear(year)) return 29;
  return DAYS[month - 1];
}

bool validDateTime(const DateTimeValue &value) {
  if (value.year < 2000 || value.year > 2199) return false;
  if (value.month < 1 || value.month > 12) return false;
  if (value.day < 1 || value.day > daysInMonth(value.year, value.month)) return false;
  if (value.hour > 23 || value.minute > 59 || value.second > 59) return false;
  return true;
}

float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

int clampInt(int value, int low, int high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

int roundedInt(float value) {
  return (int)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

float wrap360(float angle) {
  angle = fmodf(angle, 360.0f);
  if (angle < 0.0f) angle += 360.0f;
  return angle;
}

float wrap180(float angle) {
  angle = fmodf(angle + 180.0f, 360.0f);
  if (angle < 0.0f) angle += 360.0f;
  return angle - 180.0f;
}

int8_t signOf(int value) {
  if (value > 0) return 1;
  if (value < 0) return -1;
  return 0;
}

// ---------------------------------------------------------------------------
// DS3231 and fallback software clock
// ---------------------------------------------------------------------------

uint8_t bcdToBinary(uint8_t value) {
  return (value & 0x0F) + 10 * ((value >> 4) & 0x0F);
}

uint8_t binaryToBcd(uint8_t value) {
  return ((value / 10) << 4) | (value % 10);
}

bool readRtcRegister(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(RTC_I2C_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(RTC_I2C_ADDRESS, (uint8_t)1) != 1) return false;
  value = Wire.read();
  return true;
}

bool writeRtcRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(RTC_I2C_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool rtcOscillatorIsRunning() {
  uint8_t status;
  if (!readRtcRegister(0x0F, status)) return false;
  return (status & 0x80) == 0;  // OSF is set after oscillator/power failure.
}

bool readRtc(DateTimeValue &value) {
  Wire.beginTransmission(RTC_I2C_ADDRESS);
  Wire.write((uint8_t)0x00);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(RTC_I2C_ADDRESS, (uint8_t)7) != 7) return false;

  const uint8_t secondRegister = Wire.read();
  const uint8_t minuteRegister = Wire.read();
  const uint8_t hourRegister = Wire.read();
  Wire.read();  // Day of week is not used.
  const uint8_t dayRegister = Wire.read();
  const uint8_t monthRegister = Wire.read();
  const uint8_t yearRegister = Wire.read();

  value.second = bcdToBinary(secondRegister & 0x7F);
  value.minute = bcdToBinary(minuteRegister & 0x7F);

  if (hourRegister & 0x40) {  // Convert DS3231 12-hour mode if necessary.
    uint8_t hour12 = bcdToBinary(hourRegister & 0x1F);
    const bool isPm = (hourRegister & 0x20) != 0;
    value.hour = hour12 % 12;
    if (isPm) value.hour += 12;
  } else {
    value.hour = bcdToBinary(hourRegister & 0x3F);
  }

  value.day = bcdToBinary(dayRegister & 0x3F);
  value.month = bcdToBinary(monthRegister & 0x1F);
  value.year = 2000 + bcdToBinary(yearRegister);
  if (monthRegister & 0x80) value.year += 100;

  return validDateTime(value);
}

bool writeRtc(const DateTimeValue &value) {
  if (!validDateTime(value)) return false;

  Wire.beginTransmission(RTC_I2C_ADDRESS);
  Wire.write((uint8_t)0x00);
  Wire.write(binaryToBcd(value.second));
  Wire.write(binaryToBcd(value.minute));
  Wire.write(binaryToBcd(value.hour));  // Forces 24-hour mode.
  Wire.write(binaryToBcd(1));           // Day of week is unused here.
  Wire.write(binaryToBcd(value.day));

  uint8_t monthRegister = binaryToBcd(value.month);
  if (value.year >= 2100) monthRegister |= 0x80;
  Wire.write(monthRegister);
  Wire.write(binaryToBcd(value.year % 100));
  if (Wire.endTransmission() != 0) return false;

  uint8_t status;
  if (readRtcRegister(0x0F, status)) {
    writeRtcRegister(0x0F, status & 0x7F);  // Clear oscillator-stop flag.
  }
  return true;
}

uint8_t buildMonthNumber(const char *dateText) {
  const char first = dateText[0];
  const char second = dateText[1];
  const char third = dateText[2];
  if (first == 'J' && second == 'a') return 1;
  if (first == 'F') return 2;
  if (first == 'M' && third == 'r') return 3;
  if (first == 'A' && second == 'p') return 4;
  if (first == 'M' && third == 'y') return 5;
  if (first == 'J' && third == 'n') return 6;
  if (first == 'J' && third == 'l') return 7;
  if (first == 'A' && second == 'u') return 8;
  if (first == 'S') return 9;
  if (first == 'O') return 10;
  if (first == 'N') return 11;
  return 12;
}

DateTimeValue buildTime() {
  const char *dateText = __DATE__;  // "Mmm dd yyyy"
  const char *timeText = __TIME__;  // "hh:mm:ss"
  DateTimeValue value;
  value.month = buildMonthNumber(dateText);
  value.day = (dateText[4] == ' ' ? 0 : dateText[4] - '0') * 10 +
              (dateText[5] - '0');
  value.year = (dateText[7] - '0') * 1000 + (dateText[8] - '0') * 100 +
               (dateText[9] - '0') * 10 + (dateText[10] - '0');
  value.hour = (timeText[0] - '0') * 10 + (timeText[1] - '0');
  value.minute = (timeText[3] - '0') * 10 + (timeText[4] - '0');
  value.second = (timeText[6] - '0') * 10 + (timeText[7] - '0');
  return value;
}

void setSoftwareClock(const DateTimeValue &value) {
  softwareClock = value;
  softwareClockReferenceMs = millis();
}

void incrementOneSecond(DateTimeValue &value) {
  if (++value.second < 60) return;
  value.second = 0;
  if (++value.minute < 60) return;
  value.minute = 0;
  if (++value.hour < 24) return;
  value.hour = 0;
  if (++value.day <= daysInMonth(value.year, value.month)) return;
  value.day = 1;
  if (++value.month <= 12) return;
  value.month = 1;
  ++value.year;
}

void advanceSoftwareClock() {
  const unsigned long elapsedMs = millis() - softwareClockReferenceMs;
  unsigned long elapsedSeconds = elapsedMs / 1000UL;
  if (elapsedSeconds == 0) return;
  softwareClockReferenceMs += elapsedSeconds * 1000UL;
  while (elapsedSeconds-- > 0) incrementOneSecond(softwareClock);
}

DateTimeValue currentDateTime() {
  if (rtcAvailable) {
    DateTimeValue rtcTime;
    if (readRtc(rtcTime) && rtcOscillatorIsRunning()) {
      setSoftwareClock(rtcTime);  // Keeps a seamless fallback if I2C fails later.
      return rtcTime;
    }
    rtcAvailable = false;
    Serial.println(F("WARNING: RTC read failed; continuing with software clock."));
  }

  advanceSoftwareClock();
  return softwareClock;
}

// ---------------------------------------------------------------------------
// Solar position (compact NOAA approximation)
// ---------------------------------------------------------------------------

uint16_t dayOfYear(const DateTimeValue &value) {
  uint16_t result = value.day;
  for (uint8_t month = 1; month < value.month; ++month) {
    result += daysInMonth(value.year, month);
  }
  return result;
}

SunPosition calculateSunPosition(const DateTimeValue &timeValue) {
  const float DEG_TO_RAD_F = PI / 180.0f;
  const float RAD_TO_DEG_F = 180.0f / PI;
  const float yearLength = isLeapYear(timeValue.year) ? 366.0f : 365.0f;
  const float localHour = timeValue.hour + timeValue.minute / 60.0f +
                          timeValue.second / 3600.0f;

  const float gamma = 2.0f * PI / yearLength *
                      (dayOfYear(timeValue) - 1.0f +
                       (localHour - 12.0f) / 24.0f);

  const float equationOfTimeMinutes = 229.18f *
      (0.000075f + 0.001868f * cosf(gamma) -
       0.032077f * sinf(gamma) - 0.014615f * cosf(2.0f * gamma) -
       0.040849f * sinf(2.0f * gamma));

  const float declination =
      0.006918f - 0.399912f * cosf(gamma) + 0.070257f * sinf(gamma) -
      0.006758f * cosf(2.0f * gamma) + 0.000907f * sinf(2.0f * gamma) -
      0.002697f * cosf(3.0f * gamma) + 0.001480f * sinf(3.0f * gamma);

  float trueSolarMinutes = timeValue.hour * 60.0f + timeValue.minute +
      timeValue.second / 60.0f + equationOfTimeMinutes +
      4.0f * LONGITUDE_DEG - 60.0f * UTC_OFFSET_HOURS;
  trueSolarMinutes = fmodf(trueSolarMinutes, 1440.0f);
  if (trueSolarMinutes < 0.0f) trueSolarMinutes += 1440.0f;

  const float hourAngle = (trueSolarMinutes / 4.0f - 180.0f) * DEG_TO_RAD_F;
  const float latitude = LATITUDE_DEG * DEG_TO_RAD_F;
  const float sinDeclination = sinf(declination);
  const float cosDeclination = cosf(declination);
  const float sinLatitude = sinf(latitude);
  const float cosLatitude = cosf(latitude);

  float sinElevation = sinLatitude * sinDeclination +
      cosLatitude * cosDeclination * cosf(hourAngle);
  sinElevation = clampFloat(sinElevation, -1.0f, 1.0f);
  const float elevation = asinf(sinElevation);

  // Local horizontal vector: atan2 avoids the usual azimuth quadrant error.
  const float east = -cosDeclination * sinf(hourAngle);
  const float north = cosLatitude * sinDeclination -
      sinLatitude * cosDeclination * cosf(hourAngle);

  SunPosition result;
  result.elevationDeg = elevation * RAD_TO_DEG_F;
  result.azimuthDeg = wrap360(atan2f(east, north) * RAD_TO_DEG_F);
  result.azimuthStable = east * east + north * north > 0.0025f;
  return result;
}

// ---------------------------------------------------------------------------
// LDR acquisition and filtering
// ---------------------------------------------------------------------------

uint16_t readTrimmedAnalog(uint8_t pin) {
  analogRead(pin);  // Discard first conversion after changing ADC channel.

  uint32_t total = 0;
  uint16_t minimum = ADC_MAX_READING;
  uint16_t maximum = 0;
  for (uint8_t sample = 0; sample < LDR_SAMPLES_PER_READING; ++sample) {
    const uint16_t reading = analogRead(pin);
    total += reading;
    if (reading < minimum) minimum = reading;
    if (reading > maximum) maximum = reading;
  }

  // Drop the lowest and highest sample to reject brief servo/ADC spikes.
  return (total - minimum - maximum) / (LDR_SAMPLES_PER_READING - 2);
}

float calibratedBrightness(uint16_t raw, float gain) {
  float brightness = LDR_BRIGHTER_IS_HIGHER ? raw : ADC_MAX_READING - raw;
  return clampFloat(brightness * gain, 0.0f, ADC_MAX_READING);
}

float updateLowPass(TwoPoleLowPass &filter, float input, float alpha) {
  if (!filter.initialized) {
    filter.pole1 = input;
    filter.pole2 = input;
    filter.initialized = true;
    return input;
  }

  filter.pole1 += alpha * (input - filter.pole1);
  filter.pole2 += alpha * (filter.pole1 - filter.pole2);
  return filter.pole2;
}

int normalizedPairError(float positiveSide, float negativeSide) {
  const float denominator = positiveSide + negativeSide;
  if (denominator < 1.0f) return 0;
  return roundedInt(1000.0f * (positiveSide - negativeSide) / denominator);
}

int adaptiveLdrStepDeg(int errorMagnitudePermille) {
  if (errorMagnitudePermille >= LDR_COARSE_ERROR_PERMILLE) {
    return LDR_COARSE_STEP_DEG;
  }
  if (errorMagnitudePermille >= LDR_MEDIUM_ERROR_PERMILLE) {
    return LDR_MEDIUM_STEP_DEG;
  }
  if (errorMagnitudePermille >= LDR_SMALL_ERROR_PERMILLE) {
    return LDR_SMALL_STEP_DEG;
  }
  return LDR_FINE_STEP_DEG;
}

bool rawLdrsOffRails() {
  return latestRawLdr.topLeft > LDR_RAIL_MARGIN &&
         latestRawLdr.topLeft < ADC_MAX_READING - LDR_RAIL_MARGIN &&
         latestRawLdr.topRight > LDR_RAIL_MARGIN &&
         latestRawLdr.topRight < ADC_MAX_READING - LDR_RAIL_MARGIN &&
         latestRawLdr.bottomLeft > LDR_RAIL_MARGIN &&
         latestRawLdr.bottomLeft < ADC_MAX_READING - LDR_RAIL_MARGIN &&
         latestRawLdr.bottomRight > LDR_RAIL_MARGIN &&
         latestRawLdr.bottomRight < ADC_MAX_READING - LDR_RAIL_MARGIN;
}

void instantaneousLdrMetrics(float &score, float &balanceError) {
  const float topLeft = calibratedBrightness(
      latestRawLdr.topLeft, LDR_TOP_LEFT_GAIN);
  const float topRight = calibratedBrightness(
      latestRawLdr.topRight, LDR_TOP_RIGHT_GAIN);
  const float bottomLeft = calibratedBrightness(
      latestRawLdr.bottomLeft, LDR_BOTTOM_LEFT_GAIN);
  const float bottomRight = calibratedBrightness(
      latestRawLdr.bottomRight, LDR_BOTTOM_RIGHT_GAIN);

  score = (topLeft + topRight + bottomLeft + bottomRight) * 0.25f;
  const float left = (topLeft + bottomLeft) * 0.5f;
  const float right = (topRight + bottomRight) * 0.5f;
  const float top = (topLeft + topRight) * 0.5f;
  const float bottom = (bottomLeft + bottomRight) * 0.5f;
  balanceError = abs(normalizedPairError(right, left)) +
                 abs(normalizedPairError(top, bottom));
}

bool currentLdrSampleUsableForSearch(float &score, float &balanceError) {
  instantaneousLdrMetrics(score, balanceError);
  return rawLdrsOffRails() && score >= SEARCH_MIN_START_BRIGHTNESS &&
         score <= LDR_SATURATED_AVERAGE_BRIGHTNESS;
}

void resetLdrFilters() {
  topLeftFilter = {0, 0, false};
  topRightFilter = {0, 0, false};
  bottomLeftFilter = {0, 0, false};
  bottomRightFilter = {0, 0, false};
  previousLdrFilterMs = 0;
}

bool nearAdcRail(float value) {
  return value <= LDR_RAIL_MARGIN ||
         value >= ADC_MAX_READING - LDR_RAIL_MARGIN;
}

void updateLdrRailFault(float topLeft, float topRight, float bottomLeft,
                        float bottomRight) {
  const float unfilteredAverage =
      (topLeft + topRight + bottomLeft + bottomRight) * 0.25f;
  const bool suspicious =
      (nearAdcRail(topLeft) || nearAdcRail(topRight) ||
       nearAdcRail(bottomLeft) || nearAdcRail(bottomRight)) &&
      unfilteredAverage >= LDR_MIN_AVERAGE_BRIGHTNESS &&
      unfilteredAverage <= LDR_SATURATED_AVERAGE_BRIGHTNESS;

  if (suspicious) {
    if (ldrRailSuspicionCount < LDR_RAIL_FAULT_SAMPLES) {
      ++ldrRailSuspicionCount;
    }
    if (ldrRailSuspicionCount >= LDR_RAIL_FAULT_SAMPLES) {
      ldrRailFault = true;
    }
  } else {
    ldrRailSuspicionCount = 0;
    ldrRailFault = false;
  }
}

void readLdrs(unsigned long sampleTimeMs) {
  latestRawLdr.topLeft = readTrimmedAnalog(LDR_TOP_LEFT_PIN);
  latestRawLdr.topRight = readTrimmedAnalog(LDR_TOP_RIGHT_PIN);
  latestRawLdr.bottomLeft = readTrimmedAnalog(LDR_BOTTOM_LEFT_PIN);
  latestRawLdr.bottomRight = readTrimmedAnalog(LDR_BOTTOM_RIGHT_PIN);

  const float topLeft = calibratedBrightness(
      latestRawLdr.topLeft, LDR_TOP_LEFT_GAIN);
  const float topRight = calibratedBrightness(
      latestRawLdr.topRight, LDR_TOP_RIGHT_GAIN);
  const float bottomLeft = calibratedBrightness(
      latestRawLdr.bottomLeft, LDR_BOTTOM_LEFT_GAIN);
  const float bottomRight = calibratedBrightness(
      latestRawLdr.bottomRight, LDR_BOTTOM_RIGHT_GAIN);

  // A rail reading that persists while the other sensors report normal light
  // commonly means a disconnected/shorted divider. Fine tracking is frozen;
  // astronomical coarse tracking remains active.
  updateLdrRailFault(topLeft, topRight, bottomLeft, bottomRight);

  float elapsedSeconds = LDR_UPDATE_MS / 1000.0f;
  if (previousLdrFilterMs != 0) {
    elapsedSeconds = (sampleTimeMs - previousLdrFilterMs) / 1000.0f;
  }
  previousLdrFilterMs = sampleTimeMs;
  elapsedSeconds = clampFloat(elapsedSeconds, 0.001f, 5.0f);
  const float alpha = elapsedSeconds /
      (LDR_LOWPASS_TIME_CONSTANT_SECONDS + elapsedSeconds);

  latestLdr.topLeft = updateLowPass(topLeftFilter, topLeft, alpha);
  latestLdr.topRight = updateLowPass(topRightFilter, topRight, alpha);
  latestLdr.bottomLeft = updateLowPass(bottomLeftFilter, bottomLeft, alpha);
  latestLdr.bottomRight = updateLowPass(bottomRightFilter, bottomRight, alpha);

  latestLdr.left = (latestLdr.topLeft + latestLdr.bottomLeft) * 0.5f;
  latestLdr.right = (latestLdr.topRight + latestLdr.bottomRight) * 0.5f;
  latestLdr.top = (latestLdr.topLeft + latestLdr.topRight) * 0.5f;
  latestLdr.bottom = (latestLdr.bottomLeft + latestLdr.bottomRight) * 0.5f;
  latestLdr.average = (latestLdr.topLeft + latestLdr.topRight +
                       latestLdr.bottomLeft + latestLdr.bottomRight) * 0.25f;
  latestLdr.horizontalErrorPermille =
      normalizedPairError(latestLdr.right, latestLdr.left);
  latestLdr.verticalErrorPermille =
      normalizedPairError(latestLdr.top, latestLdr.bottom);
}

// ---------------------------------------------------------------------------
// Motion and two-stage tracking control
// ---------------------------------------------------------------------------

void resetErrorPersistence() {
  previousHorizontalErrorSign = 0;
  previousVerticalErrorSign = 0;
  horizontalErrorPersistence = 0;
  verticalErrorPersistence = 0;
}

bool searchInProgress() {
  return searchPhase == SEARCH_COARSE_GRID ||
         searchPhase == SEARCH_REFINE_GRID ||
         searchPhase == SEARCH_MOVE_TO_BEST;
}

bool opticalSearchAllowedNow() {
  return !timeTrusted || daylight;
}

void resetSearchPointSampling() {
  searchPointScoreSum = 0.0f;
  searchPointBalanceSum = 0.0f;
  searchPointSampleCount = 0;
  searchPointValidCount = 0;
}

void commandSearchPoint(int panDeg, int tiltDeg) {
  searchPanDeg = clampInt(panDeg, PAN_SERVO_MIN_DEG, PAN_SERVO_MAX_DEG);
  searchTiltDeg = clampInt(tiltDeg, TILT_SERVO_MIN_DEG, TILT_SERVO_MAX_DEG);
  desiredPanDeg = searchPanDeg;
  desiredTiltDeg = searchTiltDeg;
  resetSearchPointSampling();
  resetLdrFilters();
}

void startSearchGrid(SearchPhase phase, int panMinDeg, int panMaxDeg,
                     int tiltMinDeg, int tiltMaxDeg, int stepDeg,
                     bool resetBest) {
  searchPhase = phase;
  searchPanMinDeg = clampInt(panMinDeg, PAN_SERVO_MIN_DEG,
                             PAN_SERVO_MAX_DEG);
  searchPanMaxDeg = clampInt(panMaxDeg, PAN_SERVO_MIN_DEG,
                             PAN_SERVO_MAX_DEG);
  searchTiltMinDeg = clampInt(tiltMinDeg, TILT_SERVO_MIN_DEG,
                              TILT_SERVO_MAX_DEG);
  searchTiltMaxDeg = clampInt(tiltMaxDeg, TILT_SERVO_MIN_DEG,
                              TILT_SERVO_MAX_DEG);
  searchStepDeg = max(1, stepDeg);
  searchPanDirection = 1;

  if (resetBest) {
    searchBestScore = -1.0f;
    searchLowestScore = 1000000.0f;
    searchBestBalanceError = 1000000.0f;
    searchBestPanDeg = PAN_PARK_DEG;
    searchBestTiltDeg = TILT_PARK_DEG;
    searchPointsVisited = 0;
  }

  commandSearchPoint(searchPanMinDeg, searchTiltMinDeg);
}

void beginOrientationSearch() {
  panFineTrimDeg = 0;
  tiltFineTrimDeg = 0;
  resetErrorPersistence();
  startSearchGrid(SEARCH_COARSE_GRID,
                  PAN_SERVO_MIN_DEG, PAN_SERVO_MAX_DEG,
                  TILT_SERVO_AT_HORIZON_DEG,
                  TILT_SERVO_AT_OPPOSITE_HORIZON_DEG,
                  SEARCH_COARSE_STEP_DEG, true);
  Serial.println(F("Optical coarse scan started."));
}

void beginRefinementSearch() {
  const int centerPan = searchBestPanDeg;
  const int centerTilt = searchBestTiltDeg;
  startSearchGrid(
      SEARCH_REFINE_GRID,
      max(PAN_SERVO_MIN_DEG, centerPan - SEARCH_REFINE_RADIUS_DEG),
      min(PAN_SERVO_MAX_DEG, centerPan + SEARCH_REFINE_RADIUS_DEG),
      max(TILT_SERVO_MIN_DEG, centerTilt - SEARCH_REFINE_RADIUS_DEG),
      min(TILT_SERVO_MAX_DEG, centerTilt + SEARCH_REFINE_RADIUS_DEG),
      SEARCH_REFINE_STEP_DEG, false);
  Serial.println(F("Optical refinement scan started."));
}

void failOrientationSearch(const __FlashStringHelper *reason) {
  searchPhase = SEARCH_FAILED;
  panFineTrimDeg = 0;
  tiltFineTrimDeg = 0;
  desiredPanDeg = clampInt(PAN_PARK_DEG, PAN_SERVO_MIN_DEG,
                           PAN_SERVO_MAX_DEG);
  desiredTiltDeg = clampInt(TILT_PARK_DEG, TILT_SERVO_MIN_DEG,
                            TILT_SERVO_MAX_DEG);
  Serial.print(F("Optical scan failed: "));
  Serial.println(reason);
}

void finishSearchGridPoint() {
  ++searchPointsVisited;

  if (searchPointValidCount >= SEARCH_MIN_VALID_POINT_SAMPLES) {
    const float pointScore =
        searchPointScoreSum / searchPointValidCount;
    const float pointBalance =
        searchPointBalanceSum / searchPointValidCount;
    searchLowestScore = min(searchLowestScore, pointScore);

    const bool stronger = pointScore > searchBestScore + 1.0f;
    const bool equallyStrong = fabsf(pointScore - searchBestScore) <= 1.0f;
    if (stronger || (equallyStrong &&
                     pointBalance < searchBestBalanceError)) {
      searchBestScore = pointScore;
      searchBestBalanceError = pointBalance;
      searchBestPanDeg = searchPanDeg;
      searchBestTiltDeg = searchTiltDeg;
    }
  }

  const bool atRowEnd = searchPanDirection > 0
      ? searchPanDeg >= searchPanMaxDeg
      : searchPanDeg <= searchPanMinDeg;
  const bool atLastTilt = searchTiltDeg >= searchTiltMaxDeg;

  if (atRowEnd && atLastTilt) {
    if (searchPhase == SEARCH_COARSE_GRID) {
      if (searchBestScore < 0.0f) {
        failOrientationSearch(F("no valid LDR point"));
      } else {
        beginRefinementSearch();
      }
    } else {
      if (searchBestScore < 0.0f ||
          searchLowestScore >= 999999.0f ||
          searchBestScore - searchLowestScore < SEARCH_MIN_SCORE_SPREAD) {
        failOrientationSearch(F("light did not vary across the scan"));
      } else {
        searchPhase = SEARCH_MOVE_TO_BEST;
        commandSearchPoint(searchBestPanDeg, searchBestTiltDeg);
        Serial.println(F("Optical scan returning to strongest point."));
      }
    }
    return;
  }

  if (atRowEnd) {
    searchTiltDeg = min(searchTiltDeg + searchStepDeg, searchTiltMaxDeg);
    searchPanDirection = -searchPanDirection;
  } else if (searchPanDirection > 0) {
    searchPanDeg = min(searchPanDeg + searchStepDeg, searchPanMaxDeg);
  } else {
    searchPanDeg = max(searchPanDeg - searchStepDeg, searchPanMinDeg);
  }
  commandSearchPoint(searchPanDeg, searchTiltDeg);
}

void lockStrongestSearchPoint(float confirmedScore) {
  basePanDeg = searchBestPanDeg;
  baseTiltDeg = searchBestTiltDeg;
  basePanInitialized = true;
  panFineTrimDeg = 0;
  tiltFineTrimDeg = 0;
  resetErrorPersistence();
  resetLdrFilters();
  searchPhase = SEARCH_LOCKED;
  desiredPanDeg = searchBestPanDeg;
  desiredTiltDeg = searchBestTiltDeg;
  lastServoMoveMs = millis();
  Serial.print(F("Optical lock acquired at pan/tilt="));
  Serial.print(searchBestPanDeg);
  Serial.print('/');
  Serial.print(searchBestTiltDeg);
  Serial.print(F(" score="));
  Serial.println(confirmedScore, 0);
}

void serviceOrientationSearchOnLdrSample() {
  if (!ORIENTATION_INDEPENDENT_OPTICAL_MODE) return;

  if (!opticalSearchAllowedNow()) {
    if (searchPhase != SEARCH_STOPPED) {
      searchPhase = SEARCH_WAITING_FOR_LIGHT;
      searchStartValidCount = 0;
      panFineTrimDeg = 0;
      tiltFineTrimDeg = 0;
      updateDesiredAngles();
    }
    return;
  }

  if (searchPhase == SEARCH_WAITING_FOR_LIGHT) {
    float score = 0.0f;
    float balance = 0.0f;
    if (currentLdrSampleUsableForSearch(score, balance)) {
      if (searchStartValidCount < SEARCH_START_VALID_SAMPLES) {
        ++searchStartValidCount;
      }
    } else {
      searchStartValidCount = 0;
    }
    if (searchStartValidCount >= SEARCH_START_VALID_SAMPLES) {
      searchStartValidCount = 0;
      beginOrientationSearch();
    }
    return;
  }

  if (searchPhase == SEARCH_COARSE_GRID ||
      searchPhase == SEARCH_REFINE_GRID) {
    if (!servosSettled()) return;

    ++searchPointSampleCount;
    float score = 0.0f;
    float balance = 0.0f;
    if (currentLdrSampleUsableForSearch(score, balance)) {
      searchPointScoreSum += score;
      searchPointBalanceSum += balance;
      ++searchPointValidCount;
    }
    if (searchPointSampleCount >= SEARCH_POINT_SAMPLES) {
      finishSearchGridPoint();
    }
    return;
  }

  if (searchPhase == SEARCH_MOVE_TO_BEST) {
    if (!servosSettled()) return;

    ++searchPointSampleCount;
    float score = 0.0f;
    float balance = 0.0f;
    if (currentLdrSampleUsableForSearch(score, balance)) {
      searchPointScoreSum += score;
      ++searchPointValidCount;
    }
    if (searchPointSampleCount >= SEARCH_CONFIRM_SAMPLES) {
      if (searchPointValidCount < SEARCH_MIN_VALID_POINT_SAMPLES) {
        failOrientationSearch(F("best point was not electrically valid"));
      } else {
        const float confirmedScore =
            searchPointScoreSum / searchPointValidCount;
        if (confirmedScore < searchBestScore * SEARCH_CONFIRM_RATIO) {
          failOrientationSearch(F("strongest point was not repeatable"));
        } else {
          lockStrongestSearchPoint(confirmedScore);
        }
      }
    }
  }
}

void updateDesiredAngles() {
  if (ORIENTATION_INDEPENDENT_OPTICAL_MODE) {
    if (searchInProgress()) return;
    if (searchPhase == SEARCH_LOCKED && opticalSearchAllowedNow()) {
      desiredPanDeg = clampInt(roundedInt(basePanDeg) + panFineTrimDeg,
                               PAN_SERVO_MIN_DEG, PAN_SERVO_MAX_DEG);
      desiredTiltDeg = clampInt(roundedInt(baseTiltDeg) + tiltFineTrimDeg,
                                TILT_SERVO_MIN_DEG, TILT_SERVO_MAX_DEG);
    } else {
      desiredPanDeg = clampInt(PAN_PARK_DEG, PAN_SERVO_MIN_DEG,
                               PAN_SERVO_MAX_DEG);
      desiredTiltDeg = clampInt(TILT_PARK_DEG, TILT_SERVO_MIN_DEG,
                                TILT_SERVO_MAX_DEG);
    }
    return;
  }

  if (!daylight) {
    desiredPanDeg = clampInt(PAN_PARK_DEG, PAN_SERVO_MIN_DEG, PAN_SERVO_MAX_DEG);
    desiredTiltDeg = clampInt(TILT_PARK_DEG, TILT_SERVO_MIN_DEG,
                              TILT_SERVO_MAX_DEG);
    return;
  }

  desiredPanDeg = clampInt(roundedInt(basePanDeg) + panFineTrimDeg,
                           PAN_SERVO_MIN_DEG, PAN_SERVO_MAX_DEG);
  desiredTiltDeg = clampInt(roundedInt(baseTiltDeg) + tiltFineTrimDeg,
                            TILT_SERVO_MIN_DEG, TILT_SERVO_MAX_DEG);
}

void updateAstronomicalTarget(bool forceUpdate) {
  const unsigned long nowMs = millis();
  if (!forceUpdate && nowMs - lastAstronomyMs < ASTRONOMY_UPDATE_MS) return;
  lastAstronomyMs = nowMs;

  // Without trusted time, optical search can still acquire a bright source.
  if (!timeTrusted) {
    daylight = false;
    daylightStateInitialized = false;
    updateDesiredAngles();
    return;
  }

  latestSun = calculateSunPosition(currentDateTime());

  const bool previousDaylight = daylight;
  if (!daylightStateInitialized) {
    daylight = latestSun.elevationDeg > 0.0f;
    daylightStateInitialized = true;
  } else if (latestSun.elevationDeg >= ENTER_DAY_ELEVATION_DEG) {
    daylight = true;
  } else if (latestSun.elevationDeg <= ENTER_NIGHT_ELEVATION_DEG) {
    daylight = false;
  }

  if (ORIENTATION_INDEPENDENT_OPTICAL_MODE) {
    if (!daylight && searchPhase != SEARCH_STOPPED) {
      searchPhase = SEARCH_WAITING_FOR_LIGHT;
      searchStartValidCount = 0;
      panFineTrimDeg = 0;
      tiltFineTrimDeg = 0;
      resetErrorPersistence();
    }
    updateDesiredAngles();
    return;
  }

  if (!daylight) {
    panFineTrimDeg = 0;
    tiltFineTrimDeg = 0;
    resetErrorPersistence();
    updateDesiredAngles();
    return;
  }

  // Azimuth becomes physically meaningless very close to zenith; hold the
  // previous pan base there instead of allowing a large numerical swing.
  if (latestSun.azimuthStable || !basePanInitialized) {
    const float azimuthFromCenter =
        wrap180(latestSun.azimuthDeg - PAN_CENTER_AZIMUTH_DEG);
    basePanDeg = PAN_SERVO_CENTER_DEG + PAN_ASTRONOMY_DIRECTION *
        PAN_SERVO_DEG_PER_AZIMUTH_DEG * azimuthFromCenter;
    basePanDeg = clampFloat(basePanDeg, PAN_SERVO_MIN_DEG, PAN_SERVO_MAX_DEG);
    basePanInitialized = true;
  }

  const float tiltFraction = clampFloat(latestSun.elevationDeg / 90.0f,
                                        0.0f, 1.0f);
  baseTiltDeg = TILT_SERVO_AT_HORIZON_DEG + tiltFraction *
      (TILT_SERVO_AT_ZENITH_DEG - TILT_SERVO_AT_HORIZON_DEG);
  baseTiltDeg = clampFloat(baseTiltDeg, TILT_SERVO_MIN_DEG,
                           TILT_SERVO_MAX_DEG);

  // Begin each new day from the calculated target, then learn a fresh trim.
  if (!previousDaylight) {
    panFineTrimDeg = 0;
    tiltFineTrimDeg = 0;
    resetErrorPersistence();
  }
  updateDesiredAngles();
}

void serviceServoMotion() {
  if (!servosReady) return;

  const unsigned long nowMs = millis();
  if (nowMs - lastServoStepMs < SERVO_STEP_INTERVAL_MS) return;
  lastServoStepMs = nowMs;

  const int motionStep = searchInProgress() ? SEARCH_SERVO_STEP_DEG : 1;
  bool moved = false;
  if (actualPanDeg < desiredPanDeg) {
    actualPanDeg += min(motionStep, desiredPanDeg - actualPanDeg);
    panServo.write(actualPanDeg);
    moved = true;
  } else if (actualPanDeg > desiredPanDeg) {
    actualPanDeg -= min(motionStep, actualPanDeg - desiredPanDeg);
    panServo.write(actualPanDeg);
    moved = true;
  }

  if (actualTiltDeg < desiredTiltDeg) {
    actualTiltDeg += min(motionStep, desiredTiltDeg - actualTiltDeg);
    tiltServo.write(actualTiltDeg);
    moved = true;
  } else if (actualTiltDeg > desiredTiltDeg) {
    actualTiltDeg -= min(motionStep, actualTiltDeg - desiredTiltDeg);
    tiltServo.write(actualTiltDeg);
    moved = true;
  }

  if (moved) lastServoMoveMs = nowMs;
}

bool servosSettled() {
  return servosReady && actualPanDeg == desiredPanDeg &&
         actualTiltDeg == desiredTiltDeg &&
         millis() - lastServoMoveMs >= SERVO_SETTLE_MS;
}

void updatePersistence(int error, int8_t &previousSign, uint8_t &count) {
  const int8_t currentSign = signOf(error);
  if (currentSign == 0) {
    previousSign = 0;
    count = 0;
  } else if (currentSign == previousSign) {
    if (count < 255) ++count;
  } else {
    previousSign = currentSign;
    count = 1;
  }
}

bool ldrDataUseful() {
  return !ldrRailFault &&
         latestLdr.average >= LDR_MIN_AVERAGE_BRIGHTNESS &&
         latestLdr.average <= LDR_SATURATED_AVERAGE_BRIGHTNESS;
}

int effectivePanFineDirection() {
  if (!ORIENTATION_INDEPENDENT_OPTICAL_MODE) return PAN_FINE_DIRECTION;

  const int zenithOffset = actualTiltDeg - TILT_SERVO_AT_ZENITH_DEG;
  if (abs(zenithOffset) <= PAN_ZENITH_DEADBAND_DEG) return 0;
  return zenithOffset > 0 ? -PAN_FINE_DIRECTION : PAN_FINE_DIRECTION;
}

void recenterOpticalBaseIfNeeded() {
  if (!ORIENTATION_INDEPENDENT_OPTICAL_MODE ||
      searchPhase != SEARCH_LOCKED) return;

  const int recenterThresholdDeg = 30;
  if (abs(panFineTrimDeg) >= recenterThresholdDeg) {
    const int oldBase = roundedInt(basePanDeg);
    const int newBase = clampInt(oldBase + panFineTrimDeg,
                                 PAN_SERVO_MIN_DEG, PAN_SERVO_MAX_DEG);
    basePanDeg = newBase;
    panFineTrimDeg -= newBase - oldBase;
  }
  if (abs(tiltFineTrimDeg) >= recenterThresholdDeg) {
    const int oldBase = roundedInt(baseTiltDeg);
    const int newBase = clampInt(oldBase + tiltFineTrimDeg,
                                 TILT_SERVO_MIN_DEG, TILT_SERVO_MAX_DEG);
    baseTiltDeg = newBase;
    tiltFineTrimDeg -= newBase - oldBase;
  }
}

void serviceFineTracking() {
  const unsigned long nowMs = millis();
  if (nowMs - lastLdrMs < LDR_UPDATE_MS) return;
  lastLdrMs = nowMs;
  readLdrs(nowMs);

  if (ORIENTATION_INDEPENDENT_OPTICAL_MODE) {
    serviceOrientationSearchOnLdrSample();
    if (searchPhase != SEARCH_LOCKED) {
      resetErrorPersistence();
      return;
    }
  }

  const bool astronomicalFineTracking =
      daylight && latestSun.elevationDeg >= MIN_FINE_TRACK_ELEVATION_DEG;
  const bool opticalFineTracking = ORIENTATION_INDEPENDENT_OPTICAL_MODE &&
      searchPhase == SEARCH_LOCKED && opticalSearchAllowedNow();
  if ((!astronomicalFineTracking && !opticalFineTracking) ||
      !ldrDataUseful() || !servosSettled()) {
    resetErrorPersistence();
    return;
  }

  int horizontalError = latestLdr.horizontalErrorPermille;
  int verticalError = latestLdr.verticalErrorPermille;
  const int panDirection = effectivePanFineDirection();

  // At the zenith, pan has almost no pointing authority. Ignore horizontal
  // imbalance there instead of letting noise drive the bottom servo away.
  if (panDirection == 0) horizontalError = 0;

  if (abs(horizontalError) <= LDR_DEADBAND_PERMILLE ||
      fabsf(latestLdr.right - latestLdr.left) < LDR_MIN_PAIR_DIFFERENCE) {
    horizontalError = 0;
  }
  if (abs(verticalError) <= LDR_DEADBAND_PERMILLE ||
      fabsf(latestLdr.top - latestLdr.bottom) < LDR_MIN_PAIR_DIFFERENCE) {
    verticalError = 0;
  }

  updatePersistence(horizontalError, previousHorizontalErrorSign,
                    horizontalErrorPersistence);
  updatePersistence(verticalError, previousVerticalErrorSign,
                    verticalErrorPersistence);

  const bool horizontalReady = horizontalErrorPersistence >=
                               LDR_REQUIRED_SAME_DIRECTION_SAMPLES;
  const bool verticalReady = verticalErrorPersistence >=
                             LDR_REQUIRED_SAME_DIRECTION_SAMPLES;
  if (!horizontalReady && !verticalReady) return;

  // A bright corner creates both a horizontal and vertical error. Update both
  // axes in the same control cycle so GPIO25 pans and GPIO26 tilts toward it.
  bool trimChanged = false;
  if (horizontalReady) {
    const int step = adaptiveLdrStepDeg(abs(horizontalError));
    const int previousTrim = panFineTrimDeg;
    panFineTrimDeg += panDirection * signOf(horizontalError) * step;
    panFineTrimDeg = clampInt(panFineTrimDeg, -PAN_FINE_TRIM_LIMIT_DEG,
                              PAN_FINE_TRIM_LIMIT_DEG);
    trimChanged = trimChanged || panFineTrimDeg != previousTrim;
    horizontalErrorPersistence = 0;
  }
  if (verticalReady) {
    const int step = adaptiveLdrStepDeg(abs(verticalError));
    const int previousTrim = tiltFineTrimDeg;
    tiltFineTrimDeg += TILT_FINE_DIRECTION * signOf(verticalError) * step;
    tiltFineTrimDeg = clampInt(tiltFineTrimDeg, -TILT_FINE_TRIM_LIMIT_DEG,
                               TILT_FINE_TRIM_LIMIT_DEG);
    trimChanged = trimChanged || tiltFineTrimDeg != previousTrim;
    verticalErrorPersistence = 0;
  }

  if (trimChanged) {
    recenterOpticalBaseIfNeeded();
    updateDesiredAngles();
  }
}

// ---------------------------------------------------------------------------
// Serial setup and diagnostics
// ---------------------------------------------------------------------------

void printTwoDigits(uint8_t value) {
  if (value < 10) Serial.print('0');
  Serial.print(value);
}

void printDateTime(const DateTimeValue &value) {
  Serial.print(value.year);
  Serial.print('-');
  printTwoDigits(value.month);
  Serial.print('-');
  printTwoDigits(value.day);
  Serial.print(' ');
  printTwoDigits(value.hour);
  Serial.print(':');
  printTwoDigits(value.minute);
  Serial.print(':');
  printTwoDigits(value.second);
}

void printSearchPhase() {
  switch (searchPhase) {
    case SEARCH_WAITING_FOR_LIGHT:
      Serial.print(F("WAIT_LDR"));
      break;
    case SEARCH_COARSE_GRID:
      Serial.print(F("COARSE"));
      break;
    case SEARCH_REFINE_GRID:
      Serial.print(F("REFINE"));
      break;
    case SEARCH_MOVE_TO_BEST:
      Serial.print(F("RETURN_BEST"));
      break;
    case SEARCH_LOCKED:
      Serial.print(F("LOCKED"));
      break;
    case SEARCH_FAILED:
      Serial.print(F("FAILED"));
      break;
    case SEARCH_STOPPED:
      Serial.print(F("STOPPED"));
      break;
  }
}

void printStatus() {
  const DateTimeValue now = currentDateTime();
  Serial.print(F("time="));
  printDateTime(now);
  Serial.print(rtcAvailable ? F(" [RTC]") : F(" [software]"));
  if (!timeTrusted) Serial.print(F(" [TIME UNTRUSTED]"));
  Serial.print(F("  sun az/el="));
  Serial.print(latestSun.azimuthDeg, 1);
  Serial.print('/');
  Serial.print(latestSun.elevationDeg, 1);
  Serial.print(F("  raw TL/TR/BL/BR="));
  Serial.print(latestRawLdr.topLeft);
  Serial.print('/');
  Serial.print(latestRawLdr.topRight);
  Serial.print('/');
  Serial.print(latestRawLdr.bottomLeft);
  Serial.print('/');
  Serial.print(latestRawLdr.bottomRight);
  Serial.print(F("  light TL/TR/BL/BR="));
  Serial.print(latestLdr.topLeft, 0);
  Serial.print('/');
  Serial.print(latestLdr.topRight, 0);
  Serial.print('/');
  Serial.print(latestLdr.bottomLeft, 0);
  Serial.print('/');
  Serial.print(latestLdr.bottomRight, 0);
  Serial.print(F("  err H/V="));
  Serial.print(latestLdr.horizontalErrorPermille);
  Serial.print('/');
  Serial.print(latestLdr.verticalErrorPermille);
  Serial.print(F("  trim="));
  Serial.print(panFineTrimDeg);
  Serial.print('/');
  Serial.print(tiltFineTrimDeg);
  Serial.print(F("  servo="));
  Serial.print(actualPanDeg);
  Serial.print('/');
  Serial.print(actualTiltDeg);
  Serial.print(F(" -> "));
  Serial.print(desiredPanDeg);
  Serial.print('/');
  Serial.print(desiredTiltDeg);
  if (ORIENTATION_INDEPENDENT_OPTICAL_MODE) {
    Serial.print(F("  search="));
    printSearchPhase();
    Serial.print(F(" point="));
    Serial.print(searchPointsVisited);
    Serial.print(F(" best="));
    Serial.print(searchBestPanDeg);
    Serial.print('/');
    Serial.print(searchBestTiltDeg);
    Serial.print('/');
    Serial.print(searchBestScore, 0);
    Serial.print(F(" panSign="));
    Serial.print(effectivePanFineDirection());
  }
  if (ldrRailFault) Serial.print(F(" [LDR FAULT]"));
  if (!servosReady) Serial.print(F(" [SERVO FAULT]"));
  if (ORIENTATION_INDEPENDENT_OPTICAL_MODE) {
    if (searchPhase == SEARCH_LOCKED) {
      Serial.println(F(" [OPTICAL LOCK]"));
    } else if (searchInProgress()) {
      Serial.println(F(" [SCANNING]"));
    } else if (searchPhase == SEARCH_FAILED) {
      Serial.println(F(" [SCAN FAILED]"));
    } else if (searchPhase == SEARCH_STOPPED) {
      Serial.println(F(" [PARKED]"));
    } else {
      Serial.println(F(" [WAITING FOR VALID LIGHT]"));
    }
  } else {
    Serial.println(daylight ? F(" [DAY]") : F(" [PARK]"));
  }
}

int parseDigits(const char *text, uint8_t count) {
  int value = 0;
  for (uint8_t index = 0; index < count; ++index) {
    if (text[index] < '0' || text[index] > '9') return -1;
    value = value * 10 + text[index] - '0';
  }
  return value;
}

bool parseSetCommand(const char *line, DateTimeValue &value) {
  // Exact format: SET YYYY-MM-DD HH:MM:SS
  if (strlen(line) != 23 || strncmp(line, "SET ", 4) != 0 ||
      line[8] != '-' || line[11] != '-' || line[14] != ' ' ||
      line[17] != ':' || line[20] != ':') return false;

  value.year = parseDigits(line + 4, 4);
  value.month = parseDigits(line + 9, 2);
  value.day = parseDigits(line + 12, 2);
  value.hour = parseDigits(line + 15, 2);
  value.minute = parseDigits(line + 18, 2);
  value.second = parseDigits(line + 21, 2);
  return validDateTime(value);
}

void processSerialLine() {
  if (strcmp(serialLine, "STATUS") == 0) {
    printStatus();
    return;
  }
  if (strcmp(serialLine, "HELP") == 0) {
    Serial.println(F("Commands: STATUS | SCAN | PARK | SET YYYY-MM-DD HH:MM:SS (UTC) | HELP"));
    return;
  }
  if (strcmp(serialLine, "SCAN") == 0) {
    searchPhase = SEARCH_WAITING_FOR_LIGHT;
    searchStartValidCount = 0;
    panFineTrimDeg = 0;
    tiltFineTrimDeg = 0;
    resetErrorPersistence();
    updateDesiredAngles();
    Serial.println(F("Scan armed; waiting for ten valid LDR samples."));
    return;
  }
  if (strcmp(serialLine, "PARK") == 0 ||
      strcmp(serialLine, "STOP") == 0) {
    searchPhase = SEARCH_STOPPED;
    panFineTrimDeg = 0;
    tiltFineTrimDeg = 0;
    resetErrorPersistence();
    updateDesiredAngles();
    Serial.println(F("Optical scan stopped; tracker parking."));
    return;
  }

  DateTimeValue requestedTime;
  if (parseSetCommand(serialLine, requestedTime)) {
    setSoftwareClock(requestedTime);
    timeTrusted = true;
    daylightStateInitialized = false;
    if (writeRtc(requestedTime)) {
      rtcAvailable = true;
      Serial.println(F("Clock set; DS3231 detected and updated."));
    } else {
      rtcAvailable = false;
      Serial.println(F("Software clock set; DS3231 was not detected."));
    }
    updateAstronomicalTarget(true);
    return;
  }

  Serial.println(F("Unknown command. Type HELP."));
}

void serviceSerial() {
  while (Serial.available() > 0) {
    const char character = Serial.read();
    if (character == '\r') continue;
    if (character == '\n') {
      serialLine[serialLineLength] = '\0';
      if (serialLineLength > 0) processSerialLine();
      serialLineLength = 0;
    } else if (serialLineLength < sizeof(serialLine) - 1) {
      serialLine[serialLineLength++] = character;
    }
  }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  analogReadResolution(12);
  analogSetPinAttenuation(LDR_TOP_LEFT_PIN, ADC_11db);
  analogSetPinAttenuation(LDR_TOP_RIGHT_PIN, ADC_11db);
  analogSetPinAttenuation(LDR_BOTTOM_LEFT_PIN, ADC_11db);
  analogSetPinAttenuation(LDR_BOTTOM_RIGHT_PIN, ADC_11db);

  actualPanDeg = clampInt(PAN_PARK_DEG, PAN_SERVO_MIN_DEG, PAN_SERVO_MAX_DEG);
  actualTiltDeg = clampInt(TILT_PARK_DEG, TILT_SERVO_MIN_DEG,
                           TILT_SERVO_MAX_DEG);
  desiredPanDeg = actualPanDeg;
  desiredTiltDeg = actualTiltDeg;

  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);
  panServo.attach(PAN_SERVO_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  tiltServo.attach(TILT_SERVO_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  servosReady = panServo.attached() && tiltServo.attached();
  if (servosReady) {
    panServo.write(actualPanDeg);
    tiltServo.write(actualTiltDeg);
  } else {
    Serial.println(F("ERROR: One or both servo PWM channels failed to attach."));
  }
  lastServoMoveMs = millis();

  DateTimeValue initialTime = buildTime();
  setSoftwareClock(initialTime);

  DateTimeValue rtcTime;
  if (readRtc(rtcTime) && rtcOscillatorIsRunning()) {
    rtcAvailable = true;
    timeTrusted = true;
    setSoftwareClock(rtcTime);
    Serial.println(F("DS3231 clock detected."));
  } else {
    Serial.println(F("WARNING: No valid DS3231 time. Optical scan remains available."));
    Serial.println(F("Set UTC with: SET YYYY-MM-DD HH:MM:SS"));
  }

  Serial.println(F("Dual-axis tracker ready; automatic optical scan is armed."));
  Serial.println(F("Type HELP for STATUS, SCAN, PARK, and clock commands."));
  readLdrs(millis());
  updateAstronomicalTarget(true);
  printStatus();
}

void loop() {
  serviceSerial();
  updateAstronomicalTarget(false);
  serviceFineTracking();
  serviceServoMotion();

  const unsigned long nowMs = millis();
  if (nowMs - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = nowMs;
    printStatus();
  }
}
