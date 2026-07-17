#include <Arduino.h>
#include <ESP32Servo.h>

// ---------------- Pins ----------------

const int PAN_SERVO_PIN = 25;   // Bottom, horizontal servo
const int TILT_SERVO_PIN = 26;  // Top, vertical servo

// LDR positions when looking at the front of the panel.
const int LDR_TOP_LEFT_PIN = 34;
const int LDR_TOP_RIGHT_PIN = 35;
const int LDR_BOTTOM_LEFT_PIN = 39;
const int LDR_BOTTOM_RIGHT_PIN = 36;

// 1 means increasing the angle moves toward right/top.
// Change an axis to -1 if that servo moves the wrong way.
const int PAN_DIRECTION = 1;
const int TILT_DIRECTION = 1;

// ---------------- Settings ----------------

const int SERVO_MIN_ANGLE = 10;
const int SERVO_MAX_ANGLE = 170;
const int LIGHT_DEADBAND = 80;
const unsigned long UPDATE_INTERVAL_MS = 20;
const unsigned long PRINT_INTERVAL_MS = 100;

Servo panServo;
Servo tiltServo;

int panAngle = 90;
int tiltAngle = 90;

unsigned long lastUpdateMs = 0;
unsigned long lastPrintMs = 0;

int topLeft = 0;
int topRight = 0;
int bottomLeft = 0;
int bottomRight = 0;
int horizontalError = 0;
int verticalError = 0;

int readLdr(int pin) {
  // Discard the first conversion after changing ADC channels.
  analogRead(pin);

  long total = 0;
  for (int sample = 0; sample < 4; sample++) {
    total += analogRead(pin);
  }
  return total / 4;
}

int movementStep(int errorSize) {
  if (errorSize > 800) return 6;  // Large correction
  if (errorSize > 300) return 3;  // Medium correction
  return 1;                       // Fine correction
}

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetPinAttenuation(LDR_TOP_LEFT_PIN, ADC_11db);
  analogSetPinAttenuation(LDR_TOP_RIGHT_PIN, ADC_11db);
  analogSetPinAttenuation(LDR_BOTTOM_LEFT_PIN, ADC_11db);
  analogSetPinAttenuation(LDR_BOTTOM_RIGHT_PIN, ADC_11db);

  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);
  panServo.attach(PAN_SERVO_PIN, 1000, 2000);
  tiltServo.attach(TILT_SERVO_PIN, 1000, 2000);

  panServo.write(panAngle);
  tiltServo.write(tiltAngle);

  Serial.println("Simple four-LDR solar tracker started.");
  Serial.println("Higher ADC value is treated as brighter.");
}

void loop() {
  const unsigned long now = millis();

  if (now - lastUpdateMs >= UPDATE_INTERVAL_MS) {
    lastUpdateMs = now;

    topLeft = readLdr(LDR_TOP_LEFT_PIN);
    topRight = readLdr(LDR_TOP_RIGHT_PIN);
    bottomLeft = readLdr(LDR_BOTTOM_LEFT_PIN);
    bottomRight = readLdr(LDR_BOTTOM_RIGHT_PIN);

    const int leftLight = (topLeft + bottomLeft) / 2;
    const int rightLight = (topRight + bottomRight) / 2;
    const int topLight = (topLeft + topRight) / 2;
    const int bottomLight = (bottomLeft + bottomRight) / 2;

    // Positive horizontal error means the right side is brighter.
    // Positive vertical error means the top side is brighter.
    horizontalError = rightLight - leftLight;
    verticalError = topLight - bottomLight;

    if (abs(horizontalError) > LIGHT_DEADBAND) {
      const int step = movementStep(abs(horizontalError));
      panAngle += (horizontalError > 0 ? 1 : -1) * PAN_DIRECTION * step;
      panAngle = constrain(panAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
      panServo.write(panAngle);
    }

    if (abs(verticalError) > LIGHT_DEADBAND) {
      const int step = movementStep(abs(verticalError));
      tiltAngle += (verticalError > 0 ? 1 : -1) * TILT_DIRECTION * step;
      tiltAngle = constrain(tiltAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
      tiltServo.write(tiltAngle);
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
    Serial.print("  servo pan/tilt=");
    Serial.print(panAngle);
    Serial.print('/');
    Serial.println(tiltAngle);
  }
  delay(100);
}
