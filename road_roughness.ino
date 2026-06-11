#include <Wire.h>
#include <TinyGPS++.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
TinyGPSPlus gps;

HardwareSerial gpsSerial(2);

#define RXD2 16
#define TXD2 17

const int MPU = 0x68;

// Raw accel values
int16_t AcX, AcY, AcZ;

// Previous values
int16_t prevX = 0;
int16_t prevY = 0;
int16_t prevZ = 0;

// Current vibration
float vibrationDelta = 0;

// Baseline vibration
float baseline = 0;

// Effective vibration after calibration
float effectiveDelta = 0;

// Thresholds ABOVE baseline
const float GREEN_THRESHOLD = 7000;
const float YELLOW_THRESHOLD = 10000;

String roadClass = "GREEN";

// Calibration
bool calibrated = false;
unsigned long calibrationStart = 0;

float calibrationSum = 0;
int calibrationSamples = 0;

void setup() {

  Serial.begin(115200);

  SerialBT.begin("RoadSense_ESP32");

  Wire.begin(21, 22);

  gpsSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // Wake MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.println("SYSTEM STARTED");
  Serial.println("CALIBRATING FOR 5 SECONDS...");

  SerialBT.println("SYSTEM STARTED");
  SerialBT.println("CALIBRATING FOR 5 SECONDS...");

  calibrationStart = millis();
}

void loop() {

  // ---------------- GPS ----------------

  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // ---------------- MPU6050 ----------------

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU, 6, true);

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  // ---------------- DELTA CALCULATION ----------------

  float dx = abs(AcX - prevX);
  float dy = abs(AcY - prevY);
  float dz = abs(AcZ - prevZ);

  vibrationDelta = sqrt(dx * dx + dy * dy + dz * dz);

  prevX = AcX;
  prevY = AcY;
  prevZ = AcZ;

  // ---------------- CALIBRATION PHASE ----------------

  if (!calibrated) {

    calibrationSum += vibrationDelta;
    calibrationSamples++;

    // 5-second calibration
    if (millis() - calibrationStart >= 5000) {

      baseline = calibrationSum / calibrationSamples;

      calibrated = true;

      Serial.println("CALIBRATION COMPLETE");
      Serial.print("BASELINE: ");
      Serial.println(baseline);

      SerialBT.println("CALIBRATION COMPLETE");
      SerialBT.print("BASELINE: ");
      SerialBT.println(baseline);
    }

    delay(50);
    return;
  }

  // ---------------- EFFECTIVE DELTA ----------------

  effectiveDelta = vibrationDelta - baseline;

  if (effectiveDelta < 0)
    effectiveDelta = 0;

  // ---------------- ROAD CLASSIFICATION ----------------

  if (effectiveDelta < GREEN_THRESHOLD) {

    roadClass = "GREEN";

  } else if (effectiveDelta < YELLOW_THRESHOLD) {

    roadClass = "YELLOW";

  } else {

    roadClass = "RED";
  }

  // ---------------- GPS DATA ----------------

  double lat = gps.location.lat();
  double lon = gps.location.lng();
  double speedKmph = gps.speed.kmph();

  // ---------------- TIMESTAMP ----------------

  unsigned long t = millis();

  // ---------------- CSV OUTPUT ----------------

  String dataLine = "";

  dataLine += String(t);
  dataLine += ",";

  dataLine += String(lat, 6);
  dataLine += ",";

  dataLine += String(lon, 6);
  dataLine += ",";

  dataLine += String(speedKmph, 2);
  dataLine += ",";

  dataLine += String(effectiveDelta, 2);
  dataLine += ",";

  dataLine += roadClass;

  // Serial Monitor
  Serial.println(dataLine);

  // Bluetooth
  SerialBT.println(dataLine);

  delay(100);
}