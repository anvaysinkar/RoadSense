#include <Wire.h>
#include <TinyGPS++.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
TinyGPSPlus gps;

HardwareSerial gpsSerial(2);

#define RXD2 16
#define TXD2 17

const int MPU = 0x68;

// Accelerometer values
int16_t AcX, AcY, AcZ;

// Previous values
int16_t prevX = 0;
int16_t prevY = 0;
int16_t prevZ = 0;

// Delta variables
float vibrationDelta = 0;
float effectiveDelta = 0;

// Calibration
float baseline = 0;
float calibrationSum = 0;
int calibrationSamples = 0;

bool calibrated = false;
unsigned long calibrationStart;

// POTHOLE PARAMETERS
const float POTHOLE_THRESHOLD = 7000;

// To avoid multiple detections for same pothole
unsigned long lastPotholeTime = 0;
const int potholeCooldown = 1500; // milliseconds

// Counter
int potholeCount = 0;

void setup() {

  Serial.begin(115200);

  SerialBT.begin("PotholeSense_ESP32");

  Wire.begin(21, 22);

  gpsSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // Wake MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.println("==================================");
  Serial.println("POTHOLE DETECTION SYSTEM STARTED");
  Serial.println("CALIBRATING FOR 5 SECONDS...");
  Serial.println("Keep vehicle stationary or");
  Serial.println("drive on smooth road.");
  Serial.println("==================================");

  SerialBT.println("SYSTEM STARTED");

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

  // ---------------- CALIBRATION ----------------

  if (!calibrated) {

    calibrationSum += vibrationDelta;
    calibrationSamples++;

    Serial.print("Calibrating... ");
    Serial.println(vibrationDelta);

    if (millis() - calibrationStart >= 5000) {

      baseline = calibrationSum / calibrationSamples;

      calibrated = true;

      Serial.println("\n==================================");
      Serial.println("CALIBRATION COMPLETE");
      Serial.print("Baseline: ");
      Serial.println(baseline);

      Serial.print("POTHOLE THRESHOLD: ");
      Serial.println(baseline + POTHOLE_THRESHOLD);

      Serial.println("==================================");

      SerialBT.println("CALIBRATION COMPLETE");
    }

    delay(50);
    return;
  }

  // ---------------- EFFECTIVE DELTA ----------------

  effectiveDelta = vibrationDelta - baseline;

  if (effectiveDelta < 0)
    effectiveDelta = 0;

  // ---------------- GPS DATA ----------------

  double lat = gps.location.lat();
  double lon = gps.location.lng();
  double speedKmph = gps.speed.kmph();

  // ---------------- POTHOLE DETECTION ----------------

  bool potholeDetected = false;

  if (
      effectiveDelta > POTHOLE_THRESHOLD &&
      millis() - lastPotholeTime > potholeCooldown
     )
  {

    potholeDetected = true;

    potholeCount++;

    lastPotholeTime = millis();

    // ---------------- POTHOLE EVENT OUTPUT ----------------

    String potholeEvent = "";

    potholeEvent += "POTHOLE DETECTED,";

    potholeEvent += "Count:";
    potholeEvent += String(potholeCount);
    potholeEvent += ",";

    potholeEvent += "Lat:";
    potholeEvent += String(lat, 6);
    potholeEvent += ",";

    potholeEvent += "Lon:";
    potholeEvent += String(lon, 6);
    potholeEvent += ",";

    potholeEvent += "Speed:";
    potholeEvent += String(speedKmph, 2);
    potholeEvent += ",";

    potholeEvent += "Delta:";
    potholeEvent += String(effectiveDelta, 2);

    Serial.println("\n==============================");
    Serial.println(potholeEvent);
    Serial.println("==============================");

    SerialBT.println(potholeEvent);
  }

  // ---------------- LIVE STREAM DATA ----------------

  String liveData = "";

  liveData += String(millis());
  liveData += ",";

  liveData += String(lat, 6);
  liveData += ",";

  liveData += String(lon, 6);
  liveData += ",";

  liveData += String(speedKmph, 2);
  liveData += ",";

  liveData += String(effectiveDelta, 2);
  liveData += ",";

  if (potholeDetected)
    liveData += "POTHOLE";
  else
    liveData += "NORMAL";

  // Serial Monitor
  Serial.println(liveData);

  // Bluetooth
  SerialBT.println(liveData);

  delay(100);
}