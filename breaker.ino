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

// Previous accel values
int16_t prevX = 0;
int16_t prevY = 0;
int16_t prevZ = 0;

// Delta values
float vibrationDelta = 0;
float effectiveDelta = 0;

// Calibration
float baseline = 0;
float calibrationSum = 0;
int calibrationSamples = 0;

bool calibrated = false;
unsigned long calibrationStart;

// SPEED BREAKER PARAMETERS

// Lower threshold than pothole
const float BREAKER_THRESHOLD = 3500;

// Duration window
unsigned long breakerStartTime = 0;
bool breakerInProgress = false;

// Detection timing
const int MIN_BREAKER_DURATION = 300;
const int MAX_BREAKER_DURATION = 2000;

// Speed reduction
double previousSpeed = 0;
double speedDrop = 0;

// Cooldown
unsigned long lastBreakerTime = 0;
const int breakerCooldown = 3000;

// Counter
int breakerCount = 0;

void setup() {

  Serial.begin(115200);

  SerialBT.begin("SpeedBreakerSense_ESP32");

  Wire.begin(21, 22);

  gpsSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // Wake MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.println("==================================");
  Serial.println("SPEED BREAKER DETECTION SYSTEM");
  Serial.println("CALIBRATING FOR 5 SECONDS...");
  Serial.println("Keep vehicle stationary or");
  Serial.println("drive on smooth road.");
  Serial.println("==================================");

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

      Serial.print("BREAKER THRESHOLD: ");
      Serial.println(baseline + BREAKER_THRESHOLD);

      Serial.println("==================================");
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

  // ---------------- SPEED DROP ----------------

  speedDrop = previousSpeed - speedKmph;

  previousSpeed = speedKmph;

  // ---------------- SPEED BREAKER LOGIC ----------------

  bool breakerDetected = false;

  // Start breaker event
  if (
      effectiveDelta > BREAKER_THRESHOLD &&
      !breakerInProgress &&
      millis() - lastBreakerTime > breakerCooldown
     )
  {

    breakerInProgress = true;

    breakerStartTime = millis();
  }

  // End breaker event
  if (
      breakerInProgress &&
      effectiveDelta < BREAKER_THRESHOLD
     )
  {

    unsigned long breakerDuration =
      millis() - breakerStartTime;

    // Valid speed breaker conditions
    if (
        breakerDuration > MIN_BREAKER_DURATION &&
        breakerDuration < MAX_BREAKER_DURATION &&
        speedDrop > 1.0
       )
    {

      breakerDetected = true;

      breakerCount++;

      lastBreakerTime = millis();

      // ---------------- BREAKER EVENT ----------------

      String breakerEvent = "";

      breakerEvent += "SPEED_BREAKER,";

      breakerEvent += "Count:";
      breakerEvent += String(breakerCount);
      breakerEvent += ",";

      breakerEvent += "Lat:";
      breakerEvent += String(lat, 6);
      breakerEvent += ",";

      breakerEvent += "Lon:";
      breakerEvent += String(lon, 6);
      breakerEvent += ",";

      breakerEvent += "Speed:";
      breakerEvent += String(speedKmph, 2);
      breakerEvent += ",";

      breakerEvent += "Delta:";
      breakerEvent += String(effectiveDelta, 2);
      breakerEvent += ",";

      breakerEvent += "Duration:";
      breakerEvent += String(breakerDuration);

      Serial.println("\n==============================");
      Serial.println(breakerEvent);
      Serial.println("==============================");

      SerialBT.println(breakerEvent);
    }

    breakerInProgress = false;
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

  if (breakerDetected)
    liveData += "BREAKER";
  else
    liveData += "NORMAL";

  // Serial Monitor
  Serial.println(liveData);

  // Bluetooth
  SerialBT.println(liveData);

  delay(100);
}