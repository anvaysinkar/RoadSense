#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Create GPS object
TinyGPSPlus gps;

// Use UART2 on ESP32
HardwareSerial gpsSerial(2);

// GPS pins
#define RXD2 16   // ESP32 RX <- GPS TX
#define TXD2 17   // ESP32 TX -> GPS RX

void setup() {

  // Serial Monitor
  Serial.begin(115200);

  // GPS Serial
  gpsSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);

  Serial.println("GPS TEST STARTED");
}

void loop() {

  // Read GPS data continuously
  while (gpsSerial.available() > 0) {

    char c = gpsSerial.read();

    // Feed character to TinyGPS++
    gps.encode(c);
  }

  // Print every 2 seconds
  static unsigned long lastPrint = 0;

  if (millis() - lastPrint > 2000) {

    lastPrint = millis();

    Serial.println("\n========================");

    // ---------------- LOCATION ----------------

    if (gps.location.isValid()) {

      Serial.print("Latitude  : ");
      Serial.println(gps.location.lat(), 6);

      Serial.print("Longitude : ");
      Serial.println(gps.location.lng(), 6);

    } else {

      Serial.println("Waiting for GPS location...");
    }

    // ---------------- SATELLITES ----------------

    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());

    // ---------------- SPEED ----------------

    Serial.print("Speed (km/h): ");
    Serial.println(gps.speed.kmph());

    // ---------------- DATE & TIME ----------------

    if (gps.date.isValid() && gps.time.isValid()) {

      int hour = gps.time.hour();
      int minute = gps.time.minute();
      int second = gps.time.second();

      // Convert UTC to IST (+5:30)
      minute += 30;
      hour += 5;

      if (minute >= 60) {
        minute -= 60;
        hour++;
      }

      if (hour >= 24) {
        hour -= 24;
      }

      // DATE
      Serial.print("Date: ");

      if (gps.date.day() < 10) Serial.print("0");
      Serial.print(gps.date.day());

      Serial.print("/");

      if (gps.date.month() < 10) Serial.print("0");
      Serial.print(gps.date.month());

      Serial.print("/");

      Serial.println(gps.date.year());

      // TIME
      Serial.print("Time (IST): ");

      if (hour < 10) Serial.print("0");
      Serial.print(hour);

      Serial.print(":");

      if (minute < 10) Serial.print("0");
      Serial.print(minute);

      Serial.print(":");

      if (second < 10) Serial.print("0");
      Serial.println(second);

    } else {

      Serial.println("Waiting for valid date/time...");
    }

    Serial.println("========================");
  }
}