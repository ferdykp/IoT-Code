#include <SoftwareSerial.h>
#include <TinyGPS++.h>

static const int RXPin = 12, TXPin = 13;
static const uint32_t GPSBaud = 9600;

SoftwareSerial gpsSerial(RXPin, TXPin);
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPSBaud);
}

void loop() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        Serial.print("Latitude: ");
        Serial.println(gps.location.lat(), 6);
        Serial.print("Longitude: ");
        Serial.println(gps.location.lng(), 6);
        Serial.print("Altitude (meters): ");
        Serial.println(gps.altitude.meters());
      }
    }
  }
}
