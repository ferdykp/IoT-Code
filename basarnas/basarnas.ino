#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <RTClib.h>
#include <TinyGPS++.h>
#include <NewPing.h>

// Pin untuk sensor ultrasonik JSN-SR04T pada ESP32
#define TRIGGER_PIN  27
#define ECHO_PIN     26
#define MAX_DISTANCE 450

// Pin untuk waterflow sensor
#define FLOW_SENSOR_PIN 25

// Pin untuk GPS Neo 7M pada ESP32
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

// Inisialisasi objek sensor ultrasonik
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Inisialisasi objek RTC
RTC_DS3231 rtc;

// Inisialisasi objek GPS
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);

// Inisialisasi objek waterflow sensor
volatile int flow_frequency; // variabel untuk menghitung frekuensi aliran
unsigned int flow_meter_pin = FLOW_SENSOR_PIN; // pin untuk waterflow sensor
unsigned long totalmillis; // waktu total
unsigned long lastmillis; // waktu terakhir

void flow() {
  flow_frequency++;
}

void setup() {
  Serial.begin(115200);
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);  // Konfigurasi pin RX dan TX untuk GPS
  Wire.begin();

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  pinMode(flow_meter_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(flow_meter_pin), flow, RISING);
}

void loop() {
  // Membaca data GPS
  while (GPSSerial.available() > 0) {
    if (gps.encode(GPSSerial.read())) {
      // Ambil data GPS, seperti latitude, longitude, dan waktu
      Serial.print("Latitude: ");
      Serial.print(gps.location.lat(), 6);
      Serial.print("Longitude ");
      Serial.print(gps.location.lng(), 6);
      Serial.print(" Altitude: ");
      Serial.print(gps.altitude.meters());
      Serial.print(" meters  Speed: ");
      Serial.print(gps.speed.kmph());
      Serial.print(" km/h  Course: ");
      Serial.print(gps.course.deg());
      Serial.print(" degrees  Time: ");
      Serial.print(gps.time.hour());
      Serial.print(":");
      Serial.print(gps.time.minute());
      Serial.print(":");
      Serial.print(gps.time.second());
      Serial.println();
    }
  }

  // Mengukur jarak menggunakan sensor ultrasonik
  delay(1000);
  unsigned int distance = sonar.ping_cm();

  // Membaca tanggal dan waktu dari RTC
  DateTime now = rtc.now();
  Serial.print("RTC Time: ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();

  // Mengukur kecepatan arus air dengan waterflow sensor
  totalmillis = millis();
  if ((totalmillis - lastmillis) > 1000) {
    detachInterrupt(flow_meter_pin);
    float flowrate = (flow_frequency / 7.5);  // menghitung laju aliran dalam liter per detik
    Serial.print("Flowrate: ");
    Serial.print(flowrate);
    Serial.println(" L/s");
    flow_frequency = 0;
    lastmillis = millis();
    attachInterrupt(digitalPinToInterrupt(flow_meter_pin), flow, RISING);
  }

  // Menampilkan hasil pengukuran kedalaman
  Serial.print("Depth: ");
  Serial.print(distance);
  Serial.println(" cm");

//  Serial.print("Latitude: ");
//  Serial.println(gps.location.lat(), 6);
//  Serial.print("Longitude ");
//  Serial.println(gps.location.lng(), 6);
}
