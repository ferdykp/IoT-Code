//#include <Arduino.h>
#include <Wire.h>
#include <DHT.h>
#include <DHT_U.h>
#include <MPU6050.h>
//#include <ArduinoModbus.h>
#include <ModbusIP_ESP8266.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

ModbusIP mb;
DHT_Unified dht(D1, DHT11);
MPU6050 mpu;

uint32_t lastMillis = 0;
const int mqPin = A0; // Pin analog untuk sensor MQ136
int mqValue = 0;
float temperature = 0.0f;
float humidity = 0.0f;
int16_t ax, ay, az, gx, gy, gz;
static const int RXPin = 12, TXPin = 13; //RX = PIN D6 dan TX = PIN D7

TinyGPSPlus gps;

SoftwareSerial gpsSerial(RXPin, TXPin);


void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600); // Inisialisasi komunikasi serial dengan GPS

  dht.begin();
  Wire.begin();
  mpu.initialize();

  // Menghubungkan ke jaringan Wi-Fi
  WiFi.begin("?", "ferdykurnia");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  mb.server();
  mb.addHreg(0, mqValue); // Register Modbus untuk nilai sensor MQ136
  mb.addHreg(1, temperature * 10); // Register Modbus untuk suhu (* 10 untuk menyimpan desimal)
  mb.addHreg(2, humidity * 10); // Register Modbus untuk kelembaban (* 10 untuk menyimpan desimal)
  mb.addHreg(3, ax); // Register Modbus untuk akselerasi x
  mb.addHreg(4, ay); // Register Modbus untuk akselerasi y
  mb.addHreg(5, az); // Register Modbus untuk akselerasi z
  mb.addHreg(6, gx); // Register Modbus untuk kecepatan sudut x
  mb.addHreg(7, gy); // Register Modbus untuk kecepatan sudut y
  mb.addHreg(8, gz); // Register Modbus untuk kecepatan sudut z
  mb.addHreg(9, 0); // Register Modbus untuk longitude (belum diisi)
  mb.addHreg(10, 0); // Register Modbus untuk latitude (belum diisi)
}

void loop() {
  // Membaca nilai analog dari sensor MQ136
  mqValue = analogRead(mqPin);

  // Baca data suhu dan kelembaban dari sensor DHT11
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error membaca suhu!"));
  } else {
    temperature = event.temperature;
  }

  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error membaca kelembaban!"));
  } else {
    humidity = event.relative_humidity;
  }

  // Baca data MPU6050
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Baca data GPS dari modul GPS Neo-6M
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        mb.Hreg(9, gps.location.lng() * 10000); // Register Modbus untuk longitude (* 10000 untuk menyimpan desimal)
        mb.Hreg(10, gps.location.lat() * 10000); // Register Modbus untuk latitude (* 10000 untuk menyimpan desimal)
      }
    }
  }

  // Update nilai register Modbus
  mb.Hreg(0, mqValue);
  mb.Hreg(1, temperature * 10);
  mb.Hreg(2, humidity * 10);
  mb.Hreg(3, ax);
  mb.Hreg(4, ay);
  mb.Hreg(5, az);
  mb.Hreg(6, gx);
  mb.Hreg(7, gy);
  mb.Hreg(8, gz);

  Serial.print("Nilai MQ136: ");
  Serial.println(mqValue);
  Serial.print("Suhu: ");
  Serial.println(temperature);
  Serial.print("Kelembaban: ");
  Serial.println(humidity);
  Serial.print("Akselerasi x: ");
  Serial.println(ax);
  Serial.print("Akselerasi y: ");
  Serial.println(ay);
  Serial.print("Akselerasi z: ");
  Serial.println(az);
  Serial.print("Kecepatan sudut x: ");
  Serial.println(gx);
  Serial.print("Kecepatan sudut y: ");
  Serial.println(gy);
  Serial.print("Kecepatan sudut z: ");
  Serial.println(gz);
  Serial.print("Longitude: ");
  Serial.println(gps.location.lng());
  Serial.print("Latitude: ");
  Serial.println(gps.location.lat());

  mb.task();
  delay(1000); // Tunggu 1 detik sebelum membaca lagi
}
