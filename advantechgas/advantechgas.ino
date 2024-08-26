//#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <DHT.h>
#include <DHT_U.h>
//#include <ArduinoModbus.h>
#include <ModbusIP_ESP8266.h>

ModbusIP mb;
DHT_Unified dht(D4, DHT11);

uint32_t lastMillis = 0;
const int mqPin = A0; // Pin analog untuk sensor MQ136
int mqValue = 0;
float temperature = 0.0f;
float humidity = 0.0f;

void setup() {
  Serial.begin(115200);
  dht.begin();

  // Menghubungkan ke jaringan Wi-Fi
  WiFi.begin("PANGGABEAN", "panggabean123");
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

  // Update nilai register Modbus
  mb.Hreg(0, mqValue);
  mb.Hreg(1, temperature * 10);
  mb.Hreg(2, humidity * 10);

  Serial.print("Nilai MQ136: ");
  Serial.println(mqValue);
  Serial.print("Suhu: ");
  Serial.println(temperature);
  Serial.print("Kelembaban: ");
  Serial.println(humidity);

  mb.task();
  delay(1000); // Tunggu 1 detik sebelum membaca lagi
}
