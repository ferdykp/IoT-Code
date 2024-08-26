#include <Wire.h>
#include <RTClib.h>

// Pin untuk Sensor PIR
const int pirPin = 33;

// Pin untuk Relay
const int relayPin = 16;

int statusPin = 0;

RTC_DS3231 rtc;

void setup() {
  Serial.begin(115200);
  pinMode(pirPin, INPUT);
  pinMode(relayPin, OUTPUT);

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

void loop() {
  DateTime now = rtc.now();

 // Baca nilai sensor PIR
//  statusPin = digitalRead(pirPin);
  // Jika sensor mendeteksi gerakan
  if (statusPin == HIGH || (now.hour() == 22 && now.minute() == 13)) {

  // Baca nilai sensor PIR
  statusPin = digitalRead(pirPin);
    
    Serial.println("Gerakan terdeteksi! || PIR ON");
    digitalWrite(relayPin, LOW);
    delay(5000);
  }

  else if (now.hour() == 22 && now.minute() == 14) {
    digitalWrite(relayPin, HIGH);

    Serial.println("Tidak ada gerakan || PIR OFF");
  }

//  // Timer untuk on/off relay pada waktu tertentu (misalnya, hidupkan relay pada 08:00, matikan pada 20:00)
//  if (now.hour() == 20 && now.minute() == 16) {
//    digitalWrite(relayPin, LOW); // Hidupkan relay
//  }
//  else if (now.hour() == 20 && now.minute() == 17) {
//    digitalWrite(relayPin, HIGH); // Matikan relay
//  }
// Tampilkan waktu RTC saat ini di Serial Monitor
  Serial.print("Waktu RTC saat ini: ");
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();

  delay(500); // Tunda untuk menghindari pembacaan berulang yang cepat
}
