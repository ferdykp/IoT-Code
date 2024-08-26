#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>
#define buzz 23

#define ssid "XLGO-PKM"
#define pass "twinguard"
#define ip String("twinguard.my.id")
#define lamaGerak 10000
#define sampleTime 100
#define gerakanX 1
#define gerakanY 1
#define gerakanZ 1

bool bergerak = false, buzzState = false;
bool alarmGerak = false;
bool blockAlarm = false;
float x, y, z, lastX, lastY, lastZ;
float longitude[2], latitude[2];
unsigned long timingSample = millis(), timingGerak = millis(), timingJalan = millis(), timingGerak2 = millis(), lstBuzz, lstWifi;
TinyGPSPlus gps;
unsigned long lst = millis();

Adafruit_MPU6050 mpu;
const String number = "+6282257932326";
TaskHandle_t Task1;

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 5, 18);
  Serial2.begin(9600);
  pinMode(buzz, OUTPUT);
  pinMode(2, OUTPUT);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    delay(3000);
    ESP.restart();
  }
  WiFi.begin(ssid, pass);
  Serial.println("Connecting To Wifi");
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(2, !digitalRead(2));
    delay(100);
    if (millis() - lstWifi >= 10000)ESP.restart();
  }
  Serial.println("Wifi Connected");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  xTaskCreatePinnedToCore(
    Task1code, /* Function to implement the task */
    "Task1", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    0,  /* Priority of the task */
    &Task1,  /* Task handle. */
    0); /* Core where the task should run */
}

void loop() {
  if (millis() - timingSample >= sampleTime) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    lst = millis();
    x = a.acceleration.x;
    y = a.acceleration.y;
    z = a.acceleration.z;
    if (abs(abs(x) - abs(lastX)) > gerakanX || abs(abs(y) - abs(lastY)) > gerakanY || abs(abs(z) - abs(lastZ)) > gerakanZ) bergerak = true;
    else bergerak = false;
    lastX = x; lastY = y; lastZ = z;

    if (bergerak) {
      timingGerak = millis();
      if (!alarmGerak) {
        if (gps.location.isValid()) {
          Serial.println("Set First Point");
          longitude[0] = (float) gps.location.lat();
          latitude[0] = (float) gps.location.lng();
          longitude[1] = (float) longitude[0];
          latitude[1] = (float) latitude[0];
          timingJalan = millis();
          timingGerak2 = millis();
        }
      }
      alarmGerak = true;
    }
    else {
      if (millis() - timingGerak >= lamaGerak - 1000) {
        if (alarmGerak) Serial.println("Gerak Dinonaktifkan");
        longitude[0] = 0;
        latitude[0] = 0;
        longitude[1] = 0;
        latitude[1] = 0;
        alarmGerak = false;
        timingJalan = millis();
        buzzState = false;
      }
    }
  }
  if (buzzState && !blockAlarm) {
    if (millis() - lstBuzz >= 200) {
      lstBuzz = millis();
      digitalWrite(buzz, !digitalRead(buzz));
    }
  }
  else {
    digitalWrite(buzz, LOW);
  }

  if (Serial.available()) {
    String str = "";
    while (1) {
      if (Serial.available()) {
        char dt = Serial.read();
        if (dt == 10 || dt == 13)break;
        str += (char)dt;
      }
    }
    Serial.println(str);
    if (str == "gps") {
      Serial.println(getLocation());
    }
    else if (str == "exit") {
      Serial2.write(26);
    }
    else if (str == "at") {
      Serial2.print(str + "\r");
    }
    else {
    }
  }

  if (Serial2.available()) {
    char dt = Serial2.read();
    Serial.write(dt);
  }

  while (Serial1.available()) {
    gps.encode(Serial1.read());

    if (gps.location.isValid()) {
      longitude[0] = (float) gps.location.lat();
      latitude[0] = (float) gps.location.lng();
    }
  }
}


void Task1code( void * parameter) {
  for (;;) {
    if (millis() - timingGerak2 >= lamaGerak && alarmGerak && !blockAlarm) {
      timingGerak2 = millis();
      Serial.println("Terdeteksi Gerakan");
      longitude[0] = (float) gps.location.lat();
      latitude[0] = (float) gps.location.lng();
      float jarak = (float) HaversineDistance(longitude[0], latitude[0], longitude[1], latitude[1]);
      Serial.print("Latitude :"); Serial.println(latitude[0], 6);
      Serial.print("Longitude :"); Serial.println(longitude[0], 6);
      Serial.print("Distance :"); Serial.println(jarak, 6);
      longitude[1] = (float) longitude[0];
      latitude[1] = (float) latitude[0];
      buzzState = true;
      String msg = "Terindikasi Pencurian\n";
      msg += "Lokasi : ";
      msg += getLocation();
      sendSMS(msg);
    }


    if (millis() - lstWifi >= 5000) {
      lstWifi = millis();
      if (WiFi.status() == WL_CONNECTED) {
        WiFiClient client;
        HTTPClient http;
        String serverName = "http://" + ip + "/api.php";
        http.begin(client, serverName);
        http.addHeader("Content-Type", "application/x-www-form-urlencoded");
        String httpRequestData = "lat=" + String(latitude[0], 6);
        httpRequestData += "&lng=" + String(longitude[0], 6);
        int httpResponseCode = http.POST(httpRequestData);
        //        Serial.print("HTTP Response code : ");
        //        Serial.println(httpResponseCode);
        if (httpResponseCode == 200) {
          JSONVar payload = JSON.parse(http.getString());
          bool status = JSON.stringify(payload["passcode"]) == "\"0\"" ? 0 : 1;
          http.end();
          if (status) {
            http.begin(client, serverName);
            http.addHeader("Content-Type", "application/x-www-form-urlencoded");
            httpResponseCode = http.POST("passcode=1");
            http.end();
            blockAlarm = 1;
          }
          Serial.println(blockAlarm);
        }

        http.end();
      }
    }
  }


  delay(5);
}

String getLocation() {
  if (gps.location.isValid()) {
    longitude[0] = (float) gps.location.lat();
    latitude[0] = (float) gps.location.lng();
    return "https://maps.google.com/?q=" + String(gps.location.lat(), 7) + "," + String(gps.location.lng(), 7);
  } else {
    longitude[0] = 0.00;
    latitude[0] = 0.00;
    return "GPS Searching Signal";
  }
}

void sendSMS(String msg) {
  Serial2.println("AT"); //Handshaking with SIM900
  delay(500);
  if (Serial2.available()) {
    char dt = Serial2.read();
    Serial.write(dt);
  }
  Serial2.println("AT+CMGF=1"); // Configuring TEXT mode
  delay(500);
  if (Serial2.available()) {
    char dt = Serial2.read();
    Serial.write(dt);
  }
  Serial2.println("AT+CMGS=\"" + number + "\"");
  delay(500);
  if (Serial2.available()) {
    char dt = Serial2.read();
    Serial.write(dt);
  }
  Serial2.print(msg); //text content
  delay(500);
  if (Serial2.available()) {
    char dt = Serial2.read();
    Serial.write(dt);
  }
  Serial2.write(26);
}

double HaversineDistance( double y1, double x1, double y2, double x2 )
{
  double nRadius = 6370997.0; // Earth’s radius in Kilometers
  double rad = 0.0174532925199433;
  double nDLat = (y2 - y1) * rad;
  double nDLon = (x2 - x1) * rad;
  y1 *= rad;
  y2 *= rad;

  double nA =   pow( sin(nDLat / 2), 2 ) + cos(y1) * cos(y2) * pow( sin(nDLon / 2), 2 );
  double nC = 2 * atan2( sqrt(nA), sqrt( 1 - nA ));
  double nD = nRadius * nC;
  return nD;
}
