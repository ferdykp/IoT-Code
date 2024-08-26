#include <Wire.h>
//#include <ESP8266WiFi.h>
#include <MPU6050.h>

//// Mengatur SSID dan password WiFi
//const char* ssid = "PANGGABEAN";
//const char* password = "panggabean123";

// Hubungkan SCL MPU6050 ke pin D1 dan SDA MPU6050 ke pin D2 pada ESP8266
#define SCL_PIN D1
#define SDA_PIN D2

// Inisialisasi objek sensor MPU6050
MPU6050 mpu;

// Inisialisasi pin buzzer
const int buzzerPin = D1;

// Inisialisasi pin relay
const int relayPin = D2;

// Ambang batas akselerasi yang ditentukan
const int threshold = 500;

// Variabel untuk menyimpan nilai pembacaan sensor MPU6050
int16_t accelerometerX, accelerometerY, accelerometerZ;

void setup() {
  // Mulai komunikasi serial
  Serial.begin(115200);

//  // Menghubungkan ke WiFi
//  WiFi.begin(ssid, password);
//  while (WiFi.status() != WL_CONNECTED) {
//    delay(1000);
//    Serial.println("Connecting to WiFi...");
//  }
//  Serial.println("Connected to WiFi");

Wire.begin(SDA_PIN, SCL_PIN); // Inisialisasi koneksi I2C dengan pin SDA dan SCL yang Anda tentukan
//  mpu.begin();

  // Inisialisasi sensor MPU6050
  mpu.initialize();
  Serial.println("MPU6050 Initialized");

  // Mengatur mode sensor MPU6050
  mpu.setFullScaleAccelRange(MPU6050_IMU::MPU6050_ACCEL_FS_2);

  // Mengatur pin buzzer dan relay sebagai OUTPUT
//  pinMode(wsd, OUTPUT);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);

  // Menampilkan header data pada Serial Monitor
  Serial.println("AccelX\tAccelY\tAccelZ");
}

void loop() {
  // Baca data sensor MPU6050
  mpu.getAcceleration(&accelerometerX, &accelerometerY, &accelerometerZ);

  // Proses data sensor sesuai kebutuhan
  // Misalnya, berdasarkan perubahan sudut, getaran, atau gerakan

  // Contoh: Jika akselerasi pada sumbu X lebih besar dari ambang batas tertentu
  if (abs(accelerometerX) > threshold) {
    // Aktifkan relay
    digitalWrite(relayPin, HIGH);

    // Bunyikan buzzer
    tone(buzzerPin, 1000, 500);
    delay(1000); // Tahan 1 detik sebelum memainkan bunyi lagi

    // Matikan relay setelah jangka waktu tertentu
    digitalWrite(relayPin, LOW);
  }

  // Tampilkan data pada Serial Monitor
  Serial.print(accelerometerX);
  Serial.print("\t");
  Serial.print(accelerometerY);
  Serial.print("\t");
  Serial.println(accelerometerZ);

  // Tambahkan penundaan agar tidak terlalu sering membaca sensor
  delay(100);
}
