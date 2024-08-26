#include <Wire.h>
#include <ESP8266WiFi.h>
#include <MPU6050.h>

// Mengatur SSID dan password WiFi
const char* ssid = "PANGGABEAN";
const char* password = "panggabean123";

// Inisialisasi objek sensor MPU6050
MPU6050 mpu;

// Inisialisasi pin buzzer
const int buzzerPin = D1;

// Inisialisasi pin relay
const int relayPin = D2;

// Variabel untuk menyimpan nilai pembacaan sensor MPU6050
int accelerometerX, accelerometerY, accelerometerZ;
int gyroX, gyroY, gyroZ;

void setup() {
  // Mulai komunikasi serial
  Serial.begin(115200);

  // Menghubungkan ke WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Inisialisasi sensor MPU6050
  mpu.initialize();
  Serial.println("MPU6050 Initialized");

  // Mengatur mode sensor MPU6050
  mpu.setFullScaleAccelRange(MPU6050_IMU::MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_IMU::MPU6050_GYRO_FS_250);

  // Mengatur pin relay sebagai OUTPUT
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);
}

void loop() {
  // Baca data sensor MPU6050
int16_t accelerometerX, accelerometerY, accelerometerZ;
int16_t gyroX, gyroY, gyroZ;


  // Proses data sensor sesuai kebutuhan
  // Misalnya, berdasarkan perubahan sudut, getaran, atau gerakan

  // Contoh: Jika akselerasi pada sumbu X lebih besar dari ambang batas tertentu, aktifkan relay dan bunyikan buzzer
  if (abs(accelerometerX) > 5000) {
    // Aktifkan relay
    digitalWrite(relayPin, HIGH);

    // Bunyikan buzzer
    tone(buzzerPin, 1000, 500);
    delay(1000); // Tahan 1 detik sebelum memainkan bunyi lagi

    // Matikan relay setelah jangka waktu tertentu
    delay(2000);
    digitalWrite(relayPin, LOW);
  }

  // Tambahkan penundaan agar tidak terlalu sering membaca sensor
  delay(100);
}
