#include <Wire.h>
#include <DHT.h>
#include <DHT_U.h>
#include <math.h> //library includes mathematical functions
#include <MPU6050.h>
#include <ModbusIP_ESP8266.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>


// Initialize Telegram BOT
#define BOTtoken "6298927547:AAHPbRMyVbTLfkrBjtPtUBsVSgzXbwkfHmE"  // your Bot Token (Get from Botfather)

// Use @myidbot to find out the chat ID of an individual or a group
// Also note that you need to click "start" on a bot before it can
// message you
#define idChat "1418915527"

X509List cert(TELEGRAM_CERTIFICATE_ROOT);

WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

// Checks for new messages every 1 second.
//int botRequestDelay = 1000;
//unsigned long lastTimeBotRan;

// Hubungkan SCL MPU6050 ke pin D1 dan SDA MPU6050 ke pin D2 pada ESP8266
#define SCL_PIN D1
#define SDA_PIN D2

ModbusIP mb;
DHT_Unified dht(D1, DHT11);
MPU6050 mpu;

uint32_t lastMillis = 0;
const int mqPin = A0; // Pin analog untuk sensor MQ136
float mqValue = 0.0f;
float temperature = 0.0f;
float humidity = 0.0f;
int16_t pitch,roll,ax,ay,az,gx,gy,gz, pitch1, roll1;
const int threshold = -80.0;

void setup() {
  Serial.begin(115200);

  configTime(0, 0, "pool.ntp.org");      // get UTC time via NTP
  client.setTrustAnchors(&cert); // Add root certificate for api.telegram.org
  
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
//  mb.addHreg(3, ax); // Register Modbus untuk akselerasi x
//  mb.addHreg(4, ay); // Register Modbus untuk akselerasi y
//  mb.addHreg(5, az); // Register Modbus untuk akselerasi z
//  mb.addHreg(6, gx); // Register Modbus untuk kecepatan sudut x
//  mb.addHreg(7, gy); // Register Modbus untuk kecepatan sudut y
//  mb.addHreg(8, gz); // Register Modbus untuk kecepatan sudut z
  mb.addHreg(3, pitch);
  mb.addHreg(4, roll);
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

//  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;
//  float roll = atan2(ay, az) * 180 / PI;    
  
  float pitch1 = atan(ax/sqrt((ay*ay) + (az*az))); //pitch calculation
  float roll1 = atan(ay/sqrt((ax*ax) + (az*az))); //roll calculation

  float pitch = pitch1 * (180.0/3.14);
  float roll = roll1 * (180.0/3.14) ;

//  sensors_event_t pitch, roll;
//  mpu.getEvent(&pitch, &roll);


  // Update nilai register Modbus
  mb.Hreg(0, mqValue);
  mb.Hreg(1, temperature);
  mb.Hreg(2, humidity);
//  mb.Hreg(3, ax);
//  mb.Hreg(4, ay);
//  mb.Hreg(5, az);
//  mb.Hreg(6, gx);
//  mb.Hreg(7, gy);
//  mb.Hreg(8, gz);
  mb.Hreg(3, pitch);
  mb.Hreg(4, roll);

  if (temperature > 35.00) {
    bot.sendChatAction(idChat, "Sedang mengetik...");
    delay(3000);
    
    String suhu = "Intensitas suhu : ";
    suhu += float(temperature);
    suhu += " *C\n";
    suhu += "Batas Limit!\n";
    bot.sendMessage(idChat, suhu, "");
    Serial.print("Mengirim data sensor ke telegram");
  } 
    else if (humidity > 80.00) {
    bot.sendChatAction(idChat, "Sedang mengetik...");
    delay(3000);
    
    String kelembapan = "Kelembapan : ";
    kelembapan += float(humidity);
    kelembapan += " %\n";
    kelembapan += "Batas Limit!\n";
    bot.sendMessage(idChat, kelembapan, "");
    Serial.print("Mengirim data sensor ke telegram");
  }

  if (pitch > 80.0 || roll > 80.0) {
//    bot.sendChatAction(idChat, "Sedang mengetik...");
//    delay(2000);
    
    String gyro = "Nilai Pitch: " + String(pitch) + "°,\nNilai Roll: " + String(roll) + "°" + "Barang Miring Kedepan";
    bot.sendMessage(idChat, gyro, "");
    Serial.print("Mengirim data sensor ke telegram");
  }

  if (mqValue > 600) {
    bot.sendChatAction(idChat, "Sedang mengetik...");
    delay(2000);
    
    String kebusukan = "Kebusukan: " + String(mqValue) + "Susu Busuk";
    bot.sendMessage(idChat, kebusukan, "");
    Serial.print("Mengirim data sensor ke telegram\n");
  }
   
  Serial.print("Nilai MQ136: ");
  Serial.println(mqValue);
  Serial.print("Suhu: ");
  Serial.println(temperature);
  Serial.print("Kelembaban: ");
  Serial.println(humidity);
//  Serial.print("Akselerasi x: ");
//  Serial.println(ax);
//  Serial.print("Akselerasi y: ");
//  Serial.println(ay);
//  Serial.print("Akselerasi z: ");
//  Serial.println(az);
//  Serial.print("Kecepatan sudut x: ");
//  Serial.println(gx);
//  Serial.print("Kecepatan sudut y: ");
//  Serial.println(gy);
//  Serial.print("Kecepatan sudut z: ");
//  Serial.println(gz);
  Serial.print("Pitch : ");
  Serial.println(pitch);
  Serial.print("Roll : ");
  Serial.println(roll);

  mb.task();
  delay(1000); // Tunggu 1 detik sebelum membaca lagi
}
