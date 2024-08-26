#include <dummy.h>

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h> //library allows communication with I2C / TWI devices
#include <math.h> //library includes mathematical functions
#include <MPU6050.h>

MPU6050 mpu;

const int MPU=0x68; //I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //16-bit integers
int AcXcal,AcYcal,AcZcal,GyXcal,GyYcal,GyZcal,tcal; //calibration variables
double t,tx,tf,pitch,roll,ax,ay,az,gx,gy,gz;
const int buzzerPin = D4;
const int relayPin = D3;
const int threshold1 = 60;
const int threshold2 = 1;
bool tilted = false; // Initialize the tilted state as false

const char* ssid = "?";
const char* password = "ferdykurnia";
const char* mqtt_server = "broker.hivemq.com";  // test.mosquitto.org
static const int RXPin = 12, TXPin = 13;
static const uint32_t GPSBaud = 9600;
char locationBuffer[50];

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("device/temp", "MQTT Server is Connected");
      // ... and resubscribe
      client.subscribe("device/led");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
    Wire.begin(); //initiate wire library and I2C
    Wire.beginTransmission(MPU); //begin transmission to I2C slave device
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0); // set to zero (wakes up the MPU-6050)  
    Wire.endTransmission(true); //ends transmission to I2C slave device
    Serial.begin(9600); //serial communication at 9600 bauds
    pinMode(relayPin, OUTPUT);
    pinMode(buzzerPin, OUTPUT);

    digitalWrite(relayPin, LOW);      // Matikan relay secara awal
    digitalWrite(buzzerPin, LOW);     // Matikan buzzer secara awal

    pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
    Serial.begin(115200);
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    ss.begin(GPSBaud);
}


//function to convert accelerometer values into pitch and roll
void getAngle(int Ax,int Ay,int Az) 
{
    double x = Ax;
    double y = Ay;
    double z = Az;

    pitch = atan(x/sqrt((y*y) + (z*z))); //pitch calculation
    roll = atan(y/sqrt((x*x) + (z*z))); //roll calculation

    //converting radians into degrees
    pitch = pitch * (180.0/3.14);
    roll = roll * (180.0/3.14) ;
}


void loop()
{
// This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0){
    gps.encode(ss.read());
    if (gps.location.isUpdated()){
      Serial.print("Latitude= "); 
      Serial.print(gps.location.lat(), 6);
      Serial.print(" Longitude= "); 
      Serial.println(gps.location.lng(), 6);
    }
  }

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;

    // Format latitude and longitude into the locationBuffer array
    snprintf(locationBuffer, sizeof(locationBuffer), "Latitude=%.6f Longitude=%.6f", gps.location.lat(), gps.location.lng());

    Serial.print("Publish message: ");
    Serial.println(locationBuffer);

    // Publish the formatted location to MQTT
    client.publish("hapid22", locationBuffer);
  }
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0){
    gps.encode(ss.read());
    if (gps.location.isUpdated()){
      Serial.print("Latitude= "); 
      Serial.print(gps.location.lat(), 6);
      Serial.print(" Longitude= "); 
      Serial.println(gps.location.lng(), 6);
    Wire.beginTransmission(MPU); //begin transmission to I2C slave device
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false); //restarts transmission to I2C slave device
    Wire.requestFrom(MPU,14,true); //request 14 registers in total  

    //Acceleration data correction
//    AcXcal = -950;
//    AcYcal = -300;
//    AcZcal = 0;
//
//    //Temperature correction
//    tcal = -1600;
//
//    //Gyro correction
//    GyXcal = 480;
//    GyYcal = 170;
//    GyZcal = 210;


    //read accelerometer data
    AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) 0x3C (ACCEL_XOUT_L)  
    AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) 0x3E (ACCEL_YOUT_L) 
    AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) 0x40 (ACCEL_ZOUT_L)
  
    //read temperature data 
    Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) 0x42 (TEMP_OUT_L) 
  
    //read gyroscope data
    GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) 0x48 (GYRO_ZOUT_L) 

    //temperature calculation
    tx = Tmp + tcal;
    t = tx/340 + 36.53; //equation for temperature in degrees C from datasheet
    tf = (t * 9/5) + 32; //fahrenheit

    ax = AcX + AcXcal;
    ay = AcY + AcYcal;
    az = AcZ + AcZcal;

    gx = GyX + GyXcal;
    gy = GyY + GyYcal;
    gz = GyZ + GyZcal;

    //get pitch/roll
    getAngle(AcX,AcY,AcZ);
//
//    if (gy > threshold){
//      digitalWrite(relayPin, HIGH);
//
//    // Bunyikan buzzer
//    tone(buzzerPin, 1000, 500);
//    delay(1000); // Tahan 1 detik sebelum memainkan bunyi lagi
//    
//      // Matikan relay setelah jangka waktu tertentu
//    digitalWrite(relayPin, LOW);
//    digitalWrite(buzzerPin, LOW);
//    
//    }

 getAngle(AcX, AcY, AcZ);

    if (abs(pitch) < threshold1 || abs(roll) < threshold2) {
        digitalWrite(buzzerPin, HIGH); // Turn on the buzzer
        digitalWrite(relayPin, HIGH); // Turn on the buzzer
        tilted = true; // Mark the device as tilted
//        snprintf(locationBuffer, sizeof(locationBuffer), "Latitude=%.6f Longitude=%.6f", gps.location.lat(), gps.location.lng());

    } else {
        digitalWrite(buzzerPin, LOW); // Turn off the buzzer
        digitalWrite(relayPin, LOW); // Turn on the buzzer
        tilted = false; // Mark the device as not tilted
        
    }
  
    //printing values to serial port
    Serial.print("Angle: ");
    Serial.print("Pitch = "); Serial.print(pitch);
    Serial.print(" Roll = "); Serial.println(roll);
  
    Serial.print("Accelerometer: ");
    Serial.print("X = "); Serial.print(ax);
    Serial.print(" Y = "); Serial.print(ay);
    Serial.print(" Z = "); Serial.println(az); 
  
    Serial.print("Gyroscope: ");
    Serial.print("X = "); Serial.print(gx);
    Serial.print(" Y = "); Serial.print(gy);
    Serial.print(" Z = "); Serial.println(gz);
  
    delay(1000);
}
  }
}
