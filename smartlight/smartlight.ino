#include <PZEM004Tv30.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <LoRa.h>
#include <LoRaWanPacket.h>

#include <Wire.h>
#include <Time.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <RTClib.h>

SoftwareSerial serial (5, 6); // PIN SERIAL PZEM
PZEM004Tv30 pzem (serial);
RTC_DS1307 rtc;

// config pin LoRa
const int nss = 10;
const int rst = -1;
const int dio0 = 2;
// config id LoRa
const char *devAddr = "aedd263f";
const char *nwkSKey = "6d7627bb7fec1bfd0000000000000000";
const char *appSKey = "0000000000000000b6da2451d0420853";

char myStr [50];

struct LoRa_config{
  long Frequency;
  int SpreadingFactor;
  long SignalBandwidth;
  int CodingRate4;
  bool enableCrc;
  bool invertIQ;
  int SyncWord;
  int PreambleLength;  
};

static LoRa_config txLoRa = {922000000, 10, 125000, 5, true, false, 0x34, 8};

float I;
int V, i, P, T, C, A, S;
int R = 7;
int count = 0;
int sensorPin = A1;
int reading=0;
float temp=0.0;
char temp2[2];

void setup() {
  Serial.begin(9600);
  pinMode(R, OUTPUT);
  digitalWrite(R, LOW);

  while(!Serial);
  LoRaWanPacket.personalize(devAddr, nwkSKey, appSKey);
  LoRa.setPins(nss, rst, dio0);

  if (!LoRa.begin(txLoRa.Frequency)) {
    Serial.println("LoRa Failed");
    while(true);
  }

  Serial.println("LoRa Succeedded");
  Serial.println();
}

void loop() {
  if (runEvery(10000)){
    LoRa_sendData();
    Serial.println("Data Send");
  }
  
  C = analogRead(A0);
  reading = analogRead(sensorPin); //reading will have a value between 0-1023
  temp = reading * 0.488759;
  
  DateTime now = rtc.now(); 
    Serial.print(',');    
    Serial.print(now.day(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.year(), DEC);
    Serial.print(" ");   
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();



  if (C > 500 || (now.hour() == 21 && now.minute() == 50)){
    digitalWrite(R, LOW);
    S = 0;
    delay(1000);
    if (count == 1){
      data();
      count = 0;
      if(i < 50){
        A = 1;
      }
    }
  }

  else if (C < 500 || (now.hour() == 21 && now.minute() == 55)){
    digitalWrite(R, HIGH);
    S = 1;
    delay(1000);
    if (count == 0){
      data();
      count = 1;
    }
  }
}
void data (){
  reading = analogRead(sensorPin); //reading will have a value between 0-1023
  temp = reading * 0.488759;
  dtostrf(temp,4,2,temp2);
  V = pzem.voltage();
  I = pzem.current();
  P = pzem.power();
  i = I * 1000;

  char * p = myStr;
  *p++ = '{';
  p+=sprintf(p, "\"V\":%d,", V);
  p+=sprintf(p, "\"I\":%d,", i);
  p+=sprintf(p, "\"P\":%d,", P);
  p+=sprintf(p, "\"C\":%d,", C);
  p+=sprintf(p, "\"S\":%d,", S);
//  p+=sprintf(p, "\"A\":%d",  A);
  p+=sprintf(p, "\"temp\":%s", temp2);
  *p++ = '}';
  *p++ = 0;

  Serial.println (myStr);
  Serial.println();
  Serial.print (V);
  Serial.print (" , ");
  Serial.print (i);
  Serial.print (" , ");
  Serial.print (P);
  Serial.print (" , ");
  Serial.print (temp2);
  Serial.print (" , ");
  Serial.print (C);
  Serial.print (" , ");
  Serial.print (S);
  Serial.print (" , ");
  Serial.print (A);
  Serial.println();

  delay(1);
}

boolean runEvery(unsigned long interval)
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}
