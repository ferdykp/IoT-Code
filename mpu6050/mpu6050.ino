#include <Wire.h>

const int MPU = 0x68; //I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int AcXcal, AcYcal, AcZcal, GyXcal, GyYcal, GyZcal, tcal;
double t,tx,tf,pitch,roll,ax,ay,az,gx,gy,gz;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x68);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(MPU);
  Wire.write(0x68);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);

  //Acceleration data correction
  AcXcal = -1617;
  AcYcal = -1073;
  AcZcal = 1219;

  //Temperature correction
  tcal = -1600;

  //Gyro correction
  GyXcal = -71;
  GyYcal = 9;
  GyZcal = 60;


  //read accelerometer data
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) 0x40 (ACCEL_ZOUT_L)

  //read temperature data
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) 0x42 (TEMP_OUT_L)

  //read gyroscope data
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) 0x48 (GYRO_ZOUT_L)

  //temperature calculation
  tx = Tmp + tcal;
  t = tx / 340 + 36.53; //equation for temperature in degrees C from datasheet
  tf = (t * 9 / 5) + 32; //fahrenheit

  Serial.print("Accelerometer: ");
  Serial.print("X = "); Serial.print(AcX + AcXcal);
  Serial.print(" Y = "); Serial.print(AcY + AcYcal);
  Serial.print(" Z = "); Serial.println(AcZ + AcZcal);

  Serial.print("Temperature in celsius = "); Serial.print(t);
  Serial.print(" fahrenheit = "); Serial.println(tf);

  Serial.print("Gyroscope: ");
  Serial.print("X = "); Serial.print(GyX + GyXcal);
  Serial.print(" Y = "); Serial.print(GyY + GyYcal);
  Serial.print(" Z = "); Serial.println(GyZ + GyZcal);
  Serial.println();
  delay(1000);
}