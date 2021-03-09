/*
  Arduino Nano 33 BLE with LSM9DS1

  This example reads the gyroscope, accelerometer, and magnometer values 
  from the LSM9DS1 sensor and outputs them to the serial bus.

  Created 23 Feb 2021
  By Miller Sakmar
*/

#include <Arduino_LSM9DS1.h>

float ax, ay, az, gx, gy, gz, mx, my, mz;

void setup() {
  Serial.begin(115200); //serial to display data
  
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}


void loop() {
  
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);   //update acceleration
  }
  
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);      //update gyroscope
  }


  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);  //update magnometer
  }

  Serial.print(ax);
  Serial.print(',');
  Serial.print(ay);
  Serial.print(',');
  Serial.print(az);
  Serial.print(',');
  Serial.print(gx);
  Serial.print(',');
  Serial.print(gy);
  Serial.print(',');
  Serial.print(gz);
  Serial.print(',');
  Serial.print(mx);
  Serial.print(',');
  Serial.print(my);
  Serial.print(',');
  Serial.println(mz);
}