#include <Arduino_LSM9DS1.h>  //IMU

#include <stdio.h>
unsigned long timer;
int delay_freq = 1000; //change frequencies of output

void setup() {
  // put your setup code here, to run once:
  //initialization
  Serial.println(1);
  Serial.begin(9600);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}

void loop() {

  // put your main code here, to run repeatedly:

  //time stamp
  delay(delay_freq);
  timer = millis();
  Serial.print("Time: ");
  Serial.print(timer / 60000);
  Serial.print(":");
  Serial.println((timer % 60000) / 1000);

  //IMU
  float x, y, z;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    //x is the long side angle , y is the short side angle, z is surface angle
    Serial.print(F("x: "));
    Serial.print(int(x * 100) / 100.0);
    Serial.print(" \t");
    Serial.print(F("y: "));
    Serial.print(int(y * 100) / 100.0);
    Serial.print(" \t");
    Serial.print(F("z: "));
    Serial.println(int(z * 100) / 100.0);
  }

  float yaw, pitch, roll;

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(roll, pitch, yaw);

    Serial.print(F("roll: "));
    Serial.print(roll);
    Serial.print(" \t");
    Serial.print(F("pitch: "));
    Serial.print(pitch);
    Serial.print(" \t");
    Serial.print(F("yaw: "));
    Serial.println(yaw);
  }

  float mag_x, mag_y, mag_z;

  if (IMU.magneticFieldAvailable()) {

    IMU.readMagneticField(mag_x, mag_y, mag_z);
    Serial.print(F("mag_x: ")); \
    Serial.print(mag_x);
    Serial.print(" \t");
    Serial.print(F("mag_y: "));
    Serial.print(mag_y);
    Serial.print(" \t");
    Serial.print(F("mag_z: "));
    Serial.print(mag_z);
    Serial.println("uT");
  }

  Serial.println();

}
