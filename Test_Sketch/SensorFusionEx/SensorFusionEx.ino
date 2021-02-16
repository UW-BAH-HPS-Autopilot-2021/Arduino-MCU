/*
  Arduino LSM9DS1 with SensorFusion Integration

  This example reads the gyroscope, accelerometer, and magnometer values 
  from the LSM9DS1 sensor and inputs them into the SensorFusion library. 
  Absolute Roll, Pitch, and Yaw are then continuously printed to the 
  Serial Monitor or Serial Plotter.

  The circuit:
  - Arduino Nano 33 BLE Sense

  created 12 Feb 2021
  by Chase Deitner
*/

#include <Arduino_LSM9DS1.h>
#include <SensorFusion.h>
SF fusion;

float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat;

void setup() {
  Serial.begin(115200); //serial to display data
  
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");
}

void loop() {

  // now you should read the gyroscope, accelerometer (and magnetometer if you have it also)
  // NOTE: the gyroscope data have to be in radians
  // if you have them in degree convert them with: DEG_TO_RAD example: gx * DEG_TO_RAD

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);   //update acceleration
  }
  
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);      //update gyroscope
  }
  
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);  //update magnometer
  }
  
    
  deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
  //choose only one of these two:
  fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
  //fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick algorithm, it is slower but more accurate

  pitch = fusion.getPitch();
  roll = fusion.getRoll();    //you could also use getRollRadians() ecc
  yaw = fusion.getYaw();
  
  Serial.print(pitch);
  Serial.print('\t');
  Serial.print(roll);
  Serial.print('\t');
  Serial.print(yaw);
  Serial.print('\t');
  Serial.print(ax);
  Serial.print('\t');
  Serial.print(ay);
  Serial.print('\t');
  Serial.println(az);

}
