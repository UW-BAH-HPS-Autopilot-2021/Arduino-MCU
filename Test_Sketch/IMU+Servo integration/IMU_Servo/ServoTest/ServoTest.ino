/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>
#include <Arduino_LSM9DS1.h>
//hello james

Servo servoL;
Servo servoU;
Servo servoR;
Servo servoD;

// twelve servo objects can be created on most boards

int adc_max = 1024;
int joystick_x_90;
int joystick_y_90;
float roll;
float x, y, z;

int pos = 0;    // variable to store the servo position

void setup() {
  servoL.attach(9);  // attaches the servo on pin 9 to the servo object
  servoR.attach(10);  
  servoU.attach(5);  
  servoD.attach(6);  
  //set up adc read pins
  joystick_x_90 = analogRead(A1);
  joystick_y_90 = analogRead(A2);
  roll = 0;
  x = 0;
  y = 0;
  z = 0;
  IMU.begin();  
  IMU.gyroscopeSampleRate();

  
}

void loop() {
//  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
//    // in steps of 1 degree
//    myservo.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  }
//  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
//    myservo.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  }

    float k_p = 25 ;

    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(x, y, z);
    }

    roll = y * (3.1415 / 2);
    
    double x_pos; //= (analogRead(A1)/joystick_x_90) * 90; //outputs 0 to 180 depending on the joystick position
    int x_joy;

    double y_pos;
    int y_joy;

    for (int i = 0; i < 10; i++) {
      x_joy = (analogRead(A1));
      y_joy = (analogRead(A2));
      if ((x_joy > joystick_x_90 + 50) || (x_joy < joystick_x_90 - 50)) {
        x_pos = x_pos + ((x_joy*900)/joystick_x_90);
      } else { 
        x_pos = x_pos + (900);
      }

      if ((y_joy > joystick_y_90 + 50) || (y_joy < joystick_y_90 - 50)) {
        y_pos = y_pos + ((y_joy*900)/joystick_y_90);
      } else { 
        y_pos = y_pos + (900);
      }
    }
    x_pos = x_pos/100;
    y_pos = y_pos/100;

//    x_pos = x_pos + k_p*(roll);
//    y_pos = y_pos + k_p*(roll);
            
    servoL.write(x_pos + k_p*(roll));
    servoR.write(180 - x_pos + (k_p *(roll)));
    servoU.write(y_pos + k_p *(roll));
    servoD.write(180 - y_pos + (k_p *(roll)));
    //delay(10);


}
