/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>
//hello james

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int adc_max = 1024;
int joystick_x_90;
int joystick_y_90;

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object

  //set up adc read pins
  joystick_x_90 = analogRead(A1);
  joystick_y_90 = analogRead(A2);
  
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
    double x_pos; //= (analogRead(A1)/joystick_x_90) * 90; //outputs 0 to 180 depending on the joystick position
    int x_joy;

    for (int i = 0; i < 10; i++) {
      x_joy = (analogRead(A1));
      if ((x_joy > joystick_x_90 + 40) || (x_joy < joystick_x_90 - 40)) {
        x_pos = x_pos + ((x_joy*900)/joystick_x_90);
      } else { 
        x_pos = x_pos + (900);
      }
    }
    x_pos = x_pos/100;

    //int y_pos = ((analogRead(A2) * 90) / joystick_y_90);
        
    myservo.write(x_pos);
    



}
