

#include <Servo.h>
//#include <Arduino_LSM9DS1.h>
#include <AHRS_Nano33BLE_LSM9DS1.h> 

int myLed  = 11;

//Joystick Variables
Servo servoL;
Servo servoU;
Servo servoR;
Servo servoD;
int joystick_x_90;
int joystick_y_90;
float x_pos;
int x_joy;
float y_pos;
int y_joy;
float x, y, z;

//Autopilot variables
float integral_sum = 0;
float dt;
float roll_rate;
float current_roll;
float autopilot_output;
float k_p = 25;
float k_i = 0.1;
float k_d = 10;

//SensorFusion Variables
float pitch, roll, yaw;
float gx, gy, gz, ax, ay, az, mx, my, mz;
float deltat;
float temperature;

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
    
    //IMU Start
    Serial.begin(38400);
    while (!Serial);
    Serial.println("Waited for Serial.");
    
    // Initialize LED pin
    pinMode(myLed, OUTPUT);
    digitalWrite(myLed, HIGH);
    
    if (!IMU.start())
    {
      Serial.println("Failed to initialize IMU!");
      while (1);
    }
    Serial.println("Perform gyro and accel self test.");
    if (!IMU.selftestLSM9DS1())
    {
      Serial.println("Failed self test"); // check function of gyro and accelerometer via self test
      while (1);
    }
    else
    {  
      Serial.println("LSM9DS1 is online and passed self test...");
  
      Serial.print("accel sensitivity is "); Serial.print(IMU.accelerationSensitivity()); Serial.println(" LSB/mg");
      Serial.print("gyro sensitivity is "); Serial.print(IMU.gyroscopeSensitivity()); Serial.println(" LSB/mdps");
      Serial.print("mag sensitivity is "); Serial.print(IMU.magnometerSensitivity()); Serial.println(" LSB/mGauss");
  
      Serial.println("Calibrate gyro and accel");
      IMU.calibrateAccelGyro(); // Calibrate gyro and accelerometers, load biases in bias registers
  
      float* accelBias = IMU.getAccelBias();
      float* gyroBias = IMU.getGyroBias();
  
      Serial.println("accel biases (mg)"); Serial.println(1000.*accelBias[0]); Serial.println(1000.*accelBias[1]); Serial.println(1000.*accelBias[2]);
      Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
  
      IMU.calibrateMag();
      float* magBias = IMU.getMagBias();
      Serial.println("mag biases (mG)"); Serial.println(1000.*magBias[0]); Serial.println(1000.*magBias[1]); Serial.println(1000.*magBias[2]); 
  
      IMU.initLSM9DS1(); 
      Serial.println("LSM9DS1 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
  
      Serial.println("Assumption is data is in the following format:");
      Serial.println("uptime (milliseconds), roll (degrees), pitch (degrees), yaw (degrees), gyrotemperatureC (Celcius)");
      Serial.println("Begin Outputting Data!");
    } 
  
}

void loop() {

    if (IMU.accelerometerReady()) {  // check if new accel data is ready  
      IMU.readAccel(ax, ay, az);
    } 
  
    if (IMU.gyroscopeReady()) {  // check if new gyro data is ready  
      IMU.readGyro(gx, gy, gz);
    }
  
    if (IMU.magnometerReady()) {  // check if new mag data is ready  
      IMU.readMag(mx, my, mz);
    }
  
    deltat = IMU.updateDeltat(); //this have to be done before calling the fusion update
    IMU.MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, -mx, my, mz, deltat);
  
    roll = IMU.rollDegrees();    //you could also use rollRadians()
    pitch = IMU.pitchDegrees();
    yaw = IMU.yawDegrees();
    temperature = IMU.readTempC();
    
    current_roll = roll * (3.1415926 / 180);
    integral_sum = integral_sum + (current_roll * deltat);
    roll_rate = gy * (3.1415926 / 180);
    
    autopilot_output = (k_p * current_roll) + (k_i * integral_sum) + (k_d * roll_rate);
    



    //Record and average 10 values of joystick control
    x_pos = 0;
    y_pos = 0;

    for (int i = 0; i < 10; i++) {
      x_joy = (analogRead(A1));
      y_joy = (analogRead(A2));
      if ((x_joy > joystick_x_90 + 70) || (x_joy < joystick_x_90 - 70)) {
        x_pos = x_pos + ((x_joy*900)/joystick_x_90);
      } else { 
        x_pos = x_pos + (900);
      }

      if ((y_joy > joystick_y_90 + 70) || (y_joy < joystick_y_90 - 70)) {
        y_pos = y_pos + ((y_joy*900)/joystick_y_90);
      } else { 
        y_pos = y_pos + (900);
      }
    }
    x_pos = 90; //x_pos/100;
    y_pos = 90; //y_pos/100;

//    x_pos = 90; //x_pos + k_p*(roll);
//    y_pos = 90; //y_pos + k_p*(roll);
            
    servoL.write(x_pos + autopilot_output);
    servoR.write(180 - x_pos + autopilot_output);
    servoU.write(y_pos + autopilot_output);
    servoD.write(180 - y_pos + autopilot_output);
    //delay(10);

  Serial.print(millis());
  Serial.print(',');
  Serial.print(" Roll: ");
  Serial.print(roll);
//  Serial.print(',');
//  Serial.print(pitch);
//  Serial.print(',');
//  Serial.print(yaw);
  Serial.print(',');
  Serial.print(" Integral Sum: ");
  Serial.print(integral_sum);
  Serial.print(',');
  Serial.print(" Autopilot Fin Angle: ");
  Serial.println(autopilot_output);
//  Serial.print(',');
//  Serial.println(roll_rate);
}
