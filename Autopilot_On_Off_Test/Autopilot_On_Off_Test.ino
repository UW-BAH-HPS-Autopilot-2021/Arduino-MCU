

#include <Servo.h>
#include <AHRS_Nano33BLE_LSM9DS1.h> 
#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#include <SPI.h>
#include <SD.h>





// U8g2 Contructor List (Picture Loop Page Buffer)
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp

//U8G2_SSD1327_EA_W128128_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ 5, /* data=*/ 4, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SSD1327_EA_W128128_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); /* Uno: A4=SDA, A5=SCL, add "u8g2.setBusClock(400000);" into setup() for speedup if possible */
//U8G2_SSD1327_MIDAS_128X128_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); /* Uno: A4=SDA, A5=SCL, add "u8g2.setBusClock(400000);" into setup() for speedup if possible */

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// End of constructor list


int myLED  = 10;

//Joystick Variables
Servo servoL;
Servo servoU;
Servo servoR;
Servo servoD;

//ServoTimer2 servoL;
//ServoTimer2 servoU;
//ServoTimer2 servoR;
//ServoTimer2 servoD;

int joystick_x_90;
int joystick_y_90;
float x_pos;
int x_joy;
float y_pos;
int y_joy;
float x, y, z;
int pos = 0;
float JOYSTICK_SENSITIVITY = 0.25; // a value of 1 gives 90 degrees of control

//Autopilot Variables
float integral_sum = 0;
float dt;
float roll_rate;
float current_roll;
float autopilot_output;
bool autopilot_on;
float k_p = 5;
float k_i = 0.1;
float k_d = 10;

//SensorFusion Variables
float pitch, roll, yaw;
float gx, gy, gz, ax, ay, az, mx, my, mz;
float deltat;
float temperature;

//Voltage Measurment Variables
float V_BATT;
float V_5V; 
float V_3V3;
float percent_batt;

//Display Variables
int prev_time;

//SD Card Variables
File ddata;
String file = "hello21.txt"; 

//Method Instantiations
void initializeSF();
void initializeSF_SD();
void initializeSDCard();
void initializeAnalogJoystick();
void calibrateIMU();
void calibrateIMU_SD();
void updateIMU();
void serialDiagnostics();
void SDcardDiagnostics();
void diagnosticDisplay();
void voltageMonitoring();
void errorCode(int blinkNum);
void joystick_control();


void setup() {
    //initialize SD card Diagnostics
   
    // Initialize Input Switches
    pinMode(7, INPUT_PULLUP); //Calibrate
    pinMode(8, INPUT_PULLUP); //Autopilot Engage
    pinMode(myLED, OUTPUT);
    digitalWrite(myLED, HIGH);
    delay(500);
    digitalWrite(myLED, LOW);

    //Initialize Diagnostic I2C Display
    u8g2.begin();
    u8x8_cad_StartTransfer(u8g2.getU8x8());
    u8x8_cad_SendCmd( u8g2.getU8x8(), 0xbe);
    u8x8_cad_SendArg( u8g2.getU8x8(), 15);    // Max Brightness (VCOM)
    u8x8_cad_EndTransfer(u8g2.getU8x8()); 
    u8g2.setFont(u8g2_font_fub17_tf); 
    u8g2.setContrast(0xFF); //Max Brightness (Contrast)
    u8g2.setBusClock(400000);

  
    voltageMonitoring();
    
//    u8g2.firstPage();
//    do {
//      u8g2.setCursor(0, 15);
//      u8g2.setFont(u8g2_font_fub11_tf); 
//      u8g2.print(u8x8_u8toa(int(V_BATT), 2));
//      u8g2.print(".");
//      u8g2.print(u8x8_u8toa((int(V_BATT * 100) % 100), 2));
//      u8g2.print("V        ");
//      u8g2.print(u8x8_u8toa(int(percent_batt), 2));
//      u8g2.print("%"); 
//      u8g2.setFont(u8g2_font_fub17_tf);    
//      u8g2.setCursor(0, 39);  
//      u8g2.print("Autopilot");
//      u8g2.setCursor(25, 63);  
//      u8g2.print("OFF");
//    } while ( u8g2.nextPage() );


    //is Autopilot enabled
    if (!digitalRead(8)) {
      autopilot_on = true;
    } else {
      autopilot_on = false;
    }

}

void loop() {
  
//  if (digitalRead(8)) {
//    digitalWrite(myLED, LOW);
//  } else {
//    digitalWrite(myLED, HIGH);
//  }

    

  if (digitalRead(8)) { //If autopilot is enabled
    autopilot_output = 0;
    digitalWrite(myLED, LOW);
    if (autopilot_on) { 
      autopilot_on = false;
      u8g2.firstPage();
      do {
        u8g2.setCursor(3, 30);
        u8g2.setFont(u8g2_font_fub11_tf); 
        u8g2.print(F("BEGIN SDCARD")); 
        u8g2.setCursor(25, 63);
        u8g2.print(F("STARTUP")); 
      } while ( u8g2.nextPage() );
    }
  } else {
    autopilot_output = (k_p * current_roll) + (k_i * integral_sum) + (k_d * roll_rate);
    digitalWrite(myLED, HIGH);
    if (!autopilot_on) {
      autopilot_on = true;
      u8g2.firstPage();
      u8g2.firstPage();
      do {
        u8g2.setCursor(0, 15);
        u8g2.setFont(u8g2_font_fub11_tf); 
        u8g2.print(u8x8_u8toa(int(V_BATT), 2));
        u8g2.print(".");
        u8g2.print(u8x8_u8toa((int(V_BATT * 100) % 100), 2));
        u8g2.print("V        ");
        u8g2.print(u8x8_u8toa((int(percent_batt)), 2));
        u8g2.print("%");
        u8g2.setFont(u8g2_font_fub17_tf);    
        u8g2.setCursor(0, 39);  
        u8g2.print("CALIBRATE");
        u8g2.setCursor(20, 63);  
        u8g2.print("ACTIVE");
      } while ( u8g2.nextPage() );  
    }
  }

  //delay(100);
    
}



void initializeSF() {
  
    //Print to Display
    u8g2.firstPage();
    do {
    u8g2.setCursor(15, 25);
    u8g2.setFont(u8g2_font_fub11_tf); 
    u8g2.print(F("BEGIN SERIAL")); 
    u8g2.setCursor(25, 60);
    u8g2.print(F("STARTUP")); 
    } while ( u8g2.nextPage() );
  
    //IMU Start
    Serial.begin(38400);
    while (!Serial);
    Serial.println("Waited for Serial.");
  
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
      calibrateIMU();
    }
}

void initializeSF_SD() {
  
    //Print to Display
    u8g2.firstPage();
    do {
    u8g2.setCursor(3, 30);
    u8g2.setFont(u8g2_font_fub11_tf); 
    u8g2.print(F("BEGIN SDCARD")); 
    u8g2.setCursor(25, 63);
    u8g2.print(F("STARTUP")); 
    } while ( u8g2.nextPage() );
  
    //IMU Start
    ddata.println("Initializing IMU");
    if (!IMU.start())
    {
      ddata.println("Failed to initialize IMU");
      ddata.close();
      while (1) {errorCode(2);}
    }
    
    ddata.println("Perform gyro and accel self test.");
    if (!IMU.selftestLSM9DS1())
    {
      ddata.println("Failed self test"); // check function of gyro and accelerometer via self test
      ddata.close();
      while (1) {errorCode(2);}
    }
    else
    {  
      ddata.println("LSM9DS1 IMU Initialization: GO");
  
      ddata.print("accel sensitivity is "); ddata.print(IMU.accelerationSensitivity()); ddata.println(" LSB/mg");
      ddata.print("gyro sensitivity is "); ddata.print(IMU.gyroscopeSensitivity()); ddata.println(" LSB/mdps");
      ddata.print("mag sensitivity is "); ddata.print(IMU.magnometerSensitivity()); ddata.println(" LSB/mGauss");
      calibrateIMU_SD();
    }
}

void initializeSDCard() {
    if (!SD.begin(4)) { //Select the CS pin (Pin 4)
      while (1) {errorCode(3);}         
    }
    ddata = SD.open(file, FILE_WRITE);
    if (ddata) {
      ddata.println("SD Card Initialization Complete");
    } else {
    // if the file didn't open, print an error:
      while(1) {errorCode(4);}
  }
}

void initializeAnalogJoystick() {
    joystick_x_90 = analogRead(A1);
    joystick_y_90 = analogRead(A2);
    
    if ((joystick_x_90 < 700) && (joystick_x_90 > 300)) {
      ddata.print("Joystick_X = ");
      ddata.print(joystick_x_90);
      ddata.println(" : GO");
    } else {
      ddata.print("Joystick_X = ");
      ddata.print(joystick_x_90);
      ddata.println(" : *** NO GO ***");
    }
    
    if ((joystick_y_90 < 700) && (joystick_y_90 > 300)) {
      ddata.print("Joystick_Y = ");
      ddata.print(joystick_y_90);
      ddata.println(" : GO");
    } else {
      ddata.print("Joystick_Y = ");
      ddata.print(joystick_y_90);
      ddata.println(" : *** NO GO ***");
    }
    
    roll = 0;
    x = 0;
    y = 0;
    z = 0;
}


void calibrateIMU() {

    //Print to Display
    u8g2.firstPage();
    do {
    u8g2.setCursor(0, 50);
    u8g2.setFont(u8g2_font_fub17_tf); 
    u8g2.print(F("CALIBRATE")); 
    u8g2.setCursor(20, 80);
    u8g2.print(F("ACTIVE")); 
    } while ( u8g2.nextPage() );
   
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

    integral_sum = 0.0;
    deltat = IMU.updateDeltat(); //this have to be done before calling the fusion update
}

void calibrateIMU_SD() {
    ddata.println("Calibrating IMU");
    
    //Print to Display
    u8g2.firstPage();
    do {
      u8g2.setCursor(0, 15);
      u8g2.setFont(u8g2_font_fub11_tf); 
      u8g2.print(u8x8_u8toa(int(V_BATT), 2));
      u8g2.print(".");
      u8g2.print(u8x8_u8toa((int(V_BATT * 100) % 100), 2));
      u8g2.print("V        ");
      u8g2.print(u8x8_u8toa((int(percent_batt)), 2));
      u8g2.print("%");
      u8g2.setFont(u8g2_font_fub17_tf);    
      u8g2.setCursor(0, 39);  
      u8g2.print("CALIBRATE");
      u8g2.setCursor(20, 63);  
      u8g2.print("ACTIVE");
    } while ( u8g2.nextPage() );
   
    ddata.println("Calibrate gyro and accel");
    IMU.calibrateAccelGyro(); // Calibrate gyro and accelerometers, load biases in bias registers

    float* accelBias = IMU.getAccelBias();
    float* gyroBias = IMU.getGyroBias();

    ddata.println("accel biases (mg)"); ddata.println(1000.*accelBias[0]); ddata.println(1000.*accelBias[1]); ddata.println(1000.*accelBias[2]);
    ddata.println("gyro biases (dps)"); ddata.println(gyroBias[0]); ddata.println(gyroBias[1]); ddata.println(gyroBias[2]);

    IMU.calibrateMag();
    float* magBias = IMU.getMagBias();
    ddata.println("mag biases (mG)"); ddata.println(1000.*magBias[0]); ddata.println(1000.*magBias[1]); ddata.println(1000.*magBias[2]); 

    IMU.initLSM9DS1(); 
    ddata.println("LSM9DS1 IMU Calibration: GO"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

    ddata.println("Assumption is data is in the following format:");
    ddata.println("uptime (milliseconds), roll (degrees), pitch (degrees), yaw (degrees), gyrotemperatureC (Celcius)");
    ddata.println("Begin Outputting Data!");

    integral_sum = 0.0;

    u8g2.firstPage();
    do {
      u8g2.setCursor(0, 15);
      u8g2.setFont(u8g2_font_fub11_tf); 
      u8g2.print(u8x8_u8toa(int(V_BATT), 2));
      u8g2.print(".");
      u8g2.print(u8x8_u8toa((int(V_BATT * 100) % 100), 2));
      u8g2.print("V        ");
      u8g2.print(u8x8_u8toa(int(percent_batt), 2));
      u8g2.print("%");
      u8g2.setFont(u8g2_font_fub17_tf);    
      u8g2.setCursor(0, 39);  
      u8g2.print("Autopilot");
      u8g2.setCursor(20, 63);  
      u8g2.print("ACTIVE");
    } while ( u8g2.nextPage() );

    
    deltat = IMU.updateDeltat(); //this have to be done before calling the fusion update


    
}

void updateIMU() {
    
  
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
}

void serialDiagnostics() {
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

//Creates a CSV file and stores the data to the external SD card
void SDcardDiagnostics() {
    ddata.print(millis());
    ddata.print(',');
  //  ddata.print(" Roll: ");
    ddata.print(roll);
    ddata.print(',');
    ddata.print(az);
    ddata.print(',');
    ddata.print(pitch);
    ddata.print(',');
    ddata.print(yaw);
    ddata.print(',');
  //  ddata.print(" Integral Sum: ");
    ddata.print(integral_sum);
    ddata.print(',');
  //  ddata.print(" Autopilot Fin Angle: ");
    ddata.print(autopilot_output);
    ddata.print(',');
    ddata.print(roll_rate);
    ddata.print(',');
  //  ddata.print(" VBATT: ");
    ddata.print(V_BATT);
    ddata.print(',');
  //  ddata.print(" 5V: ");
    ddata.print(V_5V);
    ddata.print(',');
  //  ddata.print(" 3.3V: ");
    ddata.println(V_3V3);
    
}

void diagnosticDisplay() {
    float _roll;
    int neg_roll;
    if (roll < 0) {
      _roll = -1 * roll;
      neg_roll = 1;
    } else {
      _roll = roll;
      neg_roll = 0;
    }
   
    u8g2.setFont(u8g2_font_fub14_tf);
    u8g2.firstPage();
    do {
      u8g2.setCursor(0, 20);
      u8g2.print(F("Autopilot ON"));
      u8g2.setCursor(0, 50);
      u8g2.print("Roll = ");

      u8g2.setFont(u8g2_font_fub17_tf);
      if (neg_roll) {
        u8g2.print("-");
      }
      u8g2.print(u8x8_u8toa(int(_roll), 1));
      u8g2.print(".");
      u8g2.print(u8x8_u8toa((int(_roll * 10) % 10), 1));
      u8g2.setFont(u8g2_font_fub11_tf);  
      u8g2.setCursor(0, 90);  
      u8g2.print("Batt=");
      u8g2.print(u8x8_u8toa(int(V_BATT), 2));
      u8g2.print(".");
      u8g2.print(u8x8_u8toa((int(V_BATT * 100) % 100), 2));
      u8g2.print("V");
      u8g2.setCursor(0, 110);  
      u8g2.print(u8x8_u8toa(int(V_5V), 1));
      u8g2.print(".");
      u8g2.print(u8x8_u8toa((int(V_5V * 100) % 100), 2));
      u8g2.print("V");
      u8g2.print(", ");
      u8g2.print(u8x8_u8toa(int(V_3V3), 1));
      u8g2.print(".");
      u8g2.print(u8x8_u8toa((int(V_3V3 * 100) % 100), 2));
      u8g2.print("V");          
    } while ( u8g2.nextPage() );


}

void voltageMonitoring() {
    int V_BATT_READ = analogRead(A7);
    // read the VOUT_5V input on analog pin A6:
    int V_5V_READ = analogRead(A6);
    // read the VOUT_3V3 input on analog pin A3:
    int V_3V3_READ = analogRead(A3);
    
    // Convert the integer analog input (which goes from 0 - 1023) to input voltage at pin:
    // 0.03 offset based on pin measurements with multimeter
    float V_BATT_IN = 0.03 + V_BATT_READ * 3.3 / 1023.0;
    float V_5V_IN = 0.03 + V_5V_READ * 3.3 / 1023.0;
    float V_3V3_IN = 0.03 + V_3V3_READ * 3.3 / 1023.0;
    
    // Convert the pin voltage to original voltage based on measured divider values.
    // General formula: V = V_IN * (R1 + R2) / R1
    V_BATT = V_BATT_IN * (103e3 + 22.5e3) / 22.5e3;
    V_5V = V_5V_IN * (104e3 + 47.2e3) / 47.2e3;
    V_3V3 = V_3V3_IN * (102e3 + 47.3e3) / 47.3e3;

    percent_batt = ((V_BATT - 9.0) / 3.6) * 100.0;

}

void errorCode(int blinkNum) {
    int i = 0;
    pinMode(myLED, OUTPUT);
    for (i=0; i<blinkNum; i++) {
      digitalWrite(myLED, HIGH);
      delay(150);
      digitalWrite(myLED, LOW);
      delay(150);                
    }
    delay(1000);
    
}

void joystick_control() {
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
    x_pos = (((x_pos/100) - 90) * JOYSTICK_SENSITIVITY) + 90;
    y_pos = (((y_pos/100) - 90) * JOYSTICK_SENSITIVITY) + 90;
}   
