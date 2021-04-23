// Modified "ReadAnalogVoltage" sketch in public domain: http://www.arduino.cc/en/Tutorial/ReadAnalogVoltage
// Modified by: James Lee
// Modify Date: 4/22/2021

// *******
// GLOBALS

// Timestamp
unsigned long elapsedtime;
// END GLOBALS
// **********

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

void loop() {
  // update timestamp
  elapsedtime = millis();
  float elapsedmin = elapsedtime/60000.0; // elapsed minutes
  
  // read the VBATT input on analog pin A7:
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
  float V_BATT = V_BATT_IN * (103e3 + 22.5e3) / 22.5e3;
  float V_5V = V_5V_IN * (104e3 + 47.2e3) / 47.2e3;
  float V_3V3 = V_3V3_IN * (102e3 + 47.3e3) / 47.3e3;
  
  // print to serial in .csv format
  // output can be copied from the serial monitor to create a .csv file
  Serial.print(elapsedmin);       // elapsed time in minutes
  Serial.print(",");
  Serial.print(V_BATT_IN);
  Serial.print(",");
  Serial.print(V_5V_IN);
  Serial.print(",");
  Serial.print(V_3V3_IN);
  Serial.print(",");
  Serial.print(V_BATT);
  Serial.print(",");
  Serial.print(V_5V);
  Serial.print(",");
  Serial.println(V_3V3);
  delay(60000);                         // Delay 1 minute before next loop
}
