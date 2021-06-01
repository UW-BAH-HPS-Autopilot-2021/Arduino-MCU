/* 
Voltage Monitoring Sketch with Serial Export
HPS Autopilot 2021: https://github.com/UW-BAH-HPS-Autopilot-2021/Arduino-MCU
Based on Rev1 of Custom PCB design: https://oshwlab.com/HPS-Autopilot-2021/hps-autopilot-power-supply
References:
https://www.arduino.cc/en/Tutorial/BuiltInExamples/ReadAnalogVoltage
https://toptechboy.com/python-with-arduino-lesson-11-plotting-and-graphing-live-data-from-arduino-with-matplotlib/
Original sketch created 5/31/2021 by James Lee
Last modified 5/31/2021 by James Lee
*/

///////////
// Includes
///////////
#include <iostream>
#include <string>

///////////////////
// Global variables
///////////////////
bool processorReady = false;  // Check for incoming processor flag
String test;                  // Test string incoming from processor program
float start;                  // Operating start time (after startup process)
int baudrate = 115200;        // Verify same baudrate in processor program
int logCycle = 300;           // milliseconds between each log point
float runtime_ms = 0;         // Operating runtime in milliseconds
float runtime_s = 0;          // Operating runtime in seconds
String V_BATT;                // Analog battery voltage decoded
String V_5V;                  // Analog 5V rail decoded
String V_3V3;                 // Analog 3.3V rail decoded

///////////////////
// Helper Functions
///////////////////

void readVoltage() {
  runtime_s = runtime_ms/1000;
  // Voltage = (pin_int*3.3V/(2^n_bits-1)) * (R_total / R_bottom)
  V_BATT = String((analogRead(A7) * 3.3 / 1023.0) * (123e3 / 22e3)) + ",";
  V_5V = String((analogRead(A6) * 3.3 / 1023.0) * (133e3 / 33e3)) + ",";
  V_3V3 = String((analogRead(A3) * 3.3 / 1023.0) * (147e3 / 47e3));
  Serial.println(String(runtime_s) + "," + V_BATT + V_5V + V_3V3);
  Serial.flush();       // Wait until all serial writing complete before opening port back up
  delay(logCycle);      // set delay between each logging point
}

////////
// Setup
////////
void setup() {
  Serial.begin(baudrate);
  Serial.setTimeout(1);
  // Setup handshake with python processing program to verify startup
  while (!processorReady) {
    if (Serial.available()) {
      test = Serial.readString();
      if (test == "Ready") {
        processorReady = true;
        Serial.print("Startup pass. Starting Voltage Monitor...");
        delay(2000);
      }
    }
  }
  start = millis();
}

/////////////////
// Operating loop
/////////////////
void loop() {
    readVoltage();
    runtime_ms = millis() - start;
}