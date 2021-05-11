// Modified "ReadAnalogVoltage" sketch in public domain: http://www.arduino.cc/en/Tutorial/ReadAnalogVoltage
// Modified by: James Lee
// Modify Date: 5/10/2021

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

void loop() {
  // Voltage monitoring loop performs the following:
      // 1. Reads digital integer number from ADC
      // 2. Converts to pin voltage based on 10 bits (3.3V / 2^10-1 bins)
      // 3. Offsets 0.04 based on no-load pin measurement with multimeter
      // 4. Scales pin voltage back to original voltage based on divider value
            // V = V_IN * (R_TOP + R_BOTTOM) / R_BOTTOM
      // 5. Print to serial
  Serial.print(millis()/1000);      // elapsed time in seconds
  Serial.print(",");  
  Serial.print((0.04 + analogRead(A7) * 3.3 / 1023.0) * (123e3 / 22e3)); // V_BATT
  Serial.print(",");
  Serial.print((0.04 + analogRead(A6) * 3.3 / 1023.0) * (133e3 / 33e3)); // V_5V
  Serial.print(",");
  Serial.println((0.04 + analogRead(A3) * 3.3 / 1023.0) * (147e3 / 47e3)); // V_3V3
  delay(5000);      // Adjust time delay in milliseconds for desired log frequency
}
