/* 
Voltage Monitoring (simplified)
HPS Autopilot 2021: https://github.com/UW-BAH-HPS-Autopilot-2021/Arduino-MCU
Based on Rev1 of Custom PCB: https://oshwlab.com/HPS-Autopilot-2021/hps-autopilot-power-supply
References:
https://www.arduino.cc/en/Tutorial/BuiltInExamples/ReadAnalogVoltage
Original sketch created 5/10/2021 by James Lee
Last modified 5/31/2021 by James Lee
*/

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

void loop() {
  /*  Voltage monitoring loop performs the following:
   *  1. Reads digital integer number from ADC
   *  2. Converts to pin voltage based on 10 bits (3.3V / 2^10-1 bins)
   *  3. Scales pin voltage back to original voltage based on divider value:
   *     V = (Pin_int * 3.3V / (2^n_bits-1) ) * (R_TOP + R_BOTTOM) / R_BOTTOM
   *  4. Print to serial 
   */
  Serial.print(millis()/1000);      // elapsed time in seconds
  Serial.print(",");  
  Serial.print((analogRead(A7) * 3.3 / 1023.0) * (123e3 / 22e3)); // V_BATT
  Serial.print(",");
  Serial.print((analogRead(A6) * 3.3 / 1023.0) * (133e3 / 33e3)); // V_5V
  Serial.print(",");
  Serial.println((analogRead(A3) * 3.3 / 1023.0) * (147e3 / 47e3)); // V_3V3
  delay(5000);      // Adjust time delay in milliseconds for desired log frequency
}
