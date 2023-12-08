/*
  This program is used to identify the initial angle reading of the encoder
  Set the bar to downward position
  Start the program
  Memorize the offset and use it in the control program
*/

#include "AS5600.h"
#include "Wire.h"

// Arduino UNO: SCL:A5 SDA:A4
AS5600 encoder;

void setup() {
  // put your setup code here, to run once:

  // Increase the baudrate to speed things up
  Serial.begin(115200);
  Serial.println("\n===================================\nProgram started");

  // From the library example 
  Wire.begin();
  Wire.setClock(800000);
  encoder.begin(2);
  encoder.setDirection(AS5600_CLOCK_WISE);
  Serial.print("AS5600 connect: "); Serial.println(encoder.isConnected());

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(encoder.readAngle() * AS5600_RAW_TO_DEGREES);
}
