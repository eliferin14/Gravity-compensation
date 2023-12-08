/*
  Inertia estimation:
  Set the load to an angle and let it loose
  Measure the position evolution
  Estimate from that
*/

#include "AS5600.h"
#include "Wire.h"

// Arduino UNO: SCL:A5 SDA:A4
AS5600 encoder;

#define BUTTON 10

uint32_t Ts = 10000;
uint32_t t = 0;

void setup() {
  pinMode(BUTTON, INPUT);

  // Increase the baudrate to speed things up
  Serial.begin(115200);
  Serial.println("Program started");

  // From the library example 
  Wire.begin();
  encoder.begin(4);
  printf("AS5600 connect: %d", encoder.isConnected());

  // Wait for the button to be pressed
  Serial.println("Press the button to start the experiment");
  while(!digitalRead(BUTTON)); delay(500);

  // Measure until the button is pressed again
  while(!digitalRead(BUTTON)) {
    t = micros();

    // Just measure the position and print
    int y = encoder.readAngle();
    printf("%d; ", y);

    while( (micros()-t) < Ts );
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
