/*
  Flux density constant estimation

  Let the motor spin!

  We do basically the same thing as we did in the resistance estimation, but here we collect the angular velocity aswell
*/

#include "AS5600.h"
#include "Wire.h"

// Arduino UNO: SCL:A5 SDA:A4
AS5600 encoder;

#define INA 5
#define INB 6

#define BUTTON 4

  // Measure average speed:
  float speed_avg = 0;
  int n = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);

  // INB always on (or always off?), INA carries the PWM (in this case 0% or 100%)
  digitalWrite(INB, 1);
  digitalWrite(INA, 0);

  pinMode(BUTTON, INPUT);

  Serial.begin(9600);
  Serial.println("\nProgram started");

  // From the library example 
  Wire.begin();
  encoder.begin(4);
  //printf("AS5600 connect: %d", encoder.isConnected());
  Serial.println(encoder.isConnected());

  

  delay(3000);

}

void loop() {
  // put your main code here, to run repeatedly:
// Recursive formulation for the average: https://math.stackexchange.com/questions/2845793/recursive-mean-computation
    float speed = encoder.getAngularSpeed(AS5600_MODE_RADIANS);
    n++;
    if (n==1) {
      speed_avg = speed;
    }
    speed_avg += 1.0/n * (speed - speed_avg);

    // Print to monitor
    Serial.print(speed); Serial.print("\t"); Serial.println(speed_avg);
    //printf("n = %d, \taverage speed = %.2f", n, speed_avg);
    delay(10);
}
