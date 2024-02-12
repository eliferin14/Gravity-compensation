/*
  This program is used to estimate the static friction of the motor

  There are two possibilities:
  - manual mode: use a potentiometer to set the pwm signal and check when the motor starts spinning
  - automatic mode: sweep an array of pwm values and measure speed (with arduino) and current (with a multimeter)
*/

#include "AS5600.h"
#include "Wire.h"

// Only one should be active at once
#define MANUAL_MODE
//#define AUTO_MODE

// Arduino UNO: SCL:A5 SDA:A4
AS5600 encoder;

// L298N
#define INA 5
#define INB 6
#define EN 3
const int PWM_PIN = INA;

// Potentiometer
#define POT A1

// Control button
#define BUTTON 4

uint32_t Ts = 10000;  // us
uint32_t t_start = 0;

// Overload of map() that accept float as input
long map(float x, float in_min, float in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Take a float dutycycle in [-1,1], and "write" the correct configuration of the l298n driver
// NB: Saturation sould be detected before calling this function!!!
int setPWM(float dutycycle) {
  // Throw an error if saturation is detected. The saturation must be done outside!
  if (abs(dutycycle) > 1) {
    Serial.println("Saturation detected!");
    setPWM(0);
    exit(0);
  }

  // Set the correct direction
  if ( dutycycle >= 0 ) {
    digitalWrite(INB, 0);
  }
  else {
    digitalWrite(INB, 1);
    dutycycle = 1+dutycycle;  // We are in forward brake!
  }

  // Compute the int value of the pwm in the range [0,255]
  int pwm = map(dutycycle, 0, 1, 0, 255);
  analogWrite(PWM_PIN, pwm);

  return pwm;
}

void setup() {
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(EN, OUTPUT);

  pinMode(POT, INPUT);

  // Change this depending on the configuration
  digitalWrite(INB, 0);
  digitalWrite(INA, 0);
  digitalWrite(EN, 1);

  pinMode(BUTTON, INPUT);

  // Increase the baudrate to speed things up
  Serial.begin(115200);
  Serial.println("\n===================================\nProgram started");

  // From the library example 
  Wire.begin();
  Wire.setClock(800000);
  encoder.begin(2);
  //encoder.setDirection(AS5600_CLOCK_WISE);
  Serial.print("AS5600 connect: "); Serial.println(encoder.isConnected());

  // Wait for the button to be pressed
  Serial.println("Press the button to start the experiment");
  while(!digitalRead(BUTTON)); delay(1000);
  Serial.println("Starting the experiment");

  #ifdef MANUAL_MODE
    while(true) {
      // Read the potentiometer and set the correct dutycycle
      int potValue = analogRead(POT);
      float dc = potValue / 1023.0;
      setPWM(dc);

      Serial.print(dc); Serial.print("\t"); Serial.println(encoder.getAngularSpeed(AS5600_MODE_RADIANS));

      delay(Ts/1000);
    }
  #endif

  #ifdef AUTO_MODE
    // Sweep a set of dutycycle values
    // For each value, measure speed and current
    Serial.print("[ ");
    for (float dutycycle = 0.12; dutycycle < 0.30; dutycycle+=0.01) {
      setPWM(dutycycle);
      delay(1000);  // Skip the transient

      // Initialize speed
      float speed = 0;
      encoder.getAngularSpeed();

      // Wait for the motor to be at a stable speed. When it is, press the button to print the data and go to the next value
      while(!digitalRead(BUTTON)) {
        speed += 0.999 * ( encoder.getAngularSpeed(AS5600_MODE_RADIANS) - speed ); // Mean with forgetting factor
        delay(Ts/1000);
      }
      
      Serial.print(dutycycle); Serial.print(", "); Serial.print(speed); Serial.println("; ");
    }
    Serial.println("]");
  #endif

}

void loop() {
  // put your main code here, to run repeatedly:

}
