#include "AS5600.h"
#include "Wire.h"

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

uint32_t Ts = 10000;
uint32_t startT = 0;

long map(float x, float in_min, float in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int setPWM(float dutycycle) {
  if ( dutycycle >= 0 ) {
    digitalWrite(INB, 0);
  }
  else {
    digitalWrite(INB, 1);
    dutycycle = 1+dutycycle;
  }

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

  // Build the input vector
  // buildInput();

  // Wait for the button to be pressed
  Serial.println("Press the button to start the experiment");
  while(!digitalRead(BUTTON)); delay(1000);
  Serial.println("Starting the experiment");

  while(true) {
    int potValue = analogRead(POT);
    float dc = potValue / 1023.0;
    setPWM(dc);
    Serial.print(dc); Serial.print("\t"); Serial.println(encoder.getAngularSpeed(AS5600_MODE_RADIANS));
    delay(Ts/1000);
  }

  float pwm_friction = 0.3;
  while(true) {
    setPWM(pwm_friction);
    delay(Ts/1000);
    Serial.println(encoder.getAngularSpeed(AS5600_MODE_RADIANS));
    setPWM(pwm_friction);
    delay(Ts/1000);
    Serial.println(encoder.getAngularSpeed(AS5600_MODE_RADIANS));
  }

  for (float dutycycle = -0.5; dutycycle < 0.5; dutycycle+=0.02) {
    setPWM(dutycycle);
    delay(1000);
    Serial.print(dutycycle); Serial.print(", ");

    int N = 100;
    float speed = 0;
    encoder.getAngularSpeed();

    while(!digitalRead(BUTTON)) {
      speed += 0.99 * ( encoder.getAngularSpeed(AS5600_MODE_RADIANS) - speed );
      delay(Ts/1000);
    }
    Serial.print(speed); Serial.print("; ");
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
