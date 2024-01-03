#include "INA219.h"
#include "Wire.h"

// L298N
#define INA 5
#define INB 6
#define EN 3
const int PWM_PIN = INA;

// Potentiometer
#define POT A1

// Control button
#define BUTTON 4

// Current sensor
INA219 ina(0x40);

uint32_t Ts = 5000;
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

// low pass filtering function
float Tf = 0.2;
float y_prev = 0;
unsigned long timestamp_prev = 0;

float filter(float input){
  unsigned long timestamp = micros();
  float dt = (timestamp - timestamp_prev)*1e-6f;
  // quick fix for strange cases (micros overflow)
  if (dt < 0.0f || dt > 0.5f) dt = 1e-3f;

  // calculate the filtering 
  float alpha = Tf/(Tf + dt);
  float y = alpha*y_prev + (1.0f - alpha)*input;

  // save the variables
  y_prev = y;
  timestamp_prev = timestamp;
  return y;
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
  if (!ina.begin()) Serial.println("INA connection problems!");
  ina.setMaxCurrentShunt(5, 0.002);

  // Wait for the button to be pressed
  Serial.println("Press the button to start the experiment");
  while(!digitalRead(BUTTON)); delay(1000);
  Serial.println("Starting the experiment");
}

void loop() {

  // Read the potentiometer and set the correct dutycycle
  int potValue = analogRead(POT);
  float dc = potValue / 1023.0;
  setPWM(dc);

  float current = ina.getCurrent();
  float filtered_current = filter(current);
  Serial.print("#"); Serial.print(dc); Serial.print("\t"); Serial.print(current); Serial.print("\t"); Serial.print(filtered_current); Serial.println();

  delay(Ts/1000);

}
