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

// L298N
#define INA 5
#define INB 6
#define EN 3
const int PWM_PIN = INA;

#define BUTTON 4

// Experiment parameters
#define DATASET_SIZE 1000 // Number of samples to collect
uint32_t Ts = 10000;      // Sampling time [us]
uint32_t t_start = 0;           // time at the stat of each iteration

// Function to generate the step input
float stepInput(float t_float, float t_start) {
  if (t_float < t_start) {
    return 0;
  }
  else {
    return 1;
  }
}

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
  digitalWrite(INB, 0);
  digitalWrite(INA, 0);

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

  // Step response of the system
  encoder.getAngularSpeed();  // Initialize the sampling 

  // Collect all the samples
  for (int sample=0; sample<DATASET_SIZE; sample++) {
    // Start the time
    t_start = micros();

    // Measure spip
    float y = encoder.getAngularSpeed(AS5600_MODE_RADIANS);

    // Compute the input to apply
    float t_float = sample * Ts / 1000000.0;  // "exact" time
    float u_float = stepInput(t_float, 1.0);

    // Convert the dutycycle from float to the [0,255] range and apply
    int u = setPWM(u_float);

    // Print the data
    Serial.print(t_float,3); Serial.print(", "); Serial.print(u_float,3);    Serial.print(", "); Serial.print(y,3);    Serial.print("; ");

    // Check if the sampling time is too short
    if ( (micros() - t_start) > Ts ) {
      Serial.println("\n\nSampling time too short\n");
      break;
    }

    // Wait for the sampling time interval to finish
    while ( (micros() - t_start) < Ts );
    
  }

  Serial.println("\nExperiment completed");
  setPWM(0);

}

void loop() {
  // put your main code here, to run repeatedly:

}
