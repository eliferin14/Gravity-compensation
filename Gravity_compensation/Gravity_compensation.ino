#include "AS5600.h"
#include "Wire.h"

// Arduino UNO: SCL:A5 SDA:A4
AS5600 encoder;

// L298N
#define INA 5
#define INB 6
#define EN 3
const int PWM_PIN = INA;

// Control button
#define BUTTON 4

// Sampling time [us]
uint32_t Ts = 5000;  

// Encoder offset: the raw position when the bar is downward (our theta=0 position)
// Use the "offset.ino" program to get this value
float offset = 153;

// Power parameters
#define V_PSU 11.7 // Before the driver [V]
#define DRIVER_DROP 1.4  // Voltage drop of the driver [V]
const float V_MOT_MAX = V_PSU - DRIVER_DROP;

// Motor parameters
#define R 2.8   // [Ohm]
#define L 0.0045  // [H]
#define kphi 0.0033 // [Vs]
#define J_rot 0.000022  // [kg m^2]

// Gravity compensation parameters
#define m 0.0032     // mass [kg]
#define l 0.23    // length [m]
#define g 9.81     // gravity acceleration [m/s^2]
float linear_density = m / l;   // [kg/m]
float d_gc = (l - 0.04) / 2 + 0.02; // distance of the center of mass of the part of the bar that produces torque [m]
float m_gc = linear_density * (l - 0.04); // mass " [kg]

// Given the angle compute the dutycycle that compensate the gravity torque
float gravityCompensation(float theta) {
  float V_gc = sin(theta) * m_gc*d_gc*g * R/kphi; // Voltage [V]
  return V_gc / V_MOT_MAX;                        // dutycycle in range [0,1]
}

// Controller variables
float error = 0;
float error_integral = 0;
float previous_error = 0;
float error_derivative = 0;
float theta_ref = PI/2;
float dutycycle = 0;
int pwm = 0;

// PID gains
float kp = 0.39;
float ki = 0.7;
float kd = 0.05;

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

// Return the angle in radians in a range [-pi,pi], where 0 is the downward position
float readTheta() {
  float theta = encoder.readAngle() * AS5600_RAW_TO_RADIANS; // [0, 2*pi]
  if (theta > PI) theta = theta - 2*PI;                      // [-pi, pi]
  return theta;
}

// Return the sign of a number
float sign(float x) {
  return x / abs(x);
}

// Given the error, compute the friction compensation term
// If we use a simple sign(error) * friction_dutycycle, the input oscillates too much and the br oscillates a lot, so we need to smooth it
// Linear seems to work well

//#define THRESHOLD
#define LINEAR
//#define HYSTERESYS
float friction_dutycycle = 0.18; // Voltage required to overcome static friction
float error_threshold = 0.05;  // [rad]

float frictionCompensation(float error) {
  float dutycycle = 0;

  #ifdef THRESHOLD
    if (abs(error) > error_threshold) {
      dutycycle = sign(error) * friction_dutycycle;
    }
  #endif

  #ifdef LINEAR
    if ( abs(error) > error_threshold) {
      dutycycle = sign(error) * friction_dutycycle;
    }
    else {
      dutycycle = error / error_threshold * friction_dutycycle;
    }
  #endif

  return dutycycle;
}

void setup() {
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(EN, OUTPUT);

  // Change this depending on the configuration
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

  // Set the offset
  encoder.setOffset(-offset);

  // Wait for the button to be pressed
  Serial.println("Press the button to start the experiment");
  while(!digitalRead(BUTTON)); delay(1000);
  Serial.println("Starting the experiment");

  // Initialize error to avoid initial spike in the derivative
  error = readTheta() - theta_ref;
}

void loop() {
  uint32_t start_time = micros();

  // Read the position of the bar
  float theta = readTheta();

  // If the button is pressed, refresh the reference to be the current position
  if (digitalRead(BUTTON)) {
    theta_ref = theta;
    error_integral = 0;
  }

  // PID controller

    // Save the previous value of the error
    previous_error = error;

    // Compute the new error
    error = theta_ref - theta;
    if (error > PI) {
      error -= 2*PI;
    }
    else if (error < -PI) {
      error += 2*PI;
    }

    // If not saturating, increment the integral ("anti-windup")
    if (abs(dutycycle) <= 0.99) error_integral += error * Ts/1000000.0;

    // Compute the discrete derivative of the error
    float error_delta = error - previous_error;
    if ( abs(error_delta) < PI ) error_derivative = (error_delta) / (Ts/1000000.0);

    // Compute the dutycycle
    dutycycle = kp*error + ki*error_integral + kd*error_derivative;

  // Gravity compensation
  float g_comp = gravityCompensation(theta);
  dutycycle += g_comp;

  // Static friction compensation
  float f_comp = frictionCompensation(error);
  dutycycle += f_comp;

  // dutycycle saturation correction
  if (dutycycle > 1) dutycycle = 1;
  if (dutycycle < -1) dutycycle = -1;

  // Set the pwm
  pwm = setPWM(dutycycle);

  // Print the data
  Serial.print("#"); Serial.print( theta, 3 ); Serial.print("\t"); Serial.print( error, 3 ); Serial.print("\t"); Serial.print( error_integral, 3 ); Serial.print("\t"); Serial.print( error_derivative, 3 ); Serial.print("\t"); Serial.print(dutycycle, 3); Serial.print("\t"); Serial.print( pwm ); Serial.print("\t"); Serial.print( g_comp, 3 ); Serial.print("\t"); Serial.print( f_comp, 3 ); Serial.println();

  if ( micros()-start_time > Ts ) {
    Serial.println("Sampling time too short");
    exit(0);
  }
  while( micros()-start_time < Ts );  
}
