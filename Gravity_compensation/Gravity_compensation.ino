#include "AS5600.h"
#include "Wire.h"
#include "math.h"

// Arduino UNO i2c pins: SCL:A5 SDA:A4
// Remeber to connect the DIR pin of the encoder to 5V to set the direction to counter-clockwise
// If using the gearbox, the motor spins in the opposite direction wrt the load => connect DIR to GND
AS5600 encoder;
float motor_position, motor_position_cumulative;
float theta, theta_cumulative;

// L298N driver
#define INA 5
#define INB 6
#define EN 3
int PWM_PIN = INA;

// Control button
#define BUTTON 4

// Time variables [us]
uint32_t Ts = 5000;   // sampling time
uint32_t t_start = 0; // start time of the experiment

// Power supply parameters
#define V_PSU 11.7 // Before the driver [V]
#define DRIVER_DROP 1.8  // Voltage drop of the driver [V]
const float V_MOT_MAX = V_PSU - DRIVER_DROP;

// Motor parameters
#define R 2.8   // [Ohm]
#define L 0.0045  // [H]
#define kphi 0.0033 // [Vs]
#define J_rot 0.000022  // [kg m^2]

// Gearbox parameters
#define GEARBOX_RATIO 5.0 // The load spins 5 times slower than the motors

// Gravity compensation parameters
#define m 0.0032     // mass [kg]
#define l 0.23    // length [m]
#define g 9.81     // gravity acceleration [m/s^2]
float linear_density = m / l;   // [kg/m]
float d_gc = (l - 0.04) / 2 + 0.02; // distance of the center of mass of the part of the bar that produces torque [m]
float m_gc = linear_density * (l - 0.04); // mass " [kg]
float gcomp_magic_number = 0.0047;

// Given the angle compute the dutycycle that compensate the gravity torque
float gravityCompensation(float theta) {
  float V_gc = sin(theta) * gcomp_magic_number * R/kphi; // Voltage [V]
  return V_gc / V_MOT_MAX;                        // dutycycle in range [0,1]
}

// Controller variables
float error = 0;
float error_integral = 0;
float previous_error = 0;
float error_derivative = 0; 
float error_derivative_previous = 0;
float theta_ref = 0;
float step_amplitude = PI*3/4;
float dutycycle = 0;
int pwm = 0;

// Ziegler-Nichols ultimate gain and period
float ku = 10;
float Pu = 0.220;

// PID gains
float kp = 0.2 * ku;
float ki = 0.4 * ku / Pu;
float kd = 0.066 * ku * Pu;

// Overload of map() that uses floats
float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Take a float dutycycle in [-1,1], and "write" the correct configuration of the l298n driver
// NB: Saturation sould be detected before calling this function!!!
#define FORWARD_BRAKE
//#define FB_SWITCH_PINS
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
    dutycycle = 1+dutycycle;  // We are in forward-brake!
  }

  // Compute the int value of the pwm in the range [0,255]
  int pwm = map( abs(dutycycle), 0.0, 1.0, 0, 255);
  analogWrite(PWM_PIN, pwm);

  return pwm;
}

// Return the angle in radians in a range [-pi,pi], where 0 is the downward position
// We are measuring the motor's position, but we want to control the load => we need to take into account the gearbox
float readTheta() {
  // Measure the cumulative position of the motor
  motor_position_cumulative = (encoder.getCumulativePosition() * AS5600_RAW_TO_RADIANS);

  // Cumulative position of the load
  theta_cumulative = motor_position_cumulative / GEARBOX_RATIO; 

  // Remove complete revolutions from the measurement
  float theta = fmod(theta_cumulative, 2*PI); // in [-2*PI, 2*PI]

  // We want the measurement to be in the [-PI, PI] range
  if (theta > PI) theta -= 2*PI;  
  else if (theta < -PI) theta += 2*PI;

  return theta;
}

// Return the sign of a number
float sign(float x) {
  return x / abs(x);
}

// Low pass filter
// r is the smoothing factor: r=1 -> no smoothing; r~=0 -> big smoothing
float lowPass(float x, float yPrev, float r) {
  float y = (r-1)*yPrev + r*x;
  return y;
}

// Given the error, compute the friction compensation term
// If we use a simple sign(error) * friction_dutycycle, the input oscillates too much and the br oscillates a lot, so we need to smooth it
float friction_dutycycle = 0.21; // Voltage required to overcome static friction
float error_threshold = 0.05;  // [rad]

float frictionCompensation(float error) {
  float dutycycle = 0;

  if ( abs(error) > error_threshold) {
    dutycycle = sign(error) * friction_dutycycle;
  }
  else {
    dutycycle = error / error_threshold * friction_dutycycle;
  }

  return dutycycle;
}

//=========================================================================================================
void setup() {
  // Driver setup
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(EN, OUTPUT);
  digitalWrite(INB, 0);
  digitalWrite(INA, 0);
  digitalWrite(EN, HIGH);

  // Control button setup
  pinMode(BUTTON, INPUT);

  // Set the baudrate of the serial connection
  Serial.begin(250000);
  Serial.println("\n===================================\nProgram started");

  // Encoder setup
  Wire.begin();
  Wire.setClock(800000);
  encoder.begin(2);
  Serial.print("AS5600 connect: "); Serial.println(encoder.isConnected());

    // Set the offset for the encoder. The rod should be oriented downward!
    encoder.resetCumulativePosition();

  // Wait for the button to be pressed
  Serial.println("Press the button to start the experiment");
  while(!digitalRead(BUTTON)); delay(1000);
  Serial.println("Starting the experiment");

  // Initialize error to avoid initial spike in the derivative
  error = readTheta() - theta_ref;

  // Starting time
  t_start = micros();
}

void loop() {
  // Update time variables
  uint32_t loop_start = micros();
  float t = (loop_start - t_start) / 1000000.0; // Time past the start of the experiment, in seconds

  // Read the position of the bar
  theta = readTheta();

  // If the button is pressed, refresh the reference to be the current position
  // This basically disables the controller
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

    // Integral with anti-windup
    if (abs(dutycycle) <= 0.99 && abs(error) < 0.5) error_integral += error * (Ts/1000000.0);

    // Compute the discrete derivative of the error
    float error_delta = error - previous_error;
    if ( abs(error_delta) < PI ) error_derivative = (error_delta) / (Ts/1000000.0);
    // Low pass filter applied to derivative
    error_derivative = lowPass(error_derivative, error_derivative_previous, 0.8);
    error_derivative_previous = error_derivative;

    // Compute the control dutycycle
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

  // Change direction because we have the gearbox (positive dutycycle -> positive torque)
  dutycycle *= -1;

  // Set the pwm
  pwm = setPWM(dutycycle);

  // Log the data
  Serial.print("#"); 
  Serial.print( theta, 3 ); Serial.print("\t"); Serial.print( theta_ref, 3 ); Serial.print("\t"); Serial.print(kp* error, 3 ); Serial.print("\t"); Serial.print( ki* error_integral, 3 ); Serial.print("\t"); Serial.print(kd* error_derivative, 3 ); Serial.print("\t"); Serial.print(dutycycle, 3); Serial.print("\t"); Serial.print("\t"); Serial.print( g_comp, 3 ); Serial.print("\t"); Serial.print( f_comp, 3 ); Serial.print("\t"); Serial.print( t, 3 ); 
  Serial.println();

  // Wait 
  while( micros()-loop_start < Ts );  

  //theta_ref = t<1 ? 0 : step_amplitude;
  //if (t > 5) { Serial.println("Experiment terminated"); setPWM(0); delay(500); exit(0); }
}
