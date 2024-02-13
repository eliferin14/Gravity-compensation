#include "AS5600.h"
#include "Wire.h"
#include "math.h"

// Arduino UNO i2c pins: SCL:A5 SDA:A4
// Remeber to connect the DIR pin of the encoder to 5V to set the direction to counter-clockwise
// If using the gearbox, the motor spins in the opposite direction wrt the load => connect DIR to GND
AS5600 encoder;
float motor_position, motor_position_cumulative;
float theta, theta_cumulative;

// L298N
#define INA 5
#define INB 6
#define EN 3
int PWM_PIN = INA;

// Control button
#define BUTTON 4

// Potentiometer
#define POT A1

// Sampling time [us]
uint32_t Ts = 10000;  
uint32_t t0;

// Encoder offset: the raw position when the bar is downward (our theta=0 position)
// Use the "offset.ino" program to get this value in degrees
float offset_degrees = 143.53;
float offset_radians = offset_degrees * 2*PI / 360;

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
#define GEARBOX_EFFICIENCY 1.0

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
  float x = sin(theta);// + 0.2*cos(2*theta);
  float V_gc = x * gcomp_magic_number * R/kphi; // Voltage [V]
  return V_gc / V_MOT_MAX;                        // dutycycle in range [0,1]
}

// Controller variables
float error = 0;
float error_integral = 0;
float previous_error = 0;
float error_derivative = 0; float error_derivative_previous = 0;
float theta_ref = -PI*3/4;
float dutycycle = 0; 
float u;
int pwm = 0;

// Overload of map() that accept float as input
long map(float x, float in_min, float in_max, long out_min, long out_max) {
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

  #ifdef FORWARD_BRAKE

    #ifdef FB_SWITCH_PINS
      digitalWrite(EN, 1);
      // Set the correct direction
      if ( dutycycle >= 0 ) {
        PWM_PIN = INA;
        digitalWrite(INB, 0);
      }
      else {
        PWM_PIN = INB;
        digitalWrite(INA, 0);
      }
    #endif

    #ifndef FB_SWITCH_PINS
      digitalWrite(EN, 1);
      // Set the correct direction
      if ( dutycycle >= 0 ) {
        digitalWrite(INB, 0);
      }
      else {
        digitalWrite(INB, 1);
        dutycycle = 1+dutycycle;  // We are in forward brake!
      }
    #endif

  #endif

  #ifndef FORWARD_BRAKE
    PWM_PIN = EN;
    if (dutycycle >= 0) {
      digitalWrite(INA, 1);
      digitalWrite(INB, 0);
    }
    else {
      digitalWrite(INA, 0);
      digitalWrite(INB, 1);
    }
  #endif

  // Compute the int value of the pwm in the range [0,255]
  int pwm = map( abs(dutycycle), 0.0, 1.0, 0, 255);
  analogWrite(PWM_PIN, pwm);

  return pwm;
}

// Return the angle in radians in a range [-pi,pi], where 0 is the downward position
// We are measuring the motor's position, but we want to control the load => we need to take into account the gearbox
float readTheta() {
  // Measure the cumulative position of the motor, taking into account the offset
  motor_position_cumulative = (encoder.getCumulativePosition() * AS5600_RAW_TO_RADIANS);

  // Cumulative position of the load
  theta_cumulative = motor_position_cumulative / GEARBOX_RATIO; 

  // Remove complete revolutions from the measurement
  float theta = fmod(theta_cumulative, 2*PI); // in [-2*PI, 2*PI]

  // We want the measurement to be in the [-PI, PI] range
  //if (theta > PI) theta -= 2*PI;  
  //else if (theta < -PI) theta += 2*PI;

  return theta;
}

// Return the sign of a number
float sign(float x) {
  return x / abs(x);
}

void setup() {
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(POT, INPUT);

  // Change this depending on the configuration
  digitalWrite(INB, 0);
  digitalWrite(INA, 0);

  pinMode(BUTTON, INPUT);

  // Increase the baudrate to speed things up
  Serial.begin(250000);
  Serial.println("\n===================================\nProgram started");

  // From the library example 
  Wire.begin();
  Wire.setClock(800000);
  encoder.begin(2);
  Serial.print("AS5600 connect: "); Serial.println(encoder.isConnected());

  // Set the offset
  encoder.setOffset(-offset_degrees);
  encoder.resetCumulativePosition();

  // Wait for the button to be pressed
  Serial.println("Press the button to start the experiment");
  while(!digitalRead(BUTTON)); delay(1000);
  Serial.println("Starting the experiment");

  // Initialize error to avoid initial spike in the derivative
  error = readTheta() - theta_ref;

  // Initialize time
  t0 = micros();

  Serial.print("data = [ ");
  while( micros() - t0 < 10000000) {  // Loop for 10 seconds
    uint32_t t_loop = micros();
    uint32_t t = t_loop - t0;
    float t_seconds = t / 1000000.0;

    // Read the position of the bar
    theta = readTheta();

    // Log
    Serial.print(t_seconds); Serial.print(", "); Serial.print(u); Serial.print(", "); Serial.print(dutycycle); Serial.print(", "); Serial.print(theta); Serial.print("; "); 
    
    // Compute the voltage to apply
    float f = 0.5;
    u = 0.3 ;//* ( sin(2*PI*f * t_seconds) + sin(2.3*2*PI*f * t_seconds) );
    dutycycle = u + gravityCompensation(theta);

    // Apply the voltage
    if ( dutycycle > 1 ) dutycycle = 1;
    if ( dutycycle < -1 ) dutycycle = -1;
    setPWM(-1*dutycycle);

    // Wait
    while (micros() - t_loop < Ts);
  }
  Serial.println(" ]");
  Serial.println("Experiment finished!");
  setPWM(0);
}

void loop() {
  
    
}

