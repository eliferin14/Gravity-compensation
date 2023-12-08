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

// Encoder offset: te position when the bar is downward (our theta=0 position)
float offset = 26.0;

// Power parameters
#define V_PSU 11.7 // Before the driver [V]
#define DRIVER_DROP 1.8  // Voltage drop of the driver [V]
const float V_MOT_MAX = V_PSU - DRIVER_DROP;

// Motor parameters
#define R 2.8   // [Ohm]
#define L 0.0045  // [H]
#define kphi 0.0033 // [Vs]
#define J_rot 0.000022  // [kg m^2]

// Gravity compensation parameters
#define m 0.01     // mass [kg]
#define l 0.23    // length [m]
#define g 9.81     // gravity acceleration [m/s^2]
float linear_density = m / l;   // [kg/m]
float d_gc = (l - 0.04) / 2 + 0.02; // distance of the center of mass of the part of the bar that produces torque [m]
float m_gc = linear_density * (l - 0.04); // mass " [kg]

// Given the angle compute the dutycycle that compensate the gravity torque
float gravityCompensation(float theta) {
  float V_gc = sin(theta) * m_gc*d_gc*g * R/kphi;
  return V_gc / V_MOT_MAX;
}

// Controller parameters
float kp = 0.1;
float ki = 0.15;
float kd = 0.05;
float error = 0;
float error_integral = 0;
float previous_error = 0;
float error_derivative = 0;
float theta_ref = PI/2;

// Copied from the internet
long map(float x, float in_min, float in_max, long out_min, long out_max) {
  //if ( x > in_max ) x = in_max;
  //if ( x < in_min ) x = in_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Convert the float dutycycle [-1,1] to an int [0,255] and take into consideration the direction
int setPWM(float dutycycle) {
  if ( dutycycle >= 0 ) {
    digitalWrite(INB, 0);
  }
  else {
    digitalWrite(INB, 1);
    dutycycle = 1+dutycycle;
  }

  int pwm = map(dutycycle, 0.0, 1.0, 0, 255);
  analogWrite(INA, pwm);

  return pwm;
}

// Return the angle in radians in a range [-pi,pi], where 0 is the downward position
float readTheta() {
  float theta = encoder.readAngle() * AS5600_RAW_TO_RADIANS;
  theta = PI - theta;
  return theta;
}

float sign(float x) {
  return x / abs(x);
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

  // Print the initial position and set the offset
  //offset = encoder.readAngle() * AS5600_RAW_TO_DEGREES;
  encoder.setOffset(-offset + 180);

  // Wait for the button to be pressed
  Serial.println("Press the button to start the experiment");
  while(!digitalRead(BUTTON)); delay(1000);
  Serial.println("Starting the experiment");
}

int pwm = 0;
void loop() {
  uint32_t start_time = micros();

  // Read the position of the bar
  float theta = readTheta();

  // PID controller
  previous_error - error;
  error = theta_ref - theta;
  if (pwm < 255) error_integral += error * Ts/1000000.0;
  error_derivative = (error - previous_error) / (Ts/1000000.0);
  float dutycycle = kp*error + ki*error_integral + kd*error_derivative;

  // Gravity compensation
  //dutycycle += gravityCompensation(theta);
  dutycycle += gravityCompensation(theta_ref);

  // Static friction compensation
  //dutycycle += 0.3 * sign(error);

  // dutycycle saturation
  if (dutycycle > 1) dutycycle = 1;
  if (dutycycle < -1) dutycycle = -1;

  pwm = setPWM(dutycycle);
  Serial.print( theta, 3 ); Serial.print(" "); Serial.print( error, 3 ); Serial.print(" ");Serial.print(dutycycle, 3); Serial.print(" "); Serial.print(pwm); Serial.println();

  if ( micros()-start_time > Ts ) {
    Serial.println("Sampling time too short");
    exit(0);
  }
  while( micros()-start_time < Ts );  
}
