/*
  Resistance estimation
  Apply a voltage, turn on the driver to 100% and measure:
  - The voltage at the motor side
  - The current erogated by the psu

  Test for multiple voltages: Leave the program as it is and change the voltage of the supply

  R is calculated with linear regression

  A button is connected to turn on the driver
*/

#define INA 1
#define INB 2

#define BUTTON 10

void setup() {
  // put your setup code here, to run once:
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);

  // INB always on (or always off?), INA carries the PWM (in this case 0% or 100%)
  digitalWrite(INB, 1);

  pinMode(BUTTON, INPUT);

  Serial.begin(9600);
  Serial.println("Program started");
}

void loop() {
  // put your main code here, to run repeatedly:
  // Wait button press
  Serial.println("Press the button to turn on the driver");
  while(!digitalRead(BUTTON));

  // Send a step command (100% dutycycle)
  digitalWrite(INA, 1);

  delay(100);
  
  // Wait button press
  Serial.println("Press the button to turn off the driver");
  while(!digitalRead(BUTTON));

  // Stop the driver (0% dutycycle)
  digitalWrite(INA, 0);

  delay(100);
}
