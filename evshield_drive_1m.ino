/*
Works on Lego Mindstorms model car two motor driven wheels.
Overview of motor commands (methods): http://www.mindsensors.com/reference/EVShield/html/class_e_v_shield_bank.html

Setup for this example:
  attach external power to EVShield (6x AA battery pack)
  attach driving motors to motor ports Bank A M1 and M2
  
Adjust:
	Change the WHEEL_DIAM to match the diameter of the wheels used in your model!
  
Open the Serial terminal to view.
	It shows the calculated values of some variables. Check these for your model.
 */
 
#include <Wire.h>
#include <EVShield.h>

#define WHEEL_DIAM 4.96 // wheel diameter in cm

EVShield evshield(0x34,0x36);

// prototype the function:
void driveDistance(unsigned int distance, SH_Direction direction = SH_Direction_Reverse, int speed = SH_Speed_Medium);

void setup() {
  Serial.begin(9600);       // start serial port output, check for same speed at Serial Monitor!
  
  evshield.init( SH_HardwareI2C );
  
  // reset motors
  evshield.bank_a.motorReset();

  Serial.println(F("Press Go to start..."));
  evshield.waitForButtonPress(BTN_GO);

  driveDistance(100); // distance to travel in cm 
}

void loop() {
}


void driveDistance(unsigned int distance, SH_Direction direction = SH_Direction_Reverse, int speed = SH_Speed_Medium) {
  Serial.print(F("Driving ")); Serial.print(distance); Serial.println(F("cm"));
  double circumference = WHEEL_DIAM * PI; // PI is-build in definition
  
  Serial.print(F("circumference=")); Serial.println(circumference);
  
  unsigned long degrees = (distance / circumference) * 360;
  
  Serial.print("degrees="); Serial.println(degrees);
  
  evshield.bank_a.motorStop(SH_Motor_Both, SH_Next_Action_Float); // stop
  evshield.bank_a.motorRunDegrees(SH_Motor_Both, direction, speed, degrees, SH_Completion_Wait_For, SH_Next_Action_BrakeHold );  // SH_Next_Action_Float SH_Next_Action_Brake SH_Next_Action_BrakeHold
}
