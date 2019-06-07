/*
  Checkout the building guide:
  https://home.et.utwente.nl/slootenvanf/2019/04/04/lego-rover-car/

  This is a simplified version of the sketch, without use of  class.
  This sketch has been improved with differentialDrive by Thimo Willems.

  This sketch uses the library "EVShield"
  Check if these are present in Documents\Arduino\libraries. If not, install them:
  https://home.et.utwente.nl/slootenvanf/wp-content/uploads/appdev-download/Installation_instructions.html#arduino

  Connect USB cable then Upload this sketch.
  Open the Serial Monitor to view test output. Make sure the speed in the Serial Monitor is set to 9600.
  If communicating with another app (to recieve instructions), do not open the Serial Monitor!

  If the car is driving in the wrong direction, find occurrences of calls to differentialDrive(...) and
  swap the parameters SH_Direction_Reverse with SH_Direction_Forward and vice versa.
  Also, in function differentialDrive(), swap variables ratio_L and ratio_R at the last two lines.

  If using the EV3 version of the touch-sensor, make sure you search-and-replace all occurences of EVs_NXTTouch with EVs_EV3Touch
*/

#include <EVShield.h>
#include <EVs_NXTTouch.h>
#include <NewPing.h>

#define TRIGGER_PIN  3  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     5  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

EVShield evshield(0x34, 0x36);
EVs_NXTTouch touch;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance

String instructions = "";

SH_Motor steerMotor = SH_Motor_1; // motor (M1) on Bank B used for steering front wheels
SH_Motor sensorMotor = SH_Motor_2; // motor (M2) on Bank B on which the ultrasonic sensor is mounted

// speed and driving:
int start_speed = 30;  // start speed (speed can be any value between 0-100)
int speed = start_speed;
boolean dr_forward = false, dr_backward = false; // moving in forward or backward direction

// car dimension:
unsigned int car_rear_track = 145; // car's rear track, the distance between the centerline of each rear wheel (in millimeters)
unsigned int car_wheelbase = 185; // car's wheelbase, the distance between the center of the front wheels and the rear wheels (in millimeters)
float car_wheel_diam = 4.96; // car's wheel diameter in cm (wheels attached to motors)

// forward declaration of functions with parameters with default values
void differentialDrive(SH_Direction dir, int degr = 0);
void drive(int distance = 0);
void steer(int degr = 12);


void setup() {
  // Open serial communication:
  Serial.begin(9600);

  Serial.println(F("Use motors on Bank A to drive."));
  Serial.println(F("Press GO after instructions were recieved."));

  evshield.init( SH_HardwareI2C );
  touch.init( &evshield, SH_BAS1);
  // reset motors
  evshield.bank_a.motorReset();
  delay(100);
  evshield.bank_b.motorReset();
  delay(100);
  evshield.bank_b.motorResetEncoder( steerMotor ); // Reset the current encoder position to zero for the steering motor
  delay(100);

  unsigned int voltage = evshield.bank_a.evshieldGetBatteryVoltage();
  Serial.print(F("Battery voltage: ")); Serial.print( voltage ); Serial.println(F(" mV (on batteries, should be above 6000)"));

  if (voltage > 6500)
    evshield.bank_b.ledSetRGB(0, 255, 0); // led green, battery Ok, ready for driving
  else if (voltage > 5500)
    evshield.bank_b.ledSetRGB(160, 160, 20); // led orange, battery might be low, ready for driving
  else
    evshield.bank_b.ledSetRGB(255, 0, 0); // led red, battery low, problems might occur driving motors
}

void loop() {
  // listen for commands via Serial Connection:
  while (Serial.available()) {
    instructions = Serial.readString(); // read the incoming data as a string
    Serial.print(F("Recieved: ")); Serial.println(instructions);
  }

  // handle buttons of evshield
  if ( evshield.getButtonState(BTN_LEFT) ) { // change driving direction
    Serial.println(F("LEFT"));
    //reverseDirection();
    turn(90);
  }
  else if ( evshield.getButtonState(BTN_RIGHT) ) { // drive 1m
    Serial.println(F("RIGHT"));
    // stop, then drive 1m:
    stop();
    drive(100);
  }
  else if ( evshield.getButtonState(BTN_GO) ) { // start/stop
    Serial.println(F("GO"));
    dr_forward = !dr_forward;
    if (dr_forward) {
      if (instructions.length() > 0) { // if there are instructions, execute them
        readCommand();
        // make sure the wheels are straight after execution of commands
        straight();
      }
      else // otherwhise, just drive:
        drive();
    }
    else {
      stop();
    }
  }

  // handle sensors while driving
  if (dr_forward) {
    if (touch.isPressed()) {
      Serial.println(F("bump"));
      // call method to reverse & turn
      reverseTurn();
    }
    else {
      unsigned int distance = sonar.ping_cm();
      if (distance > 0 && distance < 30) {
        find_a_way_out();
      }
    }
  }

  delay(100);
}


void increase_speed() {
  speed = speed + 5; // increase speed
  delay(300);
  if (speed > SH_Speed_Full) speed = SH_Speed_Full;
}

void decrease_speed() {
  speed = speed - 5; // increase speed
  if (speed < start_speed) speed = start_speed;
  delay(300);
}

void forward() {
  if (dr_backward) { // if we were moving in other direction, decrease speed
    decrease_speed();
    differentialDrive(SH_Direction_Forward);
  } else {
    increase_speed();
    dr_forward = true; dr_backward = false;
    differentialDrive(SH_Direction_Reverse);
  }
}

void backward() {
  if (dr_forward) { // if we were moving in other direction, decrease speed
    decrease_speed();
    differentialDrive(SH_Direction_Reverse);
  } else {
    increase_speed();
    dr_forward = false; dr_backward = true;
    differentialDrive(SH_Direction_Forward);
  }
}

void differentialDrive(SH_Direction dir, int degr) {
  /*
     Adapt power to the motors to help steering (inner wheel less power, outer more power).
  */
  Serial.print(F("differentialDrive "));
  if (dir == SH_Direction_Reverse) Serial.print(F("Reverse"));
  else Serial.print(F("Forward"));
  Serial.print(F(" speed=")); Serial.print(speed);
  Serial.print(F(" degrees=")); Serial.print(degr);

  // if degrees is != 0, we want to travel a certain distance, measured in degrees
  // are we steering?
  if (evshield.bank_b.motorGetEncoderPosition(steerMotor) > -3 && evshield.bank_b.motorGetEncoderPosition(steerMotor) < 3) {
    // no: just drive both motors equally
    Serial.println(F(" EQUAL "));

    if (degr == 0)
      evshield.bank_a.motorRunUnlimited( SH_Motor_Both, dir, speed);
    else
      evshield.bank_a.motorRunDegrees(SH_Motor_Both, dir, speed, degr, SH_Completion_Wait_For, SH_Next_Action_Float ); // SH_Next_Action_BrakeHold
  } else {
    // yes: we are steering, so calculate power ratio
    float steer_pos = evshield.bank_b.motorGetEncoderPosition( steerMotor ) / 57.296; // calculates current steering position in radians
    float steer_radius = car_wheelbase * tan(1.571 - steer_pos); // calculates the radius, from centerline of car to center of steercircle
    float ratio_L = (steer_radius - (car_rear_track / 2)) / steer_radius;
    float ratio_R = (steer_radius + (car_rear_track / 2)) / steer_radius;

    Serial.print(F(" DIFF "));
    Serial.print(F(" ")); Serial.print(speed * ratio_L); Serial.print(F(" ")); Serial.println(speed * ratio_R);

    if (degr == 0) {
      evshield.bank_a.motorRunUnlimited( SH_Motor_1, dir, speed * ratio_L);
      evshield.bank_a.motorRunUnlimited( SH_Motor_2, dir, speed * ratio_R);
    }
    else {
      evshield.bank_a.motorRunDegrees(SH_Motor_1, dir, speed * ratio_L, degr, SH_Completion_Dont_Wait, SH_Next_Action_Float ); // SH_Next_Action_BrakeHold
      evshield.bank_a.motorRunDegrees(SH_Motor_2, dir, speed * ratio_R, degr, SH_Completion_Wait_For, SH_Next_Action_Float ); // SH_Next_Action_BrakeHold
    }
  }
}

void drive(int distance) { // distance = 0 means drive unlimited
  /*
     Drive the robot. This method has one parameter: distance, which has a default value of 0. Which means that
     if no parameter is given, the value of distance will be 0.
     The method can be called like this:
     drive()      - to drive unlimited
     drive(30)    - drive forward for 30cm (then stop)
     drive(-10)   - drive reverse for 10cm (then stop)
  */
  Serial.print(F("drive() distance=")); Serial.println(distance);
  if (distance == 0) {
    if (dr_backward) { // were we driving backward?
      dr_forward = false; // yes, remain driving backward
      differentialDrive(SH_Direction_Forward);
    } else { // otherwise, drive forward
      dr_forward = true; dr_backward = false;
      differentialDrive(SH_Direction_Reverse);
    }
  }
  else {
    double circumference = car_wheel_diam * PI;
    unsigned int degrees = (abs(distance) / circumference) * 360;
    if (distance > 0) { // positive distance, so just go in the forward direction (do not mind the SH_Direction_Reverse)
      differentialDrive(SH_Direction_Reverse, degrees);
    }
    else {
      differentialDrive(SH_Direction_Forward, degrees); // opposite direction from above
    }
    stop(); // after driving a fixed distance, we always stop
  }
}

void stop() {
  Serial.println(F("stop()"));
  dr_forward = false; dr_backward = false;
  speed = start_speed;
  evshield.bank_a.motorSetSpeed(SH_Motor_Both, start_speed);
  evshield.bank_a.motorStop(SH_Motor_Both, SH_Next_Action_Float);
  evshield.bank_b.ledSetRGB(165, 255, 0); // led orange (indicates ready for driving)
}

void steer(int degr) {
  /*
     Parameter 'degrees' determines the amount of degrees that the front wheels turn, this should not be more than 20 degrees.
     To make a left turn, variable degrees should be negative (eg. -10), to make a right turn, it should be positive.
     Left will use motor direction: SH_Direction_Forward, right: SH_Direction_Reverse
  */
  SH_Direction dir = SH_Direction_Forward;

  if (degr == 0) {
    straight();
    return;
  }
  else if (degr < 0) {
    dir = SH_Direction_Reverse;
  }

  Serial.print(F("steer() dir="));
  if (dir == SH_Direction_Reverse) Serial.print(F("Reverse"));
  else Serial.print(F("Forward"));
  Serial.print(F(" degr=")); Serial.println(degr);

  evshield.bank_b.motorRunDegrees(steerMotor, dir, SH_Speed_Slow, abs(degr), SH_Completion_Wait_For, SH_Next_Action_BrakeHold); // SH_Next_Action_Brake SH_Next_Action_BrakeHold SH_Next_Action_Float
  delay(300);
  Serial.print(F("steer: ")); Serial.println(evshield.bank_b.motorGetEncoderPosition(steerMotor));
}

void turn(int angle) {
  /*
     Rotate the car around its own axis (point turn) with the given angle (in degrees).
     So to turn the car 90 degrees use: turn(90)
     To turn into the opplosite direction use: turn(-90)
  */
  // calculate turn angle:
  unsigned int degrees = abs(angle) * (car_rear_track / car_wheel_diam);
  Serial.print("Turn degrees: "); Serial.println(degrees);
  if (angle > 0) {
    evshield.bank_a.motorRunDegrees(SH_Motor_1, SH_Direction_Forward, speed, degrees, SH_Completion_Dont_Wait, SH_Next_Action_Float);
    evshield.bank_a.motorRunDegrees(SH_Motor_2, SH_Direction_Reverse, speed, degrees, SH_Completion_Wait_For, SH_Next_Action_Float);
  }
  else {
    evshield.bank_a.motorRunDegrees(SH_Motor_2, SH_Direction_Forward, speed, degrees, SH_Completion_Dont_Wait, SH_Next_Action_Float);
    evshield.bank_a.motorRunDegrees(SH_Motor_1, SH_Direction_Reverse, speed, degrees, SH_Completion_Wait_For, SH_Next_Action_Float);
  }
}

void straight() {
  /*
     Set the steering wheels straight.
  */
  Serial.println(F("straight()"));
  if (evshield.bank_b.motorGetEncoderPosition(steerMotor) > 0)
    evshield.bank_b.motorRunTachometer(steerMotor, SH_Direction_Forward, SH_Speed_Slow, 0, SH_Move_Absolute, SH_Completion_Wait_For, SH_Next_Action_BrakeHold);
  else
    evshield.bank_b.motorRunTachometer(steerMotor, SH_Direction_Reverse, SH_Speed_Slow, 0, SH_Move_Absolute, SH_Completion_Wait_For, SH_Next_Action_BrakeHold);
  delay(100);
  evshield.bank_b.motorReset();
  evshield.bank_b.motorResetEncoder(steerMotor);
}


/**
 * Stop and reverse the direction we are driving (go back).
 * (if not driving, this will start the motors in a forward direction)     
 */
void reverseDirection() {
  Serial.println(F("reverseDirection()"));
  if (dr_forward) {
    dr_forward = false;
    dr_backward = true;
  }
  else {
    dr_forward = true;
    dr_backward = false;
  }
  drive();
}


/**
 * Read the distance from the ultrasonic sensor
 */
unsigned int readDistance() {
  return sonar.ping_cm();
}


/**
   Reads a received String (instructions) and turns it into individual commands.
   A command is one alphabetic character followed by a number, like: d100
*/
void readCommand() {
  // local variables:
  String number = "";
  char command = 0;

  instructions.trim();

  for (int i = 0; i < instructions.length(); i++) {
    char c = instructions.charAt(i);
    if (c == '-' || c == '.' || (c >= '0' && c <= '9')) { // number characters
      number.concat(c); // append the character to the number String
    }
    else { // command?
      if (number.length() > 0 && command != 0) { // output previous command
        doCommand(number.toInt(), command);
      }
      if ( (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')) { // possible command characters (a..z)
        command = c;
      }
      else {
        command = 0;
      }
      number = "";
    }
  }
  if (number.length() > 0 && command != 0) { // output previous command
    doCommand(number.toInt(), command);
  }
}


/**
   Translates received command to actual method calls.
*/
void doCommand(int i, char c) {
  if (i == 0) return;

  // show command received:
  //Serial.print("Command"); Serial.print(c); Serial.println(i);

  // send the command to the robot:
  if (c == 's') steer(i);
  else if (c == 't') turn(i);
  else if (c == 'd') drive(i);
}


void find_a_way_out() {
  Serial.println(F("find_a_way_out()"));

  stop();
  // Turn the Ultrasonic Sensor 90 degrees (use motor on top!)
  evshield.bank_b.motorRunDegrees(sensorMotor, SH_Direction_Forward, speed, 90, SH_Completion_Wait_For, SH_Next_Action_BrakeHold );
  // Read the value of the Ultrasonic Sensor (look left)
  unsigned int left = sonar.ping_cm(); // read distance from ultrasonic sensor
  // Turn the Ultrasonic Sensor -180 degrees
  evshield.bank_b.motorRunDegrees(sensorMotor, SH_Direction_Reverse, speed, 180, SH_Completion_Wait_For, SH_Next_Action_BrakeHold );
    
  // take a reading:
  unsigned int right = sonar.ping_cm(); // read distance from ultrasonic sensor

  Serial.print(F("left=")); Serial.print(left);
  Serial.print(F(" right=")); Serial.println(right);
  
  // look forward:    
  evshield.bank_b.motorRunDegrees(sensorMotor, SH_Direction_Forward, speed, 90, SH_Completion_Wait_For, SH_Next_Action_BrakeHold );

  // choose direction
  if (left>right) {
    Serial.println(F("go left"));
    steer(-20);
  }
  else {
    Serial.println(F("go right"));
    steer(20);
  }
  drive(-30);
  straight();
  drive(); // Continue moving
}

/**
 * Make a reverse turn.
 */
void reverseTurn() {
  /* Stop, drive backward a little and then make a turn,
  to 'escape' from something if we bumped into something */
  Serial.println(F("reverseTurn()"));
  steer(15);
  drive(-40); // drive back
  straight();
  drive();
}
