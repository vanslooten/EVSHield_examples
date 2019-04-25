/*
  Checkout the building guide:
  https://home.et.utwente.nl/slootenvanf/2019/04/04/lego-rover-car/

  This sketch has been improved with differentialDrive by Thimo Willems
  
  This sketch uses the libraries "EVShield" and "Dabble".
  Check if these are present in Documents\Arduino\libraries. If not, install them:
  https://home.et.utwente.nl/slootenvanf/wp-content/uploads/appdev-download/Installation_instructions.html#arduino
  
  Uses Dabble in Gamepad mode (libraries\Dabble\src\GamePadModule.h)

  This sketch receives commands via Software serial (Dabble) and execute them with EVShield to drive motors.

  Connect USB cable then Upload this sketch.
  Open the Serial Monitor to view test output. Make sure the speed in the Serial Monitor is set to 9600.

  If the car is driving in the wrong direction, find occurrences of calls to differentialDrive(...) and
  swap the parameters SH_Direction_Reverse with SH_Direction_Forward and vice versa.
  Also, in function differentialDrive(), swap variables ratio_L and ratio_R at the last two lines.
 */
#include <Wire.h>
#include <EVShield.h>

// dabble:
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#define INCLUDE_MUSIC_MODULE
#include <Dabble.h>

EVShield evshield(0x34,0x36);

unsigned int start_speed = 15;  // start speed (speed can be any value between 0-100)
unsigned int speed=start_speed;
boolean dr_forward = false, dr_backward = false; // moving in forward or backward direction

// car dimension related:
unsigned int car_rear_track = 145; // car's rear track, the distance between the centerline of each rear wheel (in millimeters!)
unsigned int car_wheelbase = 185; // car's wheelbase, the distance between the center of the front wheels and the rear wheels (in millimeters!)

// for Ultrasonic sensor:
#include <NewPing.h>

#define TRIGGER_PIN  3  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     5  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
boolean us_on = true;

void setup() {
  // Open serial communication:
  Serial.begin(9600);

  // BLE:
  Dabble.begin(9600, 10, 11); // Baudrate & RX, TX to which bluetooth module is connected

  // EVshield:
  evshield.init( SH_HardwareI2C );

  Serial.println(F("Use motors M1 and M2 on Bank A to drive."));

  // reset motors
  evshield.bank_a.motorReset();
  evshield.bank_b.motorReset();
  evshield.bank_b.motorResetEncoder( SH_Motor_1 ); // Reset the current encoder position to zero for the motor

  unsigned int voltage = evshield.bank_a.evshieldGetBatteryVoltage();
  Serial.print(F("Battery voltage: ")); Serial.print( voltage ); Serial.println(F(" mV (on batteries, should be above 6000)"));

  if (voltage>6500)
    evshield.ledSetRGB(0, 255, 0); // led green, battery Ok, ready for driving
  else if (voltage>5500)
    evshield.ledSetRGB(160, 160, 20); // led orange, battery might be low, ready for driving
  else
    evshield.ledSetRGB(255, 0, 0); // led red, battery low, problems might occur driving motors
 
  Music.play("A4"); // if already connected to Dabble app, play a tone
  delay(1000);
  Serial.println(F("Ready to roll!"));
}

void loop() {
  // processing input from ultrasonic sensor:
  unsigned int distance = sonar.ping_cm(); // read distance from ultrasonic sensor

  if (us_on && dr_forward && distance > 5 && distance < 30 ) { // if ultrasonic sensor is turned on, and driving forward and object detected nearby
    Serial.print(F("Distance: ")); Serial.println(distance);
    stop();
  }

  // processing input from buttons on EVShield:
  if ( evshield.getButtonState(BTN_GO) ) {
    Serial.println(F("GO"));
    if (dr_forward||dr_backward) { // driving (forward or backward)?
      Music.stop();
      Music.play("B4");
      stop();
    }
    else {
      forward();
    }
  }
  else if (evshield.getButtonState( BTN_LEFT ) ) {
    left();
  }
  else if (evshield.getButtonState( BTN_RIGHT ) ) {
    right();
  }

  // start processing input from Dabble app:
  Dabble.processInput();
  
  if (GamePad.isUpPressed()||GamePad.isStartPressed()) {
    Serial.println(F("UP"));
    forward();
  }
  else if (GamePad.isDownPressed()) {
    Serial.println(F("DOWN"));
    backward();
  }
  else if (GamePad.isLeftPressed()) {
    Serial.println(F("LEFT"));
    left();
  }
  else if (GamePad.isRightPressed()) {
     Serial.println(F("RIGHT"));
     right();
  }
  else if (GamePad.isSquarePressed()) {
    Music.stop();
    Music.play("B4");
    stop();
  }
  else if (GamePad.isTrianglePressed()) {
    straight();
  }

  //checkMotors(); // used for debugging only
}

void increase_speed() {
  speed=speed+5; // increase speed
  delay(300);
  if (speed>SH_Speed_Full) speed=SH_Speed_Full;
}

void decrease_speed() {
  speed=speed-5; // increase speed
  if (speed<start_speed) speed=start_speed;
  delay(300);
}

void forward() {
  if (dr_backward) { // if we were moving in other direction, decrease speed
    decrease_speed();
    differentialDrive(SH_Direction_Forward);
  } else {
    increase_speed();
    dr_forward=true; dr_backward=false;
    differentialDrive(SH_Direction_Reverse);
    evshield.ledSetRGB(255, 0, 0); // led green (indicates driving forward)
  }
}

void backward() {
  if (dr_forward) { // if we were moving in other direction, decrease speed
    decrease_speed();
    differentialDrive(SH_Direction_Reverse);
  } else {
    increase_speed();
    dr_forward=false; dr_backward=true;
    differentialDrive(SH_Direction_Forward);
    evshield.ledSetRGB(0, 0, 255); // led blue (indicates driving backward)
  }
}

void do_stop() {
  speed=start_speed;
  evshield.bank_a.motorSetSpeed(SH_Motor_Both, start_speed);
  delay(200);
  evshield.bank_a.motorStop(SH_Motor_Both, SH_Next_Action_Float);
  delay(200);
}

void stop() {
  dr_forward=false; dr_backward=false;
  do_stop();
  evshield.bank_a.motorReset();
  evshield.ledSetRGB(165, 255, 0); // led orange (indicates ready for driving)
}

void left() {
  evshield.bank_b.motorRunDegrees(SH_Motor_1, SH_Direction_Forward, SH_Speed_Slow, 12, SH_Completion_Wait_For, SH_Next_Action_BrakeHold); // SH_Next_Action_BrakeHold SH_Next_Action_Float
  delay(300);
  Serial.print(F("steer: ")); Serial.println(evshield.bank_b.motorGetEncoderPosition( SH_Motor_1 ));
  if (dr_forward) {
    differentialDrive(SH_Direction_Reverse);
  } else if (dr_backward) {
    differentialDrive(SH_Direction_Forward);
  }
}

void right() {
  evshield.bank_b.motorRunDegrees(SH_Motor_1, SH_Direction_Reverse, SH_Speed_Slow, 12, SH_Completion_Wait_For, SH_Next_Action_BrakeHold);
  delay(300);
  Serial.print(F("steer: ")); Serial.println(evshield.bank_b.motorGetEncoderPosition( SH_Motor_1 ));
  if (dr_forward) {
    differentialDrive(SH_Direction_Reverse);
  } else if (dr_backward) {
    differentialDrive(SH_Direction_Forward);
  }
}

void straight() {
  if (evshield.bank_b.motorGetEncoderPosition( SH_Motor_1 )>0)
    evshield.bank_b.motorRunTachometer(SH_Motor_1, SH_Direction_Forward, SH_Speed_Slow, 0, SH_Move_Absolute,SH_Completion_Wait_For, SH_Next_Action_BrakeHold);
  else
    evshield.bank_b.motorRunTachometer(SH_Motor_1, SH_Direction_Reverse, SH_Speed_Slow, 0, SH_Move_Absolute,SH_Completion_Wait_For, SH_Next_Action_BrakeHold);
  delay(300);
  evshield.bank_b.motorReset();
}

void checkMotors() {
/*
 Check all 3 used motors to see if they are stalled.
 From EVShield-Advanced-Development-Guide.pdf:
 Stopped due to stall:
    Position Control bit (bit 3) is 1
    ‘Motor is Powered’ bit (bit 2) is 1
    Stalled bit (bit 7) is 1
*/
  bool reset = false;
  uint8_t status = evshield.bank_b.motorGetStatusByte(SH_Motor_1);
  if (bitRead(status, 2) && bitRead(status, 3) && bitRead(status, 7)) reset = true; // if bit 2, 3 and 7 are '1'
  status = evshield.bank_a.motorGetStatusByte(SH_Motor_1);
  if (bitRead(status, 2) && bitRead(status, 3) && bitRead(status, 7)) reset = true; // if bit 2, 3 and 7 are '1'
  status = evshield.bank_a.motorGetStatusByte(SH_Motor_2);
  if (bitRead(status, 2) && bitRead(status, 3) && bitRead(status, 7)) reset = true; // if bit 2, 3 and 7 are '1'
  if (reset) {
    straight();
    stop();
    evshield.bank_a.centerLedSetRGB( 255, 0, 0); // red
  }
  else evshield.bank_a.centerLedSetRGB( 0, 0, 0); // off
}

void differentialDrive(SH_Direction dir) {
  if (evshield.bank_b.motorGetEncoderPosition( SH_Motor_1 ) > -3 && evshield.bank_b.motorGetEncoderPosition( SH_Motor_1 ) < 3) {
    //just drive both motors equally
    evshield.bank_a.motorRunUnlimited( SH_Motor_Both, dir, speed);
  } else {
    float steer_pos = evshield.bank_b.motorGetEncoderPosition( SH_Motor_1 ) / 57.296; // calculates current steering position in radians
    float steer_radius = car_wheelbase * tan(1.571 - steer_pos); // calculates the radius, from centerline of car to center of steercircle
    float ratio_L = (steer_radius - (car_rear_track / 2)) / steer_radius;
    float ratio_R = (steer_radius + (car_rear_track / 2)) / steer_radius;
    
    evshield.bank_a.motorRunUnlimited( SH_Motor_1, dir, speed * ratio_L);
    evshield.bank_a.motorRunUnlimited( SH_Motor_2, dir, speed * ratio_R);
  }
}
