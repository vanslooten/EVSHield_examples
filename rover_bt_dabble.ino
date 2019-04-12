/*
  Checkout the building guide:
  https://home.et.utwente.nl/slootenvanf/2019/04/04/lego-rover-car/
  
  This sketch uses the libraries "EVShield" and "Dabble".
  Check if these are present in Documents\Arduino\libraries. If not, install them:
  https://home.et.utwente.nl/slootenvanf/wp-content/uploads/appdev-download/Installation_instructions.html#arduino
  
  Uses Dabble in Gamepad mode (libraries\Dabble\src\GamePadModule.h)

  This sketch receives commands via Software serial (Dabble) and execute them with EVShield to drive motors.

  Connect USB cable then Upload this sketch.
  Open the Serial Monitor to view test output. Make sure the speed in the Serial Monitor is set to 9600.

  If the car is driving in the wrong direction, swap SH_Direction_Reverse and SH_Direction_Forward in forward() and backward() functions
 */
#include <Wire.h>
#include <EVShield.h>

// dabble:
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <Dabble.h>

EVShield evshield(0x34,0x36);

unsigned int speed=SH_Speed_Slow; // start speed
boolean dr_forward = false, dr_backward = false; // moving in forward or backward direction

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
 
  Serial.println(F("Waiting for App connection..."));
  Dabble.waitForAppConnection();
  delay(1000);
  Serial.println(F("Ready to roll!"));
}

void loop() {
  Dabble.processInput();
  if (GamePad.isUpPressed()||GamePad.isStartPressed()) {
    Serial.println(F("UP"));
    backward();
  }
  else if (GamePad.isDownPressed()) {
    Serial.println(F("DOWN"));
    forward();
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
    stop();
  }
  else if (GamePad.isTrianglePressed()) {
    straight();
  }

  if ( evshield.getButtonState(BTN_GO) ) {
    if (dr_forward||dr_backward) stop();
    else forward();
  }
  //checkMotors();
}

void manage_speed() {
  speed=speed+5; // increase speed
  delay(300);
  if (speed>SH_Speed_Full) speed=SH_Speed_Full;
  Serial.print(F("speed=")); Serial.println(speed);
}

void forward() {
  if (dr_backward) { // if we were moving in other direction, stop gently
    do_stop();
  }
  manage_speed();
  dr_forward=true; dr_backward=false;
  evshield.bank_a.motorRunUnlimited( SH_Motor_Both, SH_Direction_Forward, speed);
  evshield.ledSetRGB(255, 0, 0); // led green (indicates driving forward)
}

void backward() {
  if (dr_forward) { // if we were moving in other direction, stop gently
    do_stop();
  }
  manage_speed();
  dr_forward=false; dr_backward=true;
  evshield.bank_a.motorRunUnlimited( SH_Motor_Both, SH_Direction_Reverse, speed);
  evshield.ledSetRGB(0, 0, 255); // led blue (indicates driving backward)
}

void do_stop() {
  speed=SH_Speed_Slow;
  evshield.bank_a.motorSetSpeed(SH_Motor_Both, SH_Speed_Slow);
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
  evshield.bank_b.motorRunDegrees(SH_Motor_1, SH_Direction_Forward, SH_Speed_Slow, 10, SH_Completion_Wait_For, SH_Next_Action_BrakeHold); // SH_Next_Action_BrakeHold SH_Next_Action_Float
  delay(300);
}

void right() {
  evshield.bank_b.motorRunDegrees(SH_Motor_1, SH_Direction_Reverse, SH_Speed_Slow, 10, SH_Completion_Wait_For, SH_Next_Action_BrakeHold);
  delay(300);
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
