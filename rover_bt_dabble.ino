/*
/*
  This sketch uses the libraries "EVShield" and "Dabble".
  Check if these are present in Documents\Arduino\libraries.
  If not, install them:
  https://home.et.utwente.nl/slootenvanf/wp-content/uploads/appdev-download/Installation_instructions.html#arduino
  
  Uses Dabble in Gamepad mode (libraries\Dabble\src\GamePadModule.h)

  This sketch receives commands via Software serial (Dabble) and execute them with EVshield to drive motors.

  Connect USB cable then Upload this sketch.
  Open the Serial Monitor to view test output. Make sure the speed in the Serial Monitor is set to 9600.

  If the car is driving in the wrong direction, in the loop() function you can swap the calls to forward() and backward()
  at if ... GamePad.isUpPressed() and GamePad.isDownPressed()
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
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) ; // wait for serial port to connect. Needed for native USB port only

  // BLE:
  Dabble.begin(9600, 10, 11); // Baudrate & RX, TX to which bluetooth module is connected

  // EVshield:
  evshield.init( SH_HardwareI2C );

  Serial.println(F("Use motors M1 and M2 on Bank A to drive."));

  // reset motors
  evshield.bank_a.motorReset();
  evshield.bank_b.motorReset();
  evshield.bank_b.motorResetEncoder( SH_Motor_1 ); // Reset the current encoder position to zero for the motor
  Serial.println( evshield.bank_a.evshieldGetBatteryVoltage() );
  
  evshield.ledSetRGB(165, 255, 0); // led orange (indicates ready for driving)
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

  if ( evshield.getButtonState(BTN_GO) ) stop();
}

void manage_speed() {
  speed++; // increase speed
  delay(500);
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
  evshield.ledSetRGB(0, 0, 255); // led blue (indicates driving forward)
}

void do_stop() {
  speed=SH_Speed_Slow;
  evshield.bank_a.motorStop(SH_Motor_Both, SH_Next_Action_Float);
  evshield.bank_a.motorSetSpeed (SH_Motor_Both, SH_Speed_Slow);
  delay(500);
}

void stop() {
  dr_forward=false; dr_backward=false;
  do_stop();
  evshield.ledSetRGB(165, 255, 0); // led orange (indicates ready for driving)
}

void left() {
  evshield.bank_b.motorRunDegrees(SH_Motor_1, SH_Direction_Forward, SH_Speed_Slow, 10, SH_Completion_Wait_For, SH_Next_Action_BrakeHold);
}

void right() {
  evshield.bank_b.motorRunDegrees(SH_Motor_1, SH_Direction_Reverse, SH_Speed_Slow, 10, SH_Completion_Wait_For, SH_Next_Action_BrakeHold);
}

void straight() {
  if (evshield.bank_b.motorGetEncoderPosition( SH_Motor_1 )>0)
    evshield.bank_b.motorRunTachometer(SH_Motor_1, SH_Direction_Forward, SH_Speed_Slow, 0, SH_Move_Absolute,SH_Completion_Wait_For, SH_Next_Action_BrakeHold);
  else
    evshield.bank_b.motorRunTachometer(SH_Motor_1, SH_Direction_Reverse, SH_Speed_Slow, 0, SH_Move_Absolute,SH_Completion_Wait_For, SH_Next_Action_BrakeHold);
}
