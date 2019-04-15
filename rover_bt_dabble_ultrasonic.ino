/*
  Checkout the building guide:
  https://home.et.utwente.nl/slootenvanf/2019/04/04/lego-rover-car/
  
  This sketch uses the libraries "EVShield", "Dabble" and "NewPing".
  Check if these are present in Documents\Arduino\libraries. If not, install them:
  https://home.et.utwente.nl/slootenvanf/wp-content/uploads/appdev-download/Installation_instructions.html#arduino
  
  Uses Dabble in Gamepad mode (libraries\Dabble\src\GamePadModule.h)

  This sketch receives commands via Software serial (Dabble) and execute them with EVshield to drive motors.

  Connect USB cable then Upload this sketch.
  Open the Serial Monitor to view test output. Make sure the speed in the Serial Monitor is set to 9600.

  If the car is driving in the wrong direction, swap SH_Direction_Reverse and SH_Direction_Forward in forward() and backward() functions
 */
#include <Wire.h>
#include <EVShield.h>

// dabble:
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#define INCLUDE_MUSIC_MODULE
#include <Dabble.h>

EVShield evshield(0x34,0x36);

unsigned int speed=SH_Speed_Slow; // start speed
boolean dr_forward = false, dr_backward = false; // moving in forward or backward direction

boolean blink_led = false;
long previousMillis = 0;        // will store last time LED was updated

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
  Serial.print(F("Battery voltage: ")); Serial.print( voltage ); Serial.println(F(" mV (on batteries, should be above 5000)"));

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
  if ( evshield.getButtonState(BTN_GO) ) {
    Serial.println(F("GO"));
    if (dr_forward||dr_backward) { // driving (forward or backward)?
      Music.stop();
      Music.play("B4");
      stop();
    }
    else {
      straight();
      forward();
    }
  }
  else if (evshield.getButtonState( BTN_LEFT ) ) {
    left();
  }
  else if (evshield.getButtonState( BTN_RIGHT ) ) {
    right();
  }
  
  unsigned int distance = sonar.ping_cm(); // read distance from ultrasonic sensor

  if (us_on && dr_forward && distance > 5 && distance < 30 ) { // if ultrasonic sensor is turned on, and driving forward and object detected nearby
    Serial.print("Distance: "); Serial.println(distance);
    stop();
  }

  if (dr_backward) { // if driving backwards
    unsigned long currentMillis = millis();
 
    if(currentMillis - previousMillis > 500) { // blink with 500ms rate (0.5 sec)
      // save the last time you blinked the LED 
      previousMillis = currentMillis;   
   
      // if the LED is off turn it on and vice-versa:
      blink_led = !blink_led;
      if (blink_led)
        evshield.ledSetRGB(0, 0, 255); // led blue (indicates driving backward)
      else 
        evshield.ledSetRGB(0, 0, 0); // led off
    }
  }
  
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
  else if (GamePad.isCrossPressed()) {
    us_on = !us_on; // toggle us_on (ultrasonic sensor on/off)
    if (us_on) {
      Serial.println(F("us on"));
      evshield.bank_a.centerLedSetRGB( 0, 255, 0); // green if ultrasonic sensor on
    }
    else {
      Serial.println(F("us off"));
      evshield.bank_a.centerLedSetRGB( 255, 0, 0); // red
    }
    delay(300);
  }
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
  evshield.bank_a.motorRunUnlimited( SH_Motor_Both, SH_Direction_Reverse, speed);
  evshield.ledSetRGB(255, 0, 0); // led green (indicates driving forward)
}

void backward() {
  if (dr_forward) { // if we were moving in other direction, stop gently
    do_stop();
  }
  manage_speed();
  dr_forward=false; dr_backward=true;
  evshield.bank_a.motorRunUnlimited( SH_Motor_Both, SH_Direction_Forward, speed);
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
