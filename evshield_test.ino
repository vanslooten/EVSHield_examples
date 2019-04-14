/* 
  This sketch uses the library "EVShield".
  Check if this library is present in Documents\Arduino\libraries.
  If not, install it:
  https://home.et.utwente.nl/slootenvanf/wp-content/uploads/appdev-download/Installation_instructions.html#arduino

  Mount the Arduino Uno to the EVshield:
  https://home.et.utwente.nl/slootenvanf/2018/04/03/lego-evshield-battery-pack/
  
  Connect USB cable then Upload this sketch.
  Open the Serial Monitor to view test output. Make sure the speed in the Serial Monitor is set to 9600.
*/

#include <Wire.h>
#include <EVShield.h>

EVShield evshield(0x34,0x36);

void setup() {
  Serial.begin(9600); // start serial port output, check for same speed at Serial Monitor!
  
  evshield.init( SH_HardwareI2C );

  Serial.println("Press Go to start");
  evshield.waitForButtonPress(BTN_GO);

  Serial.println("Starting...");
  delay(1000);
  
  unsigned int voltage = evshield.bank_a.evshieldGetBatteryVoltage();
  Serial.print(F("Battery voltage: ")); Serial.print( voltage ); Serial.println(F(" mV (on batteries, should be above 5000)"));

  if (voltage>6500)
    evshield.ledSetRGB(0, 255, 0); // led green, battery Ok, ready for driving
  else if (voltage>5500)
    evshield.ledSetRGB(160, 160, 20); // led orange, battery might be low, ready for driving
  else
    evshield.ledSetRGB(255, 0, 0); // led red, battery low, problems might occur driving motors

  evshield.bank_a.centerLedSetRGB(160, 160, 20); // turn on center led (between buttons)
}

void loop() {
  if (evshield.getButtonState( BTN_GO ) ) {
    Serial.println(F("BTN_GO"));
    evshield.ledSetRGB(0, 255, 100); // change color
  }
  else if (evshield.getButtonState( BTN_LEFT ) ) {
    Serial.println(F("BTN_LEFT"));
    evshield.ledSetRGB(0, 0, 255); // change color
  }
  else if (evshield.getButtonState( BTN_RIGHT ) ) {
    Serial.println(F("BTN_RIGHT"));
    evshield.ledSetRGB(255, 204, 229); // change color
  }
  delay(200);
}
