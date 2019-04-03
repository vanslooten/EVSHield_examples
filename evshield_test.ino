/*
  Mount Arduino to EVshield, connect USB cable then run this sketch.
  Open the Serial terminal to view.
*/

#include <Wire.h>
#include <EVShield.h>

EVShield evshield(0x34,0x36);

void setup() {
  Serial.begin(9600); // start serial port output, check for same speed at Serial Monitor!
  while (!Serial) ; // wait for serial port to connect. Needed for native USB port only
  
  evshield.init( SH_HardwareI2C );

  Serial.print(F("Battery voltage: ")); Serial.print( evshield.bank_a.evshieldGetBatteryVoltage() ); Serial.println(F(" mV (should be above 4000)"));

  Serial.println("Press Go to start");
  evshield.waitForButtonPress(BTN_GO);

  Serial.println("Starting...");

  evshield.ledSetRGB(0, 255, 0); // turn on led, green
  evshield.bank_a.centerLedSetRGB(0, 0, 255); // turn on center led (between buttons)
  delay(1000);
}

void loop() {
  if (evshield.getButtonState( BTN_GO ) ) {
    Serial.println(F("BTN_GO"));
    evshield.ledSetRGB(165, 255, 0); // change color
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
