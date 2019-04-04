/*
  This sketch uses the libraries "EVShield" and "Dabble".
  Check if these are present in Documents\Arduino\libraries.
  If not, install them:
  https://home.et.utwente.nl/slootenvanf/wp-content/uploads/appdev-download/Installation_instructions.html#arduino
  
  Uses Dabble in Gamepad mode (libraries\Dabble\src\GamePadModule.h)

  This sketch receives commands via Software serial (Dabble) and execute them with EVshield.

  Connect USB cable then Upload this sketch.
  Open the Serial Monitor to view test output. Make sure the speed in the Serial Monitor is set to 9600.
 */
#include <Wire.h>
#include <EVShield.h>

// dabble:
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <Dabble.h>

EVShield evshield(0x34,0x36);

void setup() {
  // Open serial communications
  Serial.begin(9600);

  // Have Dabble connect to Bluetooth Module:
  Dabble.begin(9600, 10, 11); // Baudrate, RX, TX to which bluetooth module is connected

  // EVshield:
  evshield.init( SH_HardwareI2C );

  Serial.println(F("Welcome to the Dabble test."));
  
  Serial.print(F("Battery voltage: ")); Serial.print( evshield.bank_a.evshieldGetBatteryVoltage() ); Serial.println(F(" mV (should be above 4000)"));

  Serial.println(F("Connect to the Bluetooth module with the Dabble app, then press some buttons on the Gamepad"));
}

void loop() {
  Dabble.processInput();
  if (GamePad.isUpPressed()||GamePad.isStartPressed()) {
    changeColor("UP", 255, 0, 0);
  }
  else if (GamePad.isDownPressed()) {
    changeColor("DOWN", 0, 255, 0);
  }
  else if (GamePad.isLeftPressed()) {
    changeColor("LEFT", 0, 0, 255);
  }
  else if (GamePad.isRightPressed()) {
    changeColor("RIGHT", 255, 0, 255);
  }
  else if (GamePad.isSquarePressed()) {
    changeColor("STOP", 0, 255, 255);
  }
  else if (GamePad.isTrianglePressed()) {
    changeColor("Triangle", 255, 255, 0);
  }

  if ( evshield.getButtonState(BTN_GO) ) {
    changeColor("EVShield BTN_GO", 255, 255, 255);
  }
}

void changeColor(char* str, int r, int g, int b) {
  Serial.println(str);
  evshield.ledSetRGB(r,g,b); // change color
  delay(200);
}
