/*
  Dabble music with gamepad test: use the Gamepad to play some notes.
  By default only the notes A4 B4 work. To add more notes, in the Dabble App, go to "Music".
 */

// dabble:
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#define INCLUDE_MUSIC_MODULE
#include <Dabble.h>

void setup() {
  // Open serial communication:
  Serial.begin(9600);

  // initialize digital pin LED_BUILTIN as an output, we use it to blink the led briefly as a tone is played
  pinMode(LED_BUILTIN, OUTPUT);

  // BLE:
  Dabble.begin(9600, 10, 11); // Baudrate & RX, TX to which bluetooth module is connected

  Music.play("A4");
  delay(1000);
  Serial.println(F("Ready to play!"));
}

void loop() {
  digitalWrite(LED_BUILTIN, LOW); // Turn off onboard led
  
  // start processing input from Dabble app:
  Dabble.processInput();
  if (GamePad.isUpPressed()||GamePad.isStartPressed()) {
    Serial.println(F("UP"));
    play("A4");
  }
  else if (GamePad.isDownPressed()||GamePad.isSelectPressed()) {
    Serial.println(F("DOWN"));
    play("B4");
  }
  else if (GamePad.isLeftPressed()) {
    Serial.println(F("LEFT"));
    play("C4");
  }
  else if (GamePad.isRightPressed()) {
    Serial.println(F("RIGHT"));
    play("C5");
  }
  else if (GamePad.isSquarePressed()) {
    Serial.println(F("Square"));
    play("D4");
  }
  else if (GamePad.isTrianglePressed()) {
    Serial.println(F("Triangle"));
    play("E4");
  }
  else if (GamePad.isCirclePressed()) {
    Serial.println(F("Circle"));
    play("F4");
  }
  else if (GamePad.isCrossPressed()) {
    Serial.println(F("Cross"));
    play("G4");
    //Music.addToQueue("C5"); // add another, to play after the first
  }
}

void play(char* s) {
  digitalWrite(LED_BUILTIN, HIGH); // Turn on onboard led
  Music.stop();
  Music.play(s);
  delay(300);
}
