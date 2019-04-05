/**
 * Test Ultrasonic sensor with NewPing library
 * https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
 * 
 * If using this with the EVShield, and the sensor is connected to the servo header,
 * make sure the EVShield is powered by batteries!
 */

#include <NewPing.h>

#define TRIGGER_PIN  3  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     5  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup() {
  Serial.begin(9600); // start serial port output, check for same speed at Serial Monitor
}

void loop() {
  unsigned int distance = sonar.ping_cm(); // read distance from ultrasonic sensor
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(1000); // wait one second
}
