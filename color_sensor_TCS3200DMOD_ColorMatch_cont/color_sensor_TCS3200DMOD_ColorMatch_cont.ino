/* 
  TCS230 sensor continuous color matching
  https://home.et.utwente.nl/slootenvanf/2019/04/18/tcs3200-color-sensor/

  Libraries used:
  MD_TCS230 https://github.com/MajicDesigns/MD_TCS230
  FreqCount https://www.pjrc.com/teensy/td_libs_FreqCount.html
  
  Input and output is via the Serial Monitor (set baudrate to 9600 and line ending to "Both NL & CR").

  How to use this:
  1. Run the software in LEARN MODE and capture the calibration 
     constants and the output for an array of colors. Easiest is to 
     edit the table in the header file with the names of the colors
     and use that as a prompt when scanning the color.
  2. At the end of scanning all the colors, the text for the header 
     file is produced in the serial console. Cut and paste that into 
     the actual header file to capture the calibration and color 
     table data that has just been 'learnt'.
  3. Compile and download the program.
  4. Run the program in MATCH MODE to test the color recognition.
     Each color will be matched to an entry in the color table and 
     the name of the closest color will be displayed.
*/

#include <MD_TCS230.h>
#include <FreqCount.h>
#include "ColorMatch.h"

// Pin definitions
#define OE 4
#define S0 6
#define S1 7
#define S2 8
#define S3 9
// connect OUT of Color Sensor to PIN 5 of Arduino!

// Use TCS3200 version of constructor:
// https://majicdesigns.github.io/MD_TCS230/class_m_d___t_c_s230.html
// MD_TCS230(uint8_t s2, uint8_t s3, uint8_t s0, uint8_t s1, uint8_t oe)
MD_TCS230 CS(S2, S3, S0, S1, OE);

// Global variables
colorData	rgb;

long previousMillis = 0;        // will store time between colors
int last_i = -1; // previos color index

void setup()  {
	Serial.begin(9600);
	Serial.print(F("\n[TCS230 Calibrator Example]"));
	
	// initialise color sensor
	CS.begin();
	// use calibration parameters from the header file:
  CS.setDarkCal(&sdBlack);
	CS.setWhiteCal(&sdWhite);
}

uint8_t fsmReadValue(uint8_t state) {
// Finite State Machine for reading a value from the sensor
// Current FSM state is passed in and returned
// Current reading stored in a global rgb buffer
	switch(state) {
	case 0:	// start the reading process
    CS.read();
		state++;
		break;

	case 1:	// wait for a read to complete
		if (CS.available()) {
        CS.getRGB(&rgb);
        state++;
    }
		break;

	default:	// reset fsm
		state = 0;
		break;
	}

	return(state);
}

uint8_t colorMatch(colorData *rgb) {
// Root mean square distance between the color and colors in the table.
// For a limited range of colors this method works ok using RGB
// We don't work out the square root or the mean as it has no effect on the 
// comparison for minimum. Square of the distance is used to ensure that 
// negative distances do not subtract from the total.

	int32_t		d;
	uint32_t	v, minV = 999999L;
	uint8_t		minI;

	for (uint8_t i=0; i<ARRAY_SIZE(ct); i++) {
		v = 0;
		for (uint8_t j=0; j<RGB_SIZE; j++) {
			d = ct[i].rgb.value[j] - rgb->value[j];
			v += (d * d);
		}
		if (v < minV)	{ // new best
			minV = v;
			minI = i;
		}
		if (v == 0)		// perfect match, no need to search more
			break;
	}

	return(minI);
}

void loop() {
	static uint8_t	runState = 0;		
	static uint8_t	readState = 0;
  int i = -1;

  unsigned long currentMillis = millis();
 
  // Matching mode
	switch(runState) {
		case 0:	// read a value
			readState = fsmReadValue(readState);
			if (readState == 0) runState++;
			break;

		case 1: // find the matching color
				i = colorMatch(&rgb);
       Serial.print(i); Serial.print(" ");
       Serial.print(last_i); Serial.print(" ");
       if (i!=last_i) { // new color?
          Serial.print(currentMillis-previousMillis);
          Serial.print(" ");
          previousMillis = currentMillis;
          last_i = i;
       }
				Serial.println(ct[i].name);
				runState++;
			break;

		default:
			runState = 0;	// start again if we get here as something is wrong
	}
}
