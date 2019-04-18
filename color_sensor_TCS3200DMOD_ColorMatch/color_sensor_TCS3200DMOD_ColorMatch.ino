/* 
  TCS230 sensor calibration and color matching

  Libraries used:
  MD_TCS230 https://github.com/MajicDesigns/MD_TCS230
  FreqCount 
  
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

#define	BLACK_CAL	0
#define	WHITE_CAL	1
#define	READ_VAL	2
#define	LEARN_VAL	3

#define	LEARN_MODE	'L'
#define	MATCH_MODE	'M'

// Pin definitions (OUT must be on pin 5 as set in FreqCount library)
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
uint8_t	modeOp = 0;		// mode of operation
uint8_t ctIndex;
colorData	rgb;

void setup()  {
	Serial.begin(9600);
	Serial.print(F("\n[TCS230 Calibrator Example]"));
	
	// input mode of operation
	while (modeOp == 0) {
		Serial.print(F("\nOperate in Learn (L) or Match (M) mode?"));
		modeOp = getChar();
		if (modeOp != LEARN_MODE && modeOp != MATCH_MODE)
			modeOp = 0;
		else
			clearInput();
	}

	// initialise color sensor
	CS.begin();
	if (modeOp == MATCH_MODE)	{	// use calibration parameters from the header file
		CS.setDarkCal(&sdBlack);
		CS.setWhiteCal(&sdWhite);
	}
}

char getChar() {
  // blocking wait for an input character from the input stream
	while (Serial.available() == 0)
		;
  char c = toupper(Serial.read());
  delay(500);
	return c;
}

void clearInput() {
  // clear out serial input
	while (Serial.read() != -1)
		;
}

void outputHeader(void) {
	// write out the new header file with sensor values included
	Serial.print(F("\n// Header file for the ColorMatch application"));
	Serial.print(F("\n// This file is generated by the ColorMatch Application in Learn Mode"));
	
	Serial.print(F("\n\n// Calibration data"));
	Serial.print(F("\nsensorData sdBlack = { "));
	Serial.print(sdBlack.value[0]); Serial.print(F(", "));
	Serial.print(sdBlack.value[1]); Serial.print(F(", "));
	Serial.print(sdBlack.value[2]); Serial.print(F(" };"));
	
	Serial.print(F("\nsensorData sdWhite = { "));
	Serial.print(sdWhite.value[0]); Serial.print(F(", "));
	Serial.print(sdWhite.value[1]); Serial.print(F(", "));
	Serial.print(sdWhite.value[2]); Serial.print(F(" };"));

	Serial.print(F("\n\n// Color Table for matching"));
	Serial.print(F("\ntypedef struct\n{\n  char    name[9];  // color name 8+nul\n  colorData rgb;    // RGB value\n} colorTable;"));
	
	Serial.print(F("\n\ncolorTable ct[] = \n{"));
	for (uint8_t i=0; i<ARRAY_SIZE(ct); i++) {
		Serial.print(F("\n  {\""));
		Serial.print(ct[i].name);
		Serial.print(F("\", {"));
		Serial.print(ct[i].rgb.value[0]);
		Serial.print(F(", "));
		Serial.print(ct[i].rgb.value[1]);
		Serial.print(F(", "));
		Serial.print(ct[i].rgb.value[2]);
		Serial.print(F("} },"));
	}
	Serial.print(F("\n};\n"));
}

uint8_t fsmReadValue(uint8_t state, uint8_t valType) {
// Finite State Machine for reading a value from the sensor
// Current FSM state is passed in and returned
// Type of value being read is passed in
// Current reading stored in a global rgb buffer

	static	uint8_t	selChannel;
	static	uint8_t	readCount;
	static	sensorData	sd;

	switch(state) {
	case 0:	// Prompt for the user to start
		Serial.print(F("\n\nReading value for "));
		switch(valType) {
		case BLACK_CAL:	Serial.print(F("BLACK calibration"));	break;
		case WHITE_CAL:	Serial.print(F("WHITE calibration"));	break;
		case READ_VAL:	Serial.print(F("color MATCH"));		break;
		case LEARN_VAL:	Serial.print(ct[ctIndex].name);			break;
		default:		Serial.print(F("??"));					break;
		}
		
		clearInput();

		if (valType == LEARN_VAL) {
			Serial.print(F("\nNew name (<enter> for old) to start: "));    
      while (Serial.available() == 0) ; // wait for  input from the Serial Monitor
			String p=Serial.readString(); // read the typed string
      p.trim();
			if (p.length()>1)
				p.toCharArray(ct[ctIndex].name,9); // copy p into ct[ctIndex].name
			Serial.print(ct[ctIndex].name);
		}
		else {
			Serial.print(F("\nPress Enter to start ..."));
			getChar();
			clearInput();
		}
		state++;
		break;

	case 1:	// start the reading process
		CS.read();
		state++;
		break;

	case 2:	// wait for a read to complete
		if (CS.available()) {
			switch(valType)
			{
			case BLACK_CAL:	
				CS.getRaw(&sdBlack);	
				CS.setDarkCal(&sdBlack);		
				break;

			case WHITE_CAL:	
				CS.getRaw(&sdWhite);	
				CS.setWhiteCal(&sdWhite);	
				break;

			case READ_VAL:	
			case LEARN_VAL: 
				CS.getRGB(&rgb);
				Serial.print(F("\nRGB is ["));
				Serial.print(rgb.value[TCS230_RGB_R]);
				Serial.print(F(","));
				Serial.print(rgb.value[TCS230_RGB_G]);
				Serial.print(F(","));
				Serial.print(rgb.value[TCS230_RGB_B]);
				Serial.print(F("]"));
				break;
			}
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

	if (modeOp == LEARN_MODE)	// Learning mode
	{
		switch(runState)
		{
			case 0:	// calibrate black
				readState = fsmReadValue(readState, BLACK_CAL);
				if (readState == 0) runState++;
				break;

			case 1:	// calibrate white
				readState = fsmReadValue(readState, WHITE_CAL);
				if (readState == 0) runState++;
				break;

			case 2: // prep for learning mode
				Serial.println(F("\n\nFor each color, enter name and sense color"));
				ctIndex = 0;
				runState++;
				break;

			case 3:	// learn color values
				readState = fsmReadValue(readState, LEARN_VAL);
				if (readState == 0) {
					for (uint8_t j=0; j<RGB_SIZE; j++) {
						ct[ctIndex].rgb.value[j] = rgb.value[j];
					}
					ctIndex++;
					if (ctIndex >= ARRAY_SIZE(ct)) runState++;
				}
				break;

			case 4:	// output the text data
				outputHeader();
				runState++;
				break;
				
			default:
				runState = 0;	// start again if we get here as something is wrong
		}
	}
	else { // Matching mode
		switch(runState)
		{
			case 0:	// read a value
				readState = fsmReadValue(readState, READ_VAL);
				if (readState == 0) runState++;
				break;

			case 1:	{ // find the matching color
					uint8_t	i = colorMatch(&rgb);

					Serial.print(F("\nClosest color is "));
					Serial.print(ct[i].name);
					runState++;
			  }
				break;

			default:
				runState = 0;	// start again if we get here as something is wrong
		}
	}
}
