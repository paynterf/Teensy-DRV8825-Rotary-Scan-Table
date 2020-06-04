/*
    Name:       Teensy_DRV8825_Rotary_Scan_Table.ino
    Created:	6/4/2020 12:59:57 AM
    Author:     FRANKNEWXPS15\Frank
*/

/*
	Name:       Teensy_NEMA17_L298N_RotaryTable.ino
	Created:	5/27/2020 11:00:40 AM
	Author:     FRANKNEWXPS15\Frank
*/

/*
Teensy NEMA 17 L298N Rotary Table Program

This program controls a NEMA 17 stepper motor using an L298N motor driver. It
Takes user input to define the boresight (zero) position and the start and stop angles
in degrees relative to the zero position.  Then it moves to the start position and waits
for a HIGH level on SCAN_START_PIN.  After this signal is received, the motor is
stepped through to the stop position. At the stop position, the SCAN_COMPLETE_PIN is
driven HIGH.

The Stepper motor is connected to an L298N motor driver with one coil on Out1/2, and the
other coil on Out3/4.  It may turn out that one coil's leads have to be reversed to match
the motor rotation direction with the program direction.

The Teensy 3.2 is connect to In1/2/3/4 via pins 8,9,10,11 although this can be changed in the
constructor below.

The NEMA 17 motor is assumed to execute 200 steps/revolution.  Change the StepsPerRevolution
constant for other values.

05/27/20:  Performs an angular scan over user-supplied range of angles. By default a complete scan
from start position to stop position is performed, followed by a return to the start position
However, if the STEP_ENABLE_PIN (held HIGH by default) is pulled LOW, the program will wait
until it goes HIGH before performing the next step.  In this case, a scan iteration would go
as follows

	Step 1 - Scan paramaters entered, motor moves to start position, waits for LOW on SCAN_START_PIN
	Step 2 - Measurement program pulls SCAN_START_PIN LOW for 1 sec, then back HIGH
	Step 3 - Measurement program takes a measurement & retrieves step# & angle value via I2C request
	Step 4 - Measurement program pulls SCAN_STEP_ENABLE_PIN LOW
	Step 5 - Motor moves to first/next step, and outputs HIGH on SCAN_STEP_COMPLETE_PIN
	Step 6 - Measurement program waits for HIGH on SCAN_STEP_COMPLETE_PIN
	Step 7 - Measurement program takes a measurement & retrieves step# & angle value via I2C request
	Step 8 - Repeat Steps 4-7 until scan complete
	Step 9 - SCAN_COMPLETE_PIN goes HIGH
*/

//O5/30/20 Added I2C connection to pass step# & relative angle values to measurement program
//05/02/20 Rev to use Pololu DRV8825 & micro-stepping

//#include <Stepper.h>
#include <Wire.h>
#include "I2C_Anything.h"
#include "DRV8825.h"

const int SLAVE_ADDR = 8; //added 05/30/20

#pragma region MOTOR_CONSTANTS
 // Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
//06/02/20 now using Pololu DRV8825
const int MOTOR_STEPS_PER_REV = 200;
const int MICROSTEPS_PER_STEP = 32;
const int MICROSTEPS_PER_REV = MICROSTEPS_PER_STEP * MOTOR_STEPS_PER_REV;
const int DEG_PER_MICROSTEP = 360 / MICROSTEPS_PER_REV;
const int MICROSTEPS_PER_DEG = MICROSTEPS_PER_REV / 360;
const int DEFAULT_RPM = 12;
const int POSITIONING_SPEED_RPM = 100;
const int DEFAULT_MOTOR_STEPS_TO_MOVE = 10;
#pragma endregion Motor Constants

#pragma region PIN_ASSIGNMENTS
const int DIR = 8;
const int STEP = 9;
const int SLEEP = 13; // optional (just delete SLEEP from everywhere if not used)

const int MODE0 = 10;
const int MODE1 = 11;
const int MODE2 = 12;

const int SCAN_START_PIN = 2;
const int SCAN_STEP_ENABLE_PIN = 3;
const int SCAN_STEP_COMPLETE_PIN = 4;
const int SCAN_COMPLETE_PIN = 5;
#pragma endregion Pin Assignments

DRV8825 stepper(MOTOR_STEPS_PER_REV, DIR, STEP, SLEEP, MODE0, MODE1, MODE2);

char Instr[20];

//DRV8825
int zeroPos;
int scanStartDeg;
int scanStartMicroSteps; //scanStartDeg converted to steps
int scanStopDeg;
int scanStopSteps; //scanStopDeg converted to steps
int scanNumberofSteps; //added 05/27/20
int stepsToMove; //steps to move to initial position
int curMotorStepVal; //keeps track of current position, in steps
float curRelPointingAngleDeg; //keeps track of current relative pointing angle, in degrees
int curAngleStepVal; //keeps track of current table step iteration value
bool bStopPosDone = false;
int scanSpeedRpm = 6;



void setup()
{
	//added 05/30/20 to provide step# & angle values to Angle Scan program
	Wire.begin(SLAVE_ADDR);
	Wire.setSCL(18);
	Wire.setSCL(19);
	Wire.setClock(100000);
	Wire.onRequest(requestEvent);

	//06/02/20 now using Pololu DRV8825 microstepping motor driver
	stepper.begin(DEFAULT_RPM);
	// if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
	// stepper.setEnableActiveState(LOW);
	stepper.enable();

	/*
	 * Microstepping mode: 1, 2, 4, 8, 16 or 32 (where supported by driver)
	 * Mode 1 is full speed.
	 * Mode 32 is 32 microsteps per step.
	 * The motor should rotate just as fast (at the set RPM),
	 * but movement precision is increased, which may become visually apparent at lower RPMs.
	 */
	stepper.setMicrostep(MICROSTEPS_PER_STEP);   // Set microstep mode to 1:32



	Serial.begin(115200);
	unsigned long now = millis();
	while (!Serial)
	{
		delay(10);
	}

	delay(5000); //05/28/20 delay AFTER Serial object instantiation is NECESSARY!

	Serial.print("Serial available after "); Serial.print(millis() - now); Serial.print(" millisec");

	pinMode(SCAN_START_PIN, INPUT_PULLUP);
	pinMode(SCAN_STEP_ENABLE_PIN, INPUT_PULLUP);
	pinMode(SCAN_STEP_COMPLETE_PIN, OUTPUT);
	pinMode(SCAN_COMPLETE_PIN, OUTPUT);

	//init SCAN_COMPLETE_PIN, SCAN_STEP_COMPLETE to LOW (not complete) state
	digitalWrite(SCAN_COMPLETE_PIN, LOW);
	digitalWrite(SCAN_STEP_COMPLETE_PIN, LOW);

	//added 05/26/20
	Serial.printf("Serial.printf: NEMA-17 Stepper Rotary Table Program\n");

	//get motor speed to be used
	scanSpeedRpm = GetIntegerParameter("Motor speed RPM", 6);


	//set zero position
	Serial.println("Set stepper zero position.  Enter +steps for CW, -steps for CCW, Q to exit");
	while (!bStopPosDone)
	{
		while (Serial.available() == 0); //waits for input
		if (toascii(Serial.peek()) == 'Q' || toascii(Serial.peek()) == 'q')
		{
			bStopPosDone = true;
			curMotorStepVal = 0;
		}
		else
		{
			stepsToMove = GetIntegerParameter("Steps to move (- = CCW, + = CW)?", DEFAULT_MOTOR_STEPS_TO_MOVE);
			Serial.print("Steps to move = "); Serial.println(stepsToMove);
			stepper.setRPM(POSITIONING_SPEED_RPM);
			stepper.move(stepsToMove);
		}
	}

	Serial.readString(); //clear the input buffer - could still have chars in it
	Serial.print("curMotorStepVal = "); Serial.println(curMotorStepVal);

	//set scan start/stop positions in degrees
	scanStartDeg = GetIntegerParameter("Scan Start Pos in Degrees Relative to Zero", -90);
	scanStopDeg = GetIntegerParameter("Scan Stop Pos in Degrees Relative to Zero", 90);
	scanNumberofSteps = GetIntegerParameter("Number of Steps", 18);
	curRelPointingAngleDeg = scanStartDeg; //added 05/30/20

	Serial.println("Run Parameters:");
	Serial.print("Speed: "); Serial.print(scanSpeedRpm); Serial.println(" RPM");
	Serial.print("Scan Start: "); Serial.print(scanStartDeg); Serial.println(" Deg");
	Serial.print("Scan Stop: "); Serial.print(scanStopDeg); Serial.println(" Deg");
	Serial.print("Scan Steps: "); Serial.println(scanNumberofSteps);

	//06/02/20 Now using Pololu DRV8825 driver & microstepping
	int scandeg = abs(scanStopDeg - scanStartDeg);
	double scanstepdeg = scandeg / scanNumberofSteps;

	//go to scan start pos
	stepper.setRPM(POSITIONING_SPEED_RPM);
	stepper.rotate(scanStartDeg);

	while (1)  //user has opportunity to exit after each scan
	{
		//wait for trigger
		Serial.printf("waiting for LOW on SCAN_START_PIN (pin %d) ...\n", SCAN_START_PIN);
		while (digitalRead(SCAN_START_PIN) == HIGH)
		{
			delay(100);
		}

		Serial.println("Scan Triggered - starting scan, setting SCAN_COMPLETE_PIN to LOW\n");
		digitalWrite(SCAN_COMPLETE_PIN, LOW);

		stepper.setRPM(scanSpeedRpm);

		for (curAngleStepVal = 1; curAngleStepVal <= scanNumberofSteps; curAngleStepVal++)
		{
			curRelPointingAngleDeg = scanStartDeg + (curAngleStepVal)*scanstepdeg;

			//when SCAN_STEP_ENABLE_PIN goes LOW, go to first/next step.
			while (digitalRead(SCAN_STEP_ENABLE_PIN))
			{

			}

			unsigned long now = millis();
			digitalWrite(SCAN_STEP_COMPLETE_PIN, LOW);
			Serial.printf("Scan Step %d Triggered: moving to %3.2f deg\n", curAngleStepVal, curRelPointingAngleDeg);

			//DRV8825
			stepper.rotate(scanstepdeg); //rotate to next scan angle

			//output HIGH on SCAN_STEP_COMPLETE_PIN
//DEBUG!!
			//Serial.printf("Outputting HIGH on SCAN_STEP_COMPLETE_PIN\n");
			//Serial.printf("Done in %lu Msec - Current Angle = %3.2f\n", millis() - now, curRelPointingAngleDeg);
//DEBUG!!

			digitalWrite(SCAN_STEP_COMPLETE_PIN, HIGH);
			//delay(50);  //make the HIGH period visible on scope
		}

		//quit
		Serial.printf("Outputting HIGH on SCAN_COMPLETE_PIN (pin %d)\n", SCAN_COMPLETE_PIN);

		digitalWrite(SCAN_COMPLETE_PIN, HIGH);

		//return to start position
		//DRV8825
		stepper.setRPM(POSITIONING_SPEED_RPM);
		stepper.rotate(-scandeg);

		//re-initialize step counter and current pointing angle
		curAngleStepVal = 0;
		curRelPointingAngleDeg = scanStartDeg;

		Serial.printf("Enter any key to continue\n");
		while (!Serial.available())
		{

		}

		//consume bytes until Serial buffer is empty to avoid unintended triggers
		int incomingByte;
		while (Serial.available())
		{
			incomingByte = Serial.read();
			Serial.printf("Consuming 0x%x\n", incomingByte);
		}

	}
}


void loop()
{
}

int GetIntegerParameter(String prompt, int defaultval)
{
	int param = 0;
	bool bDone = false;

	while (!bDone)
	{
		Serial.print(prompt); Serial.print(" ("); Serial.print(defaultval); Serial.print("): ");
		while (Serial.available() == 0); //waits for input
										 //String res = Serial.readString().trim();
		String res = Serial.readString();
		res.trim();

		int reslen = res.length();
		if (reslen == 0) //user entered CR only
		{
			bDone = true;
			param = defaultval;
		}
		else
		{
			res.toCharArray(Instr, reslen + 1);
			//if (isNumeric(Instr) && atoi(Instr) >= 0)
			if (isNumeric(Instr))
			{
				param = atoi(Instr);
				bDone = true;
			}
			else
			{
				Serial.print(Instr); Serial.println(" Is invalid input - please try again");
			}
		}
	}
	Serial.println(param);
	return param;
}

// check a string to see if it is numeric and accept Decimal point
//copied from defragster's post at https://forum.pjrc.com/threads/27842-testing-if-a-string-is-numeric
bool isNumeric(char* str)
{
	byte ii = 0;
	bool RetVal = false;
	if ('-' == str[ii])
		ii++;
	while (str[ii])
	{
		if ('.' == str[ii]) {
			ii++;
			break;
		}
		if (!isdigit(str[ii])) return false;
		ii++;
		RetVal = true;
	}
	while (str[ii])
	{
		if (!isdigit(str[ii])) return false;
		ii++;
		RetVal = true;
	}
	return RetVal;
}

//05/30/20 added to report current angle step# & relative pointing angle to measurement program
void requestEvent()
{
	//send the current step# and current relative pointing angle
//DEBUG!!
	//Serial.printf("In requestEvent: curAngleStep = %d, curRelAngle = %2.2f\n", 
	//	curAngleStepVal, curAngleStepVal, curRelPointingAngleDeg, curRelPointingAngleDeg);
//DEBUG!!

	I2C_writeAnything(curAngleStepVal);
	I2C_writeAnything(curRelPointingAngleDeg);
}