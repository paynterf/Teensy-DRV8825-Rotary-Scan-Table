/*
	Name:       Teensy_DRV8825_Rotary_Scan_Table.ino
	Created:	6/4/2020 12:59:57 AM
	Author:     FRANKNEWXPS15\Frank
*/

/*
Teensy DRV8825 Rotary Table Program

This program controls a NEMA 17 stepper motor using a Pololu DRV8825 motor driver. It
Takes user input to define the boresight (zero) position and the start and stop angles
in degrees relative to the zero position.  Then it moves to the start position and waits
for a HIGH level on SCAN_START_PIN.  After this signal is received, the motor is
stepped through to the stop position. At the stop position, the SCAN_COMPLETE_PIN is
driven HIGH.

The Stepper motor is connected to a DRV8825 motor driver with one coil on A1/A2, and the
other coil on B1/B2.  It may turn out that one coil's leads have to be reversed to match
the motor rotation direction with the program direction.

The NEMA 17 motor is assumed to execute 200 steps/revolution.  Change the StepsPerRevolution
constant for other values.

05/27/20:  Performs an angular scan over user-supplied range of angles. By default a complete scan
from start position to stop position is performed, followed by a return to the start position
However, if the STEP_ENABLE_PIN (held HIGH by default) is pulled LOW, the program will wait
until it goes HIGH before performing the next step.  In this case, a scan iteration would go
as follows

	Step 1 - Scan parameters entered, motor moves to start position, waits for LOW on SCAN_START_PIN
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
//06/07/20 Rev to allow scan parameter changes w/o having to rebuild the project
//06/11/20 Rev to incorporate some rotation precision debugging code

#include <Wire.h>
#include "I2C_Anything.h"
#include "DRV8825.h"

const int SLAVE_ADDR = 8; //added 05/30/20

#pragma region MOTOR_CONSTANTS
 // Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
//06/02/20 now using Pololu DRV8825
const int MOTOR_STEPS_PER_REV = 200;
//const int MICROSTEPS_PER_STEP = 32;
//const int MICROSTEPS_PER_STEP = 1; //full step mode
//const int MICROSTEPS_PER_STEP = 2; //half step mode
//const int MICROSTEPS_PER_STEP = 8;
const int MICROSTEPS_PER_STEP = 32;
const int MICROSTEPS_PER_REV = MICROSTEPS_PER_STEP * MOTOR_STEPS_PER_REV;
const float DEG_PER_MICROSTEP = 360 /(float)MICROSTEPS_PER_REV;
const int DEFAULT_RPM = 6;
//const int POSITIONING_SPEED_RPM = 100;
const int POSITIONING_SPEED_RPM = 20;
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

//06/07/20 added for new scan parameter edit feature
bool bRepeating = true;
double scanStepDeg = 0; 
int scanTotDeg = 0;

//06/08/20 added for rotator precision debug
int motorStepsPerScanStep;
int motorStepsToStartPos;
int curMotorSteps = 0;


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
	stepper.setMicrostep(MICROSTEPS_PER_STEP);   // Set microstep mode



	Serial.begin(115200);
	while (!Serial)
	{
		delay(10);
	}

	delay(5000); //05/28/20 delay AFTER Serial object instantiation is NECESSARY!

	//Serial.printf("Serial available after %lu millisec\n",millis() - now); 

	pinMode(SCAN_START_PIN, INPUT_PULLUP);
	pinMode(SCAN_STEP_ENABLE_PIN, INPUT_PULLUP);
	pinMode(SCAN_STEP_COMPLETE_PIN, OUTPUT);
	pinMode(SCAN_COMPLETE_PIN, OUTPUT);

	//init SCAN_COMPLETE_PIN, SCAN_STEP_COMPLETE to LOW (not complete) state
	digitalWrite(SCAN_COMPLETE_PIN, LOW);
	digitalWrite(SCAN_STEP_COMPLETE_PIN, LOW);

	//added 05/26/20
	Serial.printf("NEMA-17 Stepper Rotary Table Program\n");

}

void loop()
{
	//06/07/20 moved everything into loop() so can repeat scans with parameter edit if desired

	//06/07/20 rev to use a function for scan parameter capture
	GetScanParameters();

	//06/02/20 Now using Pololu DRV8825 driver & microstepping
	scanTotDeg = abs(scanStopDeg - scanStartDeg);
	scanStepDeg = (float)scanTotDeg / (float)scanNumberofSteps;
	motorStepsPerScanStep = roundf(scanStepDeg / DEG_PER_MICROSTEP);
	Serial.printf("scanTotDeg = %d, scanStepDeg = %2.3f, motorStepsPerScanStep = %d\n", scanTotDeg, scanStepDeg, motorStepsPerScanStep);

	//go to scan start pos
	stepper.setRPM(POSITIONING_SPEED_RPM);
	stepper.rotate(scanStartDeg);
	motorStepsToStartPos = roundf(scanStartDeg / DEG_PER_MICROSTEP);
	Serial.printf("scanStartDeg = %d\n", scanStartDeg);
	Serial.printf("Start Position is %d steps and %d deg from zero position\n", motorStepsToStartPos, scanStartDeg);
	Serial.printf("scanStartDeg = %d\n", scanStartDeg);
	//stepper.move(motorStepsToStartPos);
	curMotorSteps = motorStepsToStartPos; //safe to overwrite previous value here

	//while (1)  //user has opportunity to exit after each scan
	bRepeating = true;
	while (bRepeating)  //user has opportunity to exit after each scan
	{
		//wait for trigger
		Serial.printf("waiting for LOW on SCAN_START_PIN (pin %d) ...\n", SCAN_START_PIN);
		while (digitalRead(SCAN_START_PIN) == HIGH)
		{
			delay(100);
		}

		Serial.printf("Scan Triggered - starting scan, setting SCAN_COMPLETE_PIN (pin %d) to LOW\n", SCAN_COMPLETE_PIN);
		digitalWrite(SCAN_COMPLETE_PIN, LOW);

		stepper.setRPM(scanSpeedRpm);

		for (curAngleStepVal = 1; curAngleStepVal <= scanNumberofSteps; curAngleStepVal++)
		{
			curRelPointingAngleDeg = scanStartDeg + (curAngleStepVal)*scanStepDeg;

			//when SCAN_STEP_ENABLE_PIN goes LOW, go to first/next step.
			while (digitalRead(SCAN_STEP_ENABLE_PIN))
			{

			}

			digitalWrite(SCAN_STEP_COMPLETE_PIN, LOW);
			Serial.printf("Scan Step %d Triggered: moving to %3.2f deg\n", curAngleStepVal, curRelPointingAngleDeg);

			//DRV8825
			stepper.rotate(scanStepDeg); //rotate to next scan angle
			//stepper.move(motorStepsPerScanStep);
			curMotorSteps += motorStepsPerScanStep;
			//Serial.printf("Motor now at %d steps\n", curMotorSteps);
			Serial.printf("Motor now at %d steps and %3.2f degrees\n", curMotorSteps, curRelPointingAngleDeg);

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
		Serial.printf("After scan completion motor at %d steps\n", curMotorSteps);

		digitalWrite(SCAN_COMPLETE_PIN, HIGH);

		//return to start position
		//DRV8825
		delay(1000);
		stepper.setRPM(POSITIONING_SPEED_RPM);
		//stepper.rotate(-scanTotDeg);
		Serial.printf("motorStepsToStartPos = %d, curMotorSteps = %d, steps to start pos = %d\n",
			motorStepsToStartPos, curMotorSteps, motorStepsToStartPos - curMotorSteps);
		stepper.move(motorStepsToStartPos - curMotorSteps);
		curMotorSteps = motorStepsToStartPos; //reset curMotorSteps

		//re-initialize step counter and current pointing angle
		curAngleStepVal = 0;
		curRelPointingAngleDeg = scanStartDeg;

		Serial.printf("Enter R to repeat this scan, N for a new scan, Q to quit the program\n");
		while (!Serial.available())
		{
		}

		//06/07/2020 rev to allow user to Repeat the current scan, start a new scan, or quit
#pragma region USER_INPUT
		int incomingByte;
		bool bDoneWithInput = false;
		while (!bDoneWithInput)
		{
			if (Serial.available() > 0)
			{
				// read the incoming byte:
				incomingByte = Serial.read();

				// say what you got:
				//Serial.printf("I received: 0x%x\n", incomingByte);

				//02/18/20 experiment with multiple commands
				switch (incomingByte)
				{
				case 0x52: //ASCII 'R' Repeat current scan
				case 0x72: //ASCII 'r'
					Serial.printf("Repeat Current Scan\n");
					bRepeating = true;
					bDoneWithInput = true;
					break;
				case 0x4E: //ASCII 'N' New Scan
				case 0x6E: //ASCII 'n'
					Serial.printf("New Scan\n");
					bRepeating = false;
					bDoneWithInput = true;
					break;
				case 0x51: //ASCII 'Q' Quit the program
				case 0x71: //ASCII 'q'
					Serial.printf("Stopping Program - Have a Nice Day!\n");
					while (1);
					break;
				default:
					Serial.printf("Please enter a Q, R, or N\n");
					bDoneWithInput = false;
					bDoneWithInput = false;
					break;
				}

				//clear out any remaining characters from Serial buffer
				while (Serial.available())
				{
					incomingByte = Serial.read();
					//Serial.printf("Consuming 0x%x\n", incomingByte);
				}
			}

		}
#pragma endregion Check for incoming character

	}
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

//06/07/20 added to encapsulate scan parameter acquisition
void GetScanParameters()
{
	//get motor speed to be used
	scanSpeedRpm = GetIntegerParameter("Motor speed RPM", 6);


	//set zero position
	bStopPosDone = false;
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

}

