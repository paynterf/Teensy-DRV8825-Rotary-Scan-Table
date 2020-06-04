# Teensy-DRV8825-Rotary-Scan-Table
Teensy 3.2 with Pololu DRV8825 and a stepper motor to perform rotary scans
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
