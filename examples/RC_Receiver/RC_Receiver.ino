/*
 *  Project     Servo Input Library
 *  @author     David Madison
 *  @link       github.com/dmadison/ServoInput
 *  @license    LGPLv3 - Copyright (c) 2020 David Madison
 *
 *  This example is part of the Arduino Servo Input Library.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Example:      RC_Receiver
 *  Description:  Reads the steering and throttle servo signals from an RC
 *                receiver and prints them over Serial: steering as an angle
 *                from -90 to 90, and throttle as an integer percentage for
 *                both forwards and reverse.
 *
 *  Wiring:       Servo signals to pin 32 (steering) and pin 35 (throttle)
 */

#include <ServoInput.h>

/* Signal pins for ServoInput MUST be interrupt-capable pins!
 *     Uno, Nano, Mini (328P): 2, 3
 *     Micro, Leonardo (32U4): 0, 1, 2, 3, 7
 *     Mega, Mega2560: 2, 3, 18, 19, 20, 21
 *     Lolin32 (ESP32): all pins
 * Reference: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
 */

// Steering Setup
const int SteeringSignalPin = 32;  // MUST be interrupt-capable!
const int SteeringPulseMin = 1277;  // microseconds (us)
const int SteeringPulseMax = 1720;  // Ideal values for your servo can be found with the "Calibration" example

ServoInputPin<SteeringSignalPin> steering(SteeringPulseMin, SteeringPulseMax);

//Throttle Setup
const int ThrottleSignalPin = 35;  // MUST be interrupt-capable!
const int ThrottlePulseMin = 1196;  // microseconds (us)
const int ThrottlePulseMax = 1807;  // Ideal values for your servo can be found with the "Calibration" example

ServoInputPin<ThrottleSignalPin> throttle(ThrottlePulseMin, ThrottlePulseMax);

void setup() {
	Serial.begin(115200);

	//attach interupts to servoinput lib as a workaround, as the internal interrupt setup doesn't work.
    pinMode(SteeringSignalPin, INPUT_PULLUP);
    pinMode(ThrottleSignalPin, INPUT_PULLUP);

	//attach interupts to servoinput lib as a workaround, as the internal interrupt setup doesn't work.
    attachInterrupt(digitalPinToInterrupt(SteeringSignalPin), steering.isr, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ThrottleSignalPin), throttle.isr, CHANGE);
}

void loop() {
	Serial.print("RC - ");

	float steeringAngle = 90.0 - steering.getAngle();  // returns 0 - 180, subtracting from 90 to center at "0" and invert for "normal" steering
	Serial.print("Steering: ");
	Serial.print(steeringAngle);
	Serial.print("deg");

	int steeringPulse = steering.getPulse();
	Serial.print(" Pulseln: ");
	Serial.print(steeringPulse);


	Serial.print(" | ");  // separator

	int throttlePercent = throttle.map(100, -100);  // remap to a percentage both forward and reverse
	Serial.print("Throttle: ");
	Serial.print(throttlePercent);
	Serial.print("% ");

	int throttlePulse = throttle.getPulse();
	Serial.print(" Pulseln: ");
	Serial.print(throttlePulse);

	if (throttlePercent >= 0) {
		Serial.print("(Forward)");
	}
	else {
		Serial.print("(Reverse)");
	}

	Serial.println();
}
