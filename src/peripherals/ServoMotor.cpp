/*
 * ServoMotor.cpp
 *
 *  Created on: April 26, 2022
 *      Author: Abby Hyde
 */

#include "ServoMotor.h"

/*
 * Constructor for ServoMotor using a potentiometer as a rotary encoder for feedback
 * @param pwmPin is the pin delivering a PWM signal for the motor controller
 * @param directionPin is the direction toggling for the motor controller
 * @param currentSensorPin is the analog input pin for the motor controller current feedback
 * @param encoderPin is the analog input pin for the potentiometer feedback
 */
ServoMotor::ServoMotor(uint8_t pwmPin) {

	motorPWMpin = pwmPin;
}

void ServoMotor::enable() {

	servo.attach(motorPWMpin);

	//attachMotor(motorPWMpin);
	//setPosition(0);


}

void ServoMotor::disable() {

}

//void ServoMotor::update() {
//
//}

int64_t ServoMotor::getPosition() {

}

uint16_t ServoMotor::positionToAnalog(float position) {

	uint16_t setpoint = position * 8;

	return setpoint;
}

/*
 * This method is called from setSpeed() to convert speed values from -1 to 1 into analog values.
 * @param speed is the desired motor speed from -1 (full speed clockwise) to 1 (full speed counter-clockwise).
 */
uint16_t ServoMotor::speedToAnalog(float speed) {

	uint16_t setpoint = abs(speed) * analogRes;

	if (setpoint > analogRes) {
		setpoint = analogRes;
	}

	else if (setpoint < 0) {
		setpoint = 0;
	}

	return setpoint;
}
/*
 * Mapping fuction used to map a value, x, between two values, outMin and outMax, based on what the inMin and inMax values are.
 * @param x is the input value to be mapped.
 * @param inMin is the lowest expected input value that x should be.
 * @param inMax is the highest expected input value that x should be.
 * @param outMin is the lowest value of desired output range.
 * @param outMax is the highest value of desired output range.
 */
float ServoMotor::fMap(float x, float inMin, float inMax, float outMin, float outMax) {

	if (x > inMax) {
		return outMax;
	}

	if (x < inMin) {
		return outMin;
	}

	return ((x - inMin) * (outMax - outMin) / (inMax - inMin)) + outMin;
}

/*
 * Sets the motor's speed and direction using a value from -1 to 1.
 * Calls speedToAnalog() function to convert a speed value from -1 to 1 to an analog value.
 * @param speed is the desired motor speed from -1 (full speed clockwise) to 1 (full speed counter-clockwise).
 * if speed parameter is 0 the motor controler will not permit to motor to spin.
 */
void ServoMotor::setSpeed(float speed) {

	uint16_t setpoint = speedToAnalog(speed);

//	Serial.println(setpoint);

	analogWrite(motorPWMpin, setpoint);

//	if (speed < 0){
//		digitalWrite(motorDirPin1, LOW);
//		digitalWrite(motorDirPin2, HIGH);
//	}
//	else if (speed > 0){
//		digitalWrite(motorDirPin1, HIGH);
//		digitalWrite(motorDirPin2, LOW);
//	}
//	else {
//		digitalWrite(motorDirPin1, LOW);
//		digitalWrite(motorDirPin2, LOW);
//	}

}

void ServoMotor::setPosition(float position) {

	//analogWrite(motorPWMpin, position);

	servo.write(position);

}

///*
// * Bind the proper motor signal pin, uses pinMode() and
// * The analog linear potentiometer pin is set
// * This function should not be run in a constructor, only during or after setup()
// * @param encoderPin is the analog input pin for the potentiometer feedback
// */
//void ServoMotor::attachEncoder(uint8_t encoderPin) {
//
//	//potPin
//	// analog pins are input by default, doesn't hurt to pinMode tho
//	//TODO analogReadResolution
//	//TODO check to make sure the pin is a proper analog input pin, print error if not?
//
//}


///*
// * Bind the proper motor signal pin, uses pinMode() and
// * The (quadrature) encoder pins are set on instantiation of an encoder object.
// * This function should not be run in a constructor, only during or after setup()
// * @param encoderPinA is the encoder signal pin A
// * @param encoderPinB is the encoder signal pin B
// */
//void ServoMotor::attachEncoder(uint8_t encoderPinA, uint8_t encoderPinB) {
//
//	// binding the quad encoder signal pins, Encoder class uses pinMode() in its constructor,
//	// making it necessary to instantiate it in the systemInit which runs after setup()
//	encoder = new Encoder(encoderPinA, encoderPinB);
//
//}


/**
 * Bind the proper motor signal and direction pins, uses pinMode()
 * This function should not be run in a constructor, only during or after setup()
 * @param pwmPin is the pin delivering a PWM signal for the motor controller
 * @param directionPin1 is the first direction toggling pin for the motor controller
 * @param directionPin2 is the second direction toggling pin for the motor controller
 */
void ServoMotor::attachMotor(uint8_t pwmPin) {

	pinMode(pwmPin, OUTPUT);
//	pinMode(directionPin1, OUTPUT);
//	pinMode(directionPin2, OUTPUT);

}


///**
// * Bind the proper motor signal pin for direct current measurement, uses pinMode()
// * @param currentSensorPin is the analog input pin for the motor controller current feedback
// */
//void ServoMotor::attachCurrentSensor(uint8_t currentSensorPin) {
//
//	//currentSensor
//	// analog pins are input by default, doesn't hurt to pinMode tho
//	//TODO analogReadResolution
//	//TODO check to make sure the pin is a proper analog input pin, print error if not?
//
//}

