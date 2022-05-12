/*
 * ICM20948.h
 *
 *  Created on: April 26, 2022
 *  Author: Abby Hyde
 */

#include <Wire.h>
#include <PWMServo.h>

#ifndef SRC_PERIPHERAL_SERVOMOTOR_H_
#define SRC_PERIPHERAL_SERVOMOTOR_H_

class ServoMotor
{

	PWMServo servo;

    TwoWire *_wire;

public:

    ServoMotor(uint8_t pwmPin);
    void enable();
    void disable();
    int64_t getPosition();
    uint16_t positionToAnalog(float position);
    uint16_t speedToAnalog(float speed);
    float fMap(float x, float inMin, float inMax, float outMin, float outMax);
    void setSpeed(float speed);
    void setPosition(float position);
    void attachMotor(uint8_t pwmPin);
//    void ServoMotor::attachEncoder(uint8_t encoderPin);
//    void ServoMotor::attachEncoder(uint8_t encoderPinA, uint8_t encoderPinB);




private:
	uint16_t analogRes = 8;
	char motorPWMpin;

};

#endif /* SRC_PERIPHERAL_SERVOMOTOR_H_ */
