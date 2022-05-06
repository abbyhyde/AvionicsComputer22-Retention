/*
 * ICM20948.h
 *
 *  Created on: April 26, 2022
 *  Author: Abby Hyde
 */

#include <Wire.h>
#include <PWNServo.h>

#ifndef SRC_PERIPHERAL_SERVOMOTOR_H_
#define SRC_PERIPHERAL_SERVOMOTOR_H_

class ICM20948
{

	PWMServo servo;

    TwoWire *_wire;

public:

    ServoMotor();


private:
	uint16_t analogRes = ANALOGRES;

}

#endif /* SRC_PERIPHERAL_SERVOMOTOR_H_ */
