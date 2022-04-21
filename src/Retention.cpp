/*
 * Retention.cpp
 * Created on: April 20, 2022
 * Author: Abby Hyde
 */

#include "Retention.h"

/*
* Constructor for the robot system object, should only be one instance, one main system per processor
*/
Retention::Retention(){};


/*
 * Init function for the system, should be run after instantiation
 * Should take SPI/I2C/Serial objects as imput parameters?
 */
bool Retention::systemInit(){

	//pinMode(LED_BUILTIN, OUTPUT);		// ! will conflict with CLK if using SPI !

	// init sensors
	bool initIMU = false;
	initIMU = imu->init();

	if(!initIMU){
		Serial.println("ICM20948 does not respond");
	}
	else{Serial.println("ICM20948 is connected");}

	imu->setGyroScale(imu->getPlusMinus2000DPS());
	imu->setAccScale(imu->getPlusMinus8Gs());

	// init servos


	return true;

}

/*
 * Function to register all loops to the system looper. The looper must have the
 * total number of system loops predefined, TOTAL_LOOPS must equal the number of
 * registerLoop() calls in this function, see Constants.h
 * @param runningLooper is the looper instance of the system manager to call
 * for adding loops
 */
void Retention::registerAllLoops(Looper * runningLooper){

	runningLooper->registerLoop(robotLoop);

}


void Retention::zeroAllSensors(){

	//robotStateEstimator->reset(millis());

	//selfRighting->zeroSensors();

}


/*
 * Configuring retention subsystems for start of mission states sequence
 */
void Retention::beginStateMachine(){

	Serial.println(F("STARTED ROBOT LOOP"));
	//zeroAllSensors();

}


void Retention::updateStateMachine(uint32_t timestamp){


	//digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
	//Serial.println(millis());
	Serial.println(timestamp);

	switch (retentionState) {
		case Idle:
//			imu->readSensorData();
//			imu->printVector(imu->getGyroRawValues());

			break;
		case Passive:
			// on pad state
			// data collection from all sensors
			//imu->readSensorData();

			break;
		case Flight:
			// data collection from all sensors
			//imu->readSensorData();

			break;
		case Landed:
			// check for switch signal
			//imu->readSensorData();

			break;
		case Orientation:
			//imu->readSensorData();

			// spin servo to rotate
			// until accel is correct values

			break;
		case Unfold:
			//imu->readSensorData();

			// activate servos until set point

			break;
		case Flight_Check:
			//imu->readSensorData();

			// drone does the thing
			// receiving data?
			// check for final signal from team

			break;
		case Release:
			//imu->readSensorData();

			// servo spins to release body (quad gripper)

			break;
		default:

			break;
	}
}


void Retention::endStateMachine(){

}
