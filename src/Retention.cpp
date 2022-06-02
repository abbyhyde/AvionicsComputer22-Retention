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
	else{
		Serial.println("ICM20948 is connected");

	}

	imu->setGyroScale(imu->getPlusMinus2000DPS());
	imu->setAccScale(imu->getPlusMinus8Gs());
	imu->calibrateGyro();
//
//	baro->init();
//	baro->setModeAltimeter();
//	baro->setOverSampleRate(0);
//	delay(50);							// let barometer start up

	// init servos
	orientation->enable();
	Serial.println("enabled servo");

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

	runningLooper->registerLoop(retentionLoop);

}


void Retention::zeroAllSensors(){



}

void Retention::collectSensorData() {

	imu->readSensorData();
	baro->readSensorData();

	if (retentionState == PASSIVE | retentionState == ROCKET_FLIGHT) {
		imu->complementaryFilter(false);
	} else {
		imu->complementaryFilter(true);
	}

	// log it in SD?
}


/*
 * Configuring retention subsystems for start of mission states sequence
 */
void Retention::beginStateMachine(){

	Serial.println(F("STARTED RETENTION LOOP"));
	//zeroAllSensors();

}


void Retention::updateStateMachine(uint32_t timestamp){

	collectSensorData();
	//digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
	//Serial.println(millis());
//	Serial.println(timestamp);
	Serial.println(retentionState);

	switch (retentionState) {
		case IDLE:

//			float currentPressure = baro->getPressure();
//			Serial.println(currentPressure);
//
//			float currentTemperature = baro->getTemperature();
//			Serial.println(currentTemperature);

			break;

		case PASSIVE:
			// on pad state
			// passive to flight = lots of accel
			currentAccel = imu->getAccZ();

			if (currentAccel == launchThreshold) {
				retentionState = ROCKET_FLIGHT;
				Serial.println("Switching to ROCKET_FLIGHT");
			}

			break;

		case ROCKET_FLIGHT:
			// flight to landed = lots of decel
			currentAccel = imu->getAccZ();

			if (currentAccel == landedThreshold) {
				retentionState = LANDED;
				Serial.println("Switching to LANDED");
			}

			break;

		case LANDED:
			// check for switch signal
			// landed to orientation = switch signal
			Serial.println("Switching to ORIENTATION");
			retentionState = ORIENTATION;
			pos = 0;

			break;

		case ORIENTATION:
			// spin servo to rotate until accel is correct values
			currentRoll = imu->getPitch();
			if (currentRoll > 0) {						// probably needs to be some sort of range but idk how sensitive the measurements are
				pos++;
				orientation->setPosition(pos);
			} else if (currentRoll < 0) {
				pos--;
				orientation->setPosition(pos);
			} else {
				Serial.println("Switching to UNFOLD");
				retentionState = UNFOLD;
				pos = 0;
			}

			break;

		case UNFOLD:
			// activate servos until set point
			if (pos < armDeployThreshold) {
				pos += 1; 						// this can be changed depending on how fast you want the servo to move and release the arms
				armMotor1->setPosition(pos);
				armMotor2->setPosition(pos);
			} else if (pos > armDeployThreshold || pos == armDeployThreshold) {
				retentionState = FLIGHT_CHECK;
				Serial.println("Switching to FLIGHT_CHECK");
				pos = 0;
			}

			break;

		case FLIGHT_CHECK:
			// drone does the thing
			// receiving data?
			// check for final signal from team
			Serial.println("Switching to RELEASE");
			retentionState = RELEASE;

			break;

		case RELEASE:
			// servo spins to release body (quad gripper)
			if (pos < armDeployThreshold) {
				pos += 1; 						// this can be changed depending on how fast you want the servo to move and release the arms
				quadGripper->setPosition(pos);
			} else if (pos > armDeployThreshold || pos == armDeployThreshold) {
				Serial.println("Switching to IDLE");
				retentionState = IDLE;
				pos = 0;
			}

			break;

		default:

			break;
	}
}


void Retention::endStateMachine(){

}
