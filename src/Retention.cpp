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

	pos = 0;
	up = true;
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

	//robotStateEstimator->reset(millis());

	//selfRighting->zeroSensors();

}

void Retention::collectSensorData() {

	imu->readSensorData();
	imu->complementaryFilter();
	baro->readSensorData();

//	int16_t accelZ = imu->getAccZ();

//	Serial.println(imu->getRoll());

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

	switch (retentionState) {
		case IDLE:
			//Serial.println(imu->getPitch());
			orientation->setPosition(pos);
			if (up && pos != 255) {
				pos++;
			} else if (up && pos == 255) {
				up = false;
				pos--;
			} else if (!up && pos != 0) {
				pos--;
			} else {
				up = true;
				pos++;
			}
			Serial.println(pos);

//			float currentPressure = baro->getPressure();
//			Serial.println(currentPressure);
//
//			float currentTemperature = baro->getTemperature();
//			Serial.println(currentTemperature);

			break;
		case PASSIVE:
			// on pad state
			// data collection from all sensors
			// passive to flight = lots of accel

			break;
		case FLIGHT:
			// data collection from all sensors
			// flight to landed = lots of decel

			break;
		case LANDED:
			// check for switch signal
			// landed to orientation = switch signal

			break;
		case ORIENTATION:

			// spin servo to rotate
			// until accel is correct values

			break;
		case UNFOLD:

			// activate servos until set point

			break;
		case FLIGHT_CHECK:

			// drone does the thing
			// receiving data?
			// check for final signal from team

			break;
		case RELEASE:

			// servo spins to release body (quad gripper)

			break;
		default:

			break;
	}
}


void Retention::endStateMachine(){

}
