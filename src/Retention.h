/*
 * Retention.h
 * Created on: April 20, 2022
 * Author: Abby Hyde
 */

#ifndef SRC_RETENTION_H
#define SRC_RETENTION_H_

#include "Arduino.h"

#include "../Constants.h"
#include "SystemInterface.h"
#include "loops/Looper.h"
#include "loops/loop.h"

#include "peripherals/ICM20948.h"
#include "peripherals/MPL3115A2.h"
#include "peripherals/ServoMotor.h"
//#include "peripheral/LoRaRadio.h"

// Servo Motor Pins
#define ORIENTATION_MOTOR 9
#define ARM_RELEASE_MOTOR_1 10
#define ARM_RELEASE_MOTOR_2 11
#define QUAD_GRIPPER_MOTOR 12

#define ICM_ADDRESS 0x69


/*
 * Retention has TODO primary states of autonomous operation throughout its mission which begins when the system is powered on
 *
 * STATE DETAILS
 */
enum RetentionState {

	IDLE,
	PASSIVE, 			// safe mode until massive accel
	ROCKET_FLIGHT, 		// until landing detected
	LANDED,				// until switch is triggered manually
	ORIENTATION,		// until rotation complete
	UNFOLD, 			// until quad arms are out
	FLIGHT_CHECK, 		// complete pre-flight check
	RELEASE				// until quad is fully released

};

/*
 * The Retention object, instantiates all retention subsystems and registers their loops
 *
 * DETAILS
 *
 */
class Retention : public SystemInterface {

private:

	RetentionState retentionState = IDLE;			// initial system state is IDLE

	ICM20948 * imu = new ICM20948(ICM_ADDRESS);

	MPL3115A2 * baro = new MPL3115A2();

	// orientation servo
	ServoMotor * orientation = new ServoMotor(ORIENTATION_MOTOR);
	// arm deploy servo 1 and 2
	ServoMotor * armMotor1 = new ServoMotor(ARM_RELEASE_MOTOR_1);
	ServoMotor * armMotor2 = new ServoMotor(ARM_RELEASE_MOTOR_2);
	// quad gripper servo
	ServoMotor * quadGripper = new ServoMotor(QUAD_GRIPPER_MOTOR);

	uint8_t pos = 0;
	int16_t currentAccel;				// current accel value for PASSIVE and ROCKET_FLIGHT states
	float currentRoll;					// current roll value for ORIENTATION state

	int16_t launchThreshold = 10; 		// int16_t to be consistent with the accel val
	int16_t landedThreshold = -10;
	uint8_t armDeployThreshold = 100; 	// the angle the servo needs to move to in order to release the quad arms

public:

	Retention();

	/* Retention loop functionality */
	class RetentionLoop : public Loop {
		Retention * retention_;

	public:
		RetentionLoop(Retention * instance){
			retention_ = instance;
		};

		void onStart(uint32_t timestamp){
			retention_->beginStateMachine();
		}
		void onLoop(uint32_t timestamp){
			retention_->updateStateMachine(timestamp);

		}
		void onStop(uint32_t timestamp){
			retention_->endStateMachine();
		}
	} * retentionLoop = new RetentionLoop(this);		// instantiate the main system loop and pass it the system instance



	bool systemInit();
	void registerAllLoops(Looper * runningLooper);

	void zeroAllSensors();
	void collectSensorData();

	void beginStateMachine();
	void updateStateMachine(uint32_t timestamp);
	void endStateMachine();

};


#endif /* SRC_RETENTION_H_ */
