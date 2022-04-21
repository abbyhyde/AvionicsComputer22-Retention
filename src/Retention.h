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
//#include "peripheral/LoRaRadio.h"


/*
 * Retention has TODO primary states of autonomous operation throughout its mission which begins when the system is powered on
 *
 * STATE DETAILS
 */
enum RetentionState {

	IDLE,
	PASSIVE, 			// safe mode until massive accel
	FLIGHT, 			// until landing detected
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

	ICM20948 * imu = new ICM20948(0x68);
	MPL3115A2 * baro = new MPL3115A2();

	// orientation servo
	// arm deploy servo 1 and 2
	// quad gripper servo


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
	} * RetentionLoop = new RetentionLoop(this);		// instantiate the main system loop and pass it the system instance



	bool systemInit();
	void registerAllLoops(Looper * runningLooper);

	void zeroAllSensors();

	void beginStateMachine();
	void updateStateMachine(uint32_t timestamp);
	void endStateMachine();

};


#endif /* SRC_RETENTION_H_ */
