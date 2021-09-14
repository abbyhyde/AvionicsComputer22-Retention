/*
 * Robot.h
 * Created on: Sep 12, 2021
 * Author: Peter Dentch
 */

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

#include "Arduino.h"

//#include "../Constants.h"
#include "loops/Looper.h"
#include "loops/loop.h"

//#include "subsystems/DriveTrain.h"

//#include "peripheral/GyroAccel.h"
//#include "peripheral/LoRaRadio.h"


/*
 * Robot has TODO primary states of autonomous operation throughout its mission which begins when the system is powered on
 *
 * STATE DETAILS
 */
enum RobotState {

	TESTING,
	Waiting,
	Driving
};

/*
 * The Robot object, instantiates all robot subsystems and registers their loops
 *
 * DETAILS
 *
 */
class Robot {

private:

	Looper * runningLooper;					// looper instance so robot can register its subsystem loops on startup

	RobotState robotState;					// initial state is Landed


//	MPU6050 * robotIMU = new MPU6050();



//	DriveTrain * driveTrain = new DriveTrain(robotIMU);



public:

	Robot(Looper * looper);
	//	Robot(){}

	/* Robot loop functionality */
	class RobotLoop : public Loop {
		Robot * robot_;

	public:
		RobotLoop(Robot * instance){
			robot_ = instance;
		};

		void onStart(uint32_t timestamp){
			robot_->beginStateMachine();
		}
		void onLoop(uint32_t timestamp){
			robot_->updateStateMachine();

		}
		void onStop(uint32_t timestamp){
			robot_->endStateMachine();
		}
	} * robotLoop = new RobotLoop(this);		// instantiate the main system loop and pass it the system instance



	void systemInit();

	void zeroAllSensors();

	void beginStateMachine();
	void updateStateMachine();
	void endStateMachine();

};




#endif /* SRC_ROBOT_H_ */
