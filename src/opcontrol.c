/** @file opcontrol.c
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

 /*
  * Runs the user operator control code. This function will be started in its own task with the
  * default priority and stack size whenever the robot is enabled via the Field Management System
  * or the VEX Competition Switch in the operator control mode. If the robot is disabled or
  * communications is lost, the operator control task will be stopped by the kernel. Re-enabling
  * the robot will restart the task, not resume it from where it left off.
  *
  * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will
  * run the operator control task. Be warned that this will also occur if the VEX Cortex is
  * tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
  *
  * Code running in this task can take almost any action, as the VEX Joystick is available and
  * the scheduler is operational. However, proper use of delay() or taskDelayUntil() is highly
  * recommended to give other tasks (including system tasks such as updating LCDs) time to run.
  *
  * This task should never exit; it should end with some kind of infinite loop, even if empty.
  */
#include "main.h"
#include "chassis.h"
#include "claw.h"
#include "math.h"
#include "position.h"
// elbow encoder ratio 0.5
//shoulder encoder ratio 0.6

void operatorControl() {
	int power, turn;
  Encoder shoulderEnc;
	Encoder elbowEnc;
	shoulderEnc = encoderInit(8, 9, false);
	elbowEnc = encoderInit(6, 7, false);
	encoderReset(shoulderEnc);
	encoderReset(elbowEnc);

	//encoder initialize
	pinMode(3, INPUT); //shoulder
	pinMode(4, INPUT); //elbow
	//limit switch initialize
	int error;
	int error2;

	int target;
	int lock = encoderGet(shoulderEnc);

//a1 = 74.3666;
//a2 = -31.982;


	delay(20);
	while (1) {

			power = joystickGetAnalog(1, 3); // vertical axis on left joystick
			turn  = joystickGetAnalog(1, 4); // horizontal axis on left joystick
			chassisSet(-(power + turn), -(power - turn));
		  clawSet(joystickGetAnalog(1,2), joystickGetAnalog(1,1));

			if (joystickGetDigital(1,5,JOY_UP)) {//elbow
		    motorSet(6,127);
		  } else if (joystickGetDigital(1,5,JOY_DOWN)) {
		    motorSet(6,-127);
		  } else {
		    motorSet(6,0);
		  }


		  if (joystickGetDigital(1,6,JOY_UP)) {
		    motorSet(5,127);//shoulder
		    lock = encoderGet(shoulderEnc);
		  } else if (joystickGetDigital(1,6,JOY_DOWN)) {
		    motorSet(5,-127);
		    lock = encoderGet(shoulderEnc);
		  } else {
		    motorSet(5,-3*(lock - encoderGet(shoulderEnc)));
			}

			if(joystickGetDigital(1,7,JOY_LEFT)) {//shoulder homing
				while(digitalRead(5) == HIGH) {
			      motorSet(5,50);
			  }
			  motorSet(5,0);
				encoderReset(shoulderEnc);
				while(encoderGet(shoulderEnc) < 104) {
					motorSet(5,-50);
				}
				motorSet(5,0);
				encoderReset(shoulderEnc);
				lock = encoderGet(shoulderEnc);
			}

			if(joystickGetDigital(1,7,JOY_UP)) { //elbow homing
				while(digitalRead(3) == HIGH) {
			      motorSet(6,-50);
			  }
			  motorSet(6,0);
				encoderReset(elbowEnc);
				while(encoderGet(elbowEnc) > -190) {
					motorSet(6,50);
				}
				motorSet(6,0);
				encoderReset(elbowEnc);
			}


			if (joystickGetDigital(1,7,JOY_RIGHT)) {
				int x1 = 20;
				int y1 = 0;
				int l1 = 11;
				int l2 = 14;
				double a2;
				double a1;
				for(int x = 0; x < 5; x++) {	
					a2 = position1(x1,y1,l1,l2);
					a1 = position2(a2,x1,y1,l1,l2);
					a2 += a1;
					a1 *= (180/M_PI);
					a2 *= (180/M_PI);
				}
				error = (0.6*encoderGet(shoulderEnc) - a1);
				error2 = (0.5*encoderGet(elbowEnc) - a2);
				if((error < 42) && (error > -42)) {
					motorSet(5,error*3);
				} else if(error >= 42) {
					motorSet(5,127);
				} else if(error <= -42) {
					motorSet(5,-127);
				} else {
					motorSet(5,0);
				}

				if((error2 < 42) && (error2 > -42)) {
					motorSet(6,error2*3);
				} else if(error2 >= 42) {
					motorSet(6,127);
				} else if(error2 <= -42) {
					motorSet(6,-127);
				} else {
					motorSet(6,0);
				}

			}
// precondition()^ needs to be homed first

			delay(20);
	}
}
