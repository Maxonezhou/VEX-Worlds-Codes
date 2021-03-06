#pragma config(Sensor, in1,    FourBarPot,     sensorPotentiometer)
#pragma config(Sensor, in2,    MobGoalPot,     sensorPotentiometer)
#pragma config(Sensor, in3,    LeftArmPot,     sensorPotentiometer)
#pragma config(Sensor, in4,    RightArmPot,    sensorPotentiometer)
#pragma config(Sensor, dgtl1,  ,               sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  ,               sensorQuadEncoder)
#pragma config(Motor,  port1,           GoliathIntake, tmotorVex393_HBridge, PIDControl, encoderPort, dgtl3)
#pragma config(Motor,  port2,           LeftWheels,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           RightWheels,   tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           LeftArmLiftBottom, tmotorVex393_MC29, openLoop, encoderPort, None)
#pragma config(Motor,  port5,           RightArmLiftBottom, tmotorVex393_MC29, openLoop, encoderPort, None)
#pragma config(Motor,  port6,           LeftFourBar,   tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           RightArmLiftTop, tmotorVex393_MC29, openLoop, encoderPort, None)
#pragma config(Motor,  port8,           RightFourBar,  tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           LeftArmLiftTop, tmotorVex393_MC29, openLoop, encoderPort, None)
#pragma config(Motor,  port10,          MobileGoalLift, tmotorVex393_HBridge, PIDControl, reversed, encoderPort, dgtl3)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VEX EDR                      */
/*                                                                           */
/*---------------------------------------------------------------------------*/

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton()
{
  // Set bStopTasksBetweenModes to false if you want to keep user created tasks
  // running between Autonomous and Driver controlled modes. You will need to
  // manage all user created tasks if set to false.
  bStopTasksBetweenModes = true;

	// Set bDisplayCompetitionStatusOnLcd to false if you don't want the LCD
	// used by the competition include file, for example, you might want
	// to display your team name on the LCD in this function.
	// bDisplayCompetitionStatusOnLcd = false;

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task autonomous()
{	motor[LeftArmLiftTop] = 127;
	motor[LeftArmLiftBottom] = -127;
	motor[RightArmLiftTop] = -127;
	motor[RightArmLiftBottom] = 127;
	wait1Msec(2000);
	motor[LeftArmLiftTop] = 0;
	motor[LeftArmLiftBottom] = 0;
	motor[RightArmLiftTop] = 0;
	motor[RightArmLiftBottom] = 0;
	motor[MobileGoalLift] = 127;
	wait1Msec(2150);
	motor[LeftWheels] = 127;
	motor[RightWheels] = 127;
	motor[MobileGoalLift] = 127;
	wait1Msec(1575);
	motor[MobileGoalLift] = -127;
	motor[LeftWheels] = 0;
	motor[RightWheels] = 0;
	wait1Msec(3750);
	motor[LeftWheels] = -127;
	motor[RightWheels] = -127;
	wait1Msec(500);
  motor[LeftWheels] = 127;
  motor[RightWheels] = -127;
  motor[MobileGoalLift] = 0;
  wait1Msec(1100);
  motor[LeftWheels] = 127;
  motor[RightWheels] = 127;
  wait1Msec(600);
  motor[LeftWheels] = 0;
  motor[RightWheels] = 0;
  motor[MobileGoalLift] = 127;
  wait1Msec(3200);
  motor[LeftWheels] = -127;
  motor[RightWheels] = -127;
  motor[MobileGoalLift] = 0;
  wait1Msec(500);
}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task usercontrol()
{

	int threshold = 10;

	while (true)
	{motor[LeftWheels] = vexRT[Ch3];
	 motor[RightWheels] = vexRT[Ch2];

		if (abs(vexRT[Ch3]) > threshold)
		{motor[LeftWheels]  = (vexRT[Ch3])/2;
		}

		else
		{motor[LeftWheels]  = 0;
		}

		if (abs(vexRT[Ch2]) > threshold)
		{motor[RightWheels] = (vexRT[Ch2])/2;
		}

		else
		{motor[RightWheels] = 0;
		}

		if (vexRT[Btn6U] == 1)
		{motor[MobileGoalLift] = 127;
		}

		else if (vexRT[Btn5U] == 1)
		{motor[MobileGoalLift] = -127;
		}

		else
		{motor[MobileGoalLift] = 0;
		}

		if	(vexRT[Btn5UXmtr2] == 1)
		{ motor[LeftArmLiftTop] = 127;
			motor[LeftArmLiftBottom] = -127;
			motor[RightArmLiftTop] = -127;
			motor[RightArmLiftBottom] = 127;
		}

		else if (vexRT[Btn5DXmtr2] == 1)
		{ motor[LeftArmLiftTop] = -127;
			motor[RightArmLiftTop] = 127;
			motor[LeftArmLiftBottom] = 127;
			motor[RightArmLiftBottom] = -127;
		}

		else
		{ motor[LeftArmLiftTop] = 0;
			motor[RightArmLiftTop] = 0;
			motor[LeftArmLiftBottom] = 0;
			motor[RightArmLiftBottom] = 0;
		}

		if (vexRT[Btn7DXmtr2] == 1)
		{ motor[GoliathIntake] = 127;
		}

		else if (vexRT[Btn8DXmtr2] == 1)
		{ motor[GoliathIntake] = -127;
		}

		else
		{ motor[GoliathIntake] = 0;
		}

		if (vexRT[Btn6DXmtr2] == 1)
		{ motor[LeftFourBar] = 127;
			motor[RightFourBar] = 127;
		}

		else if (vexRT[Btn6UXmtr2] == 1)
		{ motor[LeftFourBar] = -65;
			motor[RightFourBar] = -65;
		}
		else
		{ motor[LeftFourBar] = 0;
			motor[RightFourBar] = 0;
		}
  }
}
