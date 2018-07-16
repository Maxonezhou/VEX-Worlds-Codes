#pragma config(Sensor, in1,    ClawArmPotL,    sensorPotentiometer)
#pragma config(Sensor, in2,    ClawArmPotR,    sensorPotentiometer)
#pragma config(Sensor, in3,    ArmPotL,        sensorPotentiometer)
#pragma config(Sensor, in4,    ArmPotR,        sensorPotentiometer)
#pragma config(Sensor, in5,    MobilePotL,     sensorPotentiometer)
#pragma config(Sensor, dgtl9,  EncRight,       sensorQuadEncoder)
#pragma config(Sensor, dgtl11, EncLeft,        sensorQuadEncoder)
#pragma config(Motor,  port1,           Right2,        tmotorVex393_HBridge, openLoop, reversed, driveLeft, encoderPort, dgtl11)
#pragma config(Motor,  port2,           Left1,         tmotorVex393_MC29, openLoop, driveLeft, encoderPort, dgtl11)
#pragma config(Motor,  port3,           Roller,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           ClawArmL,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           TL,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           TR,            tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           ClawArmR,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           MobileGoalL,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           Right1,        tmotorVex393_MC29, openLoop, reversed, driveRight, encoderPort, dgtl9)
#pragma config(Motor,  port10,          Left2,         tmotorVex393_HBridge, openLoop, driveRight, encoderPort, dgtl9)


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

int optimalL;
int optimalR;
int MoptimalL;
int CoptimalL;
int CoptimalR;


task PID_ClawLift()
{
	float ckp = 0.24; //tune
	float cki = 0.004; //tune
	float ckd = 0.28; //tune

	float ccurrentL ;
	float cerrorTl;
	float clastErrorL = 0;
	float ccurrentR ;
	float cerrorTr;
	float clastErrorR = 0;
	float cproportionL;
	float cintegralL;
	float cderivativeL;
	float cproportionR;
	float cintegralR;
	float cderivativeR;

	while(true){


		float cerrorL = CoptimalL - SensorValue[ClawArmPotL];
		float cerrorR = CoptimalR - SensorValue[ClawArmPotR];



		if (abs(cerrorL) < 200 && cerrorL !=0)
		{
			cerrorTl = cerrorL;
		}
		else{
			cerrorTl = 0;
		}

		if (abs(cerrorR) < 200 &&cerrorR !=0)
		{
			cerrorTr = cerrorR;
		}
		else{
			cerrorTr = 0;
		}

		if (abs(cerrorTl)> 50 / cki){
			cerrorTl = 50 / cki;
		}

		if (cerrorL ==0){
			cderivativeL = 0;
		}

		if (abs(cerrorTr)> 50 / cki){
			cerrorTr = 50 / cki;
		}

		if (cerrorR ==0){
			cderivativeR = 0;
		}




		cproportionL = cerrorL * ckp;
		cproportionR = cerrorR * ckp;
		cintegralL = cerrorTl  * cki;
		cintegralR = cerrorTr  * cki;
		cderivativeL = (cerrorL - clastErrorL) * ckd;
		cderivativeR = (cerrorR - clastErrorR) * ckd;

		clastErrorL = cerrorL;
		clastErrorR = cerrorR;

		ccurrentL = cproportionL + cintegralL + cderivativeL;
		ccurrentR = cproportionR + cintegralR + cderivativeR;

		motor[ClawArmR] = -ccurrentR;
		motor[ClawArmL] = -ccurrentL;



		wait1Msec(40);

	}
	return;
}
task PID_MG()
{
	float mkp = 0.3; //tune
	float mki = 0.001; //tune
	float mkd = 0.2; //tune

	float mcurrentL ;
	float merrorTl;
	float mlastErrorL = 0;
	float mproportionL;
	float mintegralL;
	float mderivativeL;
	//float mcurrentR ;
	//float merrorTr;
	//float mlastErrorR = 0;
	//float mproportionR;
	//float mintegralR;
	//float mderivativeR;

	while(true){


		float merrorL = (MoptimalL - SensorValue[MobilePotL]);
		//float merrorR = (MoptimalR - SensorValue[MobilePotR]);



		if (abs(merrorL) < 200 && merrorL !=0)
		{
			merrorTl = merrorL;
		}
		else{
			merrorTl = 0;
		}

		/*if (abs(merrorR) < 200 && merrorR !=0)
		{
		merrorTr = merrorR;
		}
		else{
		merrorTr = 0;
		}*/

		if (abs(merrorTl)> 50 / mki){
			merrorTl = 50 / mki;
		}

		if (merrorL ==0){
			mderivativeL = 0;
		}

		/*if (abs(merrorTr)> 50 / mki){
		merrorTr = 50 / mki;
		}
		*/
		/*if (merrorR ==0){
		mderivativeR = 0;
		}*/




		mproportionL = merrorL * mkp;
		//mproportionR = merrorR * mkp;
		mintegralL = merrorTl  * mki;
		//mintegralR = merrorTr  * mki;
		mderivativeL = (merrorL - mlastErrorL) * mkd;
		//mderivativeR = (merrorR - mlastErrorR) * mkd;

		mlastErrorL = merrorL;
		//mlastErrorR = merrorR;

		mcurrentL = mproportionL + mintegralL + mderivativeL;
		//mcurrentR = mproportionR + mintegralR + mderivativeR;

		//motor[MobileGoalR] = mcurrentR;
		motor[MobileGoalL] = mcurrentL;



		wait1Msec(40);

	}
	return;
}

task PID_RD4B()
{
	float kp = 0.5; //tune
	float ki = 0.0001; //tune
	float kd = 0.4; //tune

	float currentL ;
	float errorTl;
	float lastErrorL = 0;
	float proportionL;
	float integralL;
	float derivativeL;
	float currentR ;
	float errorTr;
	float lastErrorR = 0;
	float proportionR;
	float integralR;
	float derivativeR;

	while(true){


		float errorL = optimalL - SensorValue[ArmPotL];
		float errorR = optimalR - SensorValue[ArmPotR];
		//PidAtTargetL = abs(SensorValue[ArmPotL] - optimalL) < 50;
		//PidAtTargetR = abs(SensorValue[ArmPotR] - optimalR) < 50;


		if (abs(errorL) < 200 && errorL !=0)
		{
			errorTl = errorL;
		}
		else{
			errorTl = 0;
		}

		if (abs(errorR) < 200 && errorR !=0)
		{
			errorTr = errorR;
		}
		else{
			errorTr = 0;
		}

		if (abs(errorTl)> 50 / ki){
			errorTl = 50 / ki;
		}

		if (errorL ==0){
			derivativeL = 0;
		}

		if (abs(errorTr)> 50 / ki){
			errorTr = 50 / ki;
		}

		if (errorR ==0){
			derivativeR = 0;
		}




		proportionL = errorL * kp;
		proportionR = errorR * kp;
		integralL = errorTl  * ki;
		integralR = errorTr  * ki;
		derivativeL = (errorL - lastErrorL) * kd;
		derivativeR = (errorR - lastErrorR) * kd;

		lastErrorL = errorL;
		lastErrorR = errorR;

		currentL = proportionL + integralL + derivativeL;
		currentR = proportionR + integralR + derivativeR;

		motor[TL] = -currentR;
		motor[TR] = -currentL;



		wait1Msec(40);

	}
	return;
}



int drive_optimalL;
int drive_optimalR;

task PID_Drive()
{

	float kp = 0.7; //tune
	float ki = 0.0001; //tune
	float kd = 0.8; //tune
	//float ks = 1.0; //tune

	//ks is the autostraight constant.
	float currentL = 0;
	float errorTl;
	float lastErrorL;
	float proportionL;
	float integralL;
	float derivativeL;
	float currentR = 0;
	float errorTr;
	float lastErrorR;
	float proportionR;
	float integralR;
	float derivativeR;
	SensorValue[EncLeft] = 0;
	SensorValue[EncRight] = 0;

	//clear the enc values first)

	while(true){


		float errorL = drive_optimalL - SensorValue[EncLeft];
		float errorR = drive_optimalR - SensorValue[EncRight];
		if (abs(errorL) < 1000 && errorL !=0)
		{
			errorTl = errorL;
		}
		else{
			errorTl = 0;
		}

		if (abs(errorR) < 1000 && errorR !=0)
		{
			errorTr = errorR;
		}
		else{
			errorTr = 0;
		}

		if (abs(errorTl)> 50 / ki){
			errorTl = 50 / ki;
		}

		if (errorL ==0){
			derivativeL = 0;
		}

		if (abs(errorTr)> 50 / ki){
			errorTr = 50 / ki;
		}

		if (errorR ==0){
			derivativeR = 0;
		}



		proportionL = errorL * kp;
		proportionR = errorR * kp;
		integralL = errorTl  * ki;
		integralR = errorTr  * ki;
		derivativeL = (errorL - lastErrorL) * kd;
		derivativeR = (errorR - lastErrorR) * kd;

		lastErrorL = errorL;
		lastErrorR = errorR;

		currentL = proportionL + integralL + derivativeL;
		currentR = proportionR + integralR + derivativeR;

		motor[Right1] = motor [Right2] = currentR;
		motor[Left1] = motor [Left2] = currentL;



		wait1Msec(40);
	}
}





//secondary functions to simplify the coding process
//not sure how the waitUntil funcitons work


void drive(int LeftDrive, int RightDrive){
	SensorValue (EncLeft) = SensorValue (EncRight) = 0;
	drive_optimalL = LeftDrive;
	drive_optimalR = RightDrive;
	//waitUntil (abs(SensorValue[EncLeft]) < LeftDrive - 50);
	wait1Msec (50);
	return;
}

void RD4B (int a, int b){
	optimalL = a;
	optimalR = b;
	waitUntil ( abs(SensorValue[ArmPotL]) > abs(a - 100));
}


void MG (int x){
	MoptimalL = x;

	waitUntil ( abs(SensorValue[MobilePotL]) > abs(x - 100));
}
void clawlift (int g, int h){
	CoptimalL = g;
	CoptimalR = h;
	waitUntil ( abs(SensorValue[ClawArmPotL]) > abs(g - 100));
}
void Roll (int q){
	motor[Roller] = q;
	wait1Msec(500);
	motor[Roller] = 0;
}

//---------------------------------------------------------------------------------------------
task a1 (){
	RD4B (2315,2110);
	return;
}

task a2 () {
	wait1Msec (300);
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (1400, 1400);

	return;
}
task a3 () {
	wait1Msec(300);
	MG (2922);
	wait1Msec(500);
	return;
}

void a (){
	startTask (a1 );
	startTask (a2 );
	startTask (a3 );
	wait1Msec (2300);
	stopTask (a1 );
	stopTask (a2 );
	stopTask (a3 );
	return;
}
//-----------------------------------------------------------------------------------------------
void b1 () {
	MG (896);
	wait1Msec (900);
	return;
}
task b2 () {

	RD4B (2752,2447);
	wait1Msec (500);
	return;
}
task b3 () {
	wait1Msec(500);
	Roll(120);
	wait1Msec (300);
	return;
}

//-------------------------------------------------------------------------------------------------------
task c1b () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1285, -1285);
	wait1Msec (1600);
	return;
}
void c(){
	startTask(b2);
	startTask(b3);
	startTask(c1b);
	wait1Msec(1600);
	stopTask(b2);
	stopTask(b3);
	stopTask(c1b);
	return;
}
void c1 () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-180, 180);
	wait1Msec (400);
	return;
}
void c2 (){
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-1200, -1200);
	wait1Msec (1600);
	return;
}
void c3 () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-450, 450);
	wait1Msec (700);
	return;
}
//---------------------------------------------------
void f1 (){
	RD4B (2342,2106);
	wait1Msec (300);
	return;
}

task e (){
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (1000, 1000);
	wait1Msec (1200);
	return;

}
//-------------------------------------------------------






task f4a () {
	MG (1647);
	wait1Msec(600);
	return;
}



void f2a(){
	wait1Msec(400);
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-500, -500);
	wait1Msec (700);
	return;
}

void f4b (){
	startTask(f4a);
	startTask(e);
	wait1Msec(1000);
	stopTask(f4a);
	stopTask(e);
	return;
}
/*
void f3() {
wait1Msec(500);
MG (287);
wait1Msec(200);
}
*/
void f4 () {
	SensorValue[EncLeft] = SensorValue [EncRight] = 0;
	drive (-500, -500);
	wait1Msec (500);
	return;
}

task autonomous()
{
	optimalL = SensorValue [ArmPotL];
	optimalR = SensorValue [ArmPotR];
	MoptimalL = SensorValue [MobilePotL];
	//MoptimalR = SensorValue [MobilePotR];
	CoptimalL = SensorValue [ClawArmPotL];
	CoptimalR = SensorValue [ClawArmPotR];
	startTask(PID_RD4B);
	startTask(PID_Drive);
	startTask(PID_MG);
	startTask(PID_ClawLift);
	a();
	b1();

	c();
	c1();
	c2();
	c3();
	f1();


	f4b();
	f2a();
	//	f2a();
	//	f3();
	f4();
	stopTask(PID_RD4B);
	stopTask(PID_Drive);
	stopTask(PID_ClawLift);
	stopTask (PID_MG);
	motor[Right1] = motor [Right2] = motor[Left1] = motor [Left2] = 0;
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




	CoptimalL = SensorValue [ClawArmPotL];
	CoptimalR = SensorValue [ClawArmPotR];



	startTask (PID_ClawLift);


	while(true){

		if(vexRT[Btn7DXmtr2] == 1)
		{
			//while((vexRT[Btn7DXmtr2] == 1)) {
			motor[Roller] = 127;

			//wait1Msec (40);
			//}
		}
		else if(vexRT[Btn8DXmtr2] == 1)
		{
			//while(vexRT[Btn8DXmtr2] == 1) {
			motor[Roller] = -127;

			//wait1Msec (40);
			//}
		}

		else {
			motor[Roller] = 25;
		}



		if(vexRT[Btn5UXmtr2] == 1)
		{
			motor[TL] = motor [TR] = 120;

		}

		else if (vexRT[Btn5DXmtr2] == 1) {
			// 	while (vexRT[Btn5UXmtr2] == 1){
			motor[TL] = motor [TR] = -120;

		}
		//}


		else{
			motor[TL] = motor [TR] = 0;
		}

		if ((vexRT [Btn6DXmtr2] == 1)) {
			// motor[ClawArmL] = motor[ClawArmR] = 120;
			CoptimalL = 313;
			CoptimalR = 249;
		}

		else if ((vexRT[Btn6UXmtr2] == 1)){
			CoptimalL = 2950;
			CoptimalR = 2880;

		}
		else {

		}


		if ((vexRT [Btn5U] == 1)) {
			motor[MobileGoalL] = -120;
		}

		else if (vexRT [Btn6U] == 1) {
			motor[MobileGoalL] = 120;
		}
		else {
			motor [MobileGoalL] = 0;
		}


		//---------------------------------------------------------------------------------------------------------------
		float ch3_drive;
		float ch1_drive;
		float a = vexRT[Ch3] / 127.0;
		float b = vexRT[Ch1] / 127.0;
		ch3_drive = (0.463 * (pow(a,3))- (0.069 * pow(a,2)) + (0.614 * a) ) * 127;
		ch1_drive = (0.463 * (pow(b,3))- (0.069 * pow(b,2)) + (0.614 * b) ) * 127;


		motor[Left1]  = ((ch3_drive) + (ch1_drive));
		motor[Left2]  = ((ch3_drive) + (ch1_drive));
		motor[Right1] = ((ch3_drive) - (ch1_drive));
		motor[Right2] = ((ch3_drive) - (ch1_drive));


		wait1Msec(40);
	}
}
