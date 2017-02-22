#pragma config(Sensor, in1,    pot,            sensorPotentiometer)
#pragma config(Sensor, in8,    gyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  blinker,        sensorDigitalOut)
#pragma config(Sensor, dgtl3,  HangLock,       sensorDigitalOut)
#pragma config(Sensor, dgtl4,  Claw1,          sensorDigitalOut)
#pragma config(Sensor, dgtl5,  Claw2,          sensorDigitalOut)
#pragma config(Sensor, dgtl9,  LeftEnc,        sensorQuadEncoder)
#pragma config(Sensor, dgtl11, RightEnc,       sensorQuadEncoder)
#pragma config(Motor,  port1,           BRD,           tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           MRD,           tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           ORL,           tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port4,           TRL,           tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           BRL,           tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           BLL,           tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port7,           TLL,           tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port8,           OLL,           tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           MLD,           tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          BLD,           tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"
#include <Movement.h>
#include <Auton.h>
#include <LCD.h>

void pre_auton()
{
	startTask(selectAuton);
	fillSlew(0);
	SensorValue[blinker] = 0;
	SensorValue[Claw1] = 0; // in
	SensorValue[Claw2] = 0;
	SensorValue[HangLock] = 0; // unlocked
	resetEncoders();
	gyroCalibrate();
	SensorValue[blinker] = 1;
	SensorValue[pot] = 0;
}

void collectCenterAuto()
{
	clearTimer(T1);
	while (SensorValue[pot] < 1200)
	{
		lift(127);
		if (time1[T1] % 200 == 0)
			claw();
	}
	SensorValue[Claw1] = SensorValue[Claw2] = 0;
	while (SensorValue[pot] > bottomHeight)
	{
		lift(-80);
		wait1Msec(20);
	}
	lift(0);
	wait1Msec(500);
	SensorValue[Claw1] = 1;
	SensorValue[Claw2] = 1;
	wait1Msec(500);
	drive(1400, 60, 0.1, 0, 0);
	while(SensorValue[RightEnc] < 1100) wait1Msec(20);
	claw();
	startTask(hold);
	wait1Msec(500);
	stopTask(pidL);
	if(direction == 1)
	{
		turn(-1100, 127, 80);
	}
	else
	{
		turn(1100, 127, 80);
	}
	wait1Msec(500);
	drive(-1000, 127);
	wait1Msec(200);
	dumping();
	wait1Msec(1000);
	drive(1200, 50, 0.1, 0, 0);
	while (SensorValue[RightEnc] < 1150) wait1Msec(20);
	claw();
	wait1Msec(300);
	drive(-1300, 127);
	startTask(hold);
	dumping();

}

void knockStarsAuto()
{
	drive(-200, 127);
	startTask(hold);
	wait1Msec(500);
	drive(-1200, 127);
	dumping();

}
task autonomous()
{
	switch(autonVal){
		case 1:
			collectCenterAuto();
			break;
		case 2:
			knockStarsAuto();
			break;
		default:
			break;
	}
}

task usercontrol()
{
	stopTask(hold);
	dumpMode = 1;
	startTask(DriveControl);
	startTask(LiftControl);
	startTask(IntakeControl);
	//drive(1000, 40, 0.1, 0, 0); // works well
	//turn(900, 127);  // works well
	while (1)
	{
		if (vexRT[Btn6U])
		{
			while (vexRT[Btn6U]) wait1Msec(20);
			if (dumpMode == 1) startTask(hold);
			else
			{
				//stopTask(hold);
				dumping();
			}
		}
		if (vexRT[Btn8D])
		{
			while (vexRT[Btn8D]) wait1Msec(20);
			hanging();
		}
		if (vexRT[Btn8L])
		{
			while (vexRT[Btn8L]) wait1Msec(20);
			SensorValue[HangLock] = abs(SensorValue[HangLock] - 1);
		}
		wait1Msec(20);
		if (vexRT[Btn8R] && vexRT[Btn7R])
		{
			pre_auton();
			collectCenterAuto();
		}
	}
}
