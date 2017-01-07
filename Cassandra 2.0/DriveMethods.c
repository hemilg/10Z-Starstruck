void lift (int pwr)
{
	motor[Lift1] = motor[Lift2] = motor[Lift3] = motor[Lift4] = motor[Lift5] = pwr;
}

void claw (int pwr)
{
	motor[Claw] = pwr;
}
task DriveControl()
{
	int x = 0; //strafe
	int y = 0; //forward+back
	int r = 0; //rotate
	while (1)
	{
		x = abs(vexRT[Ch4]) > 5 ? vexRT[Ch4] : 0;
		y = abs(vexRT[Ch3]) > 5 ? vexRT[Ch3] : 0;
		r = abs(vexRT[Ch1]) > 5 ? vexRT[Ch1] : 0;

		motor[LF] = r + x + y;
		motor[LB] = r - x + y;
		motor[RF] = r + x - y;
		motor[RB] = r - x - y ;

		wait1Msec(20);
	}
}
task LiftControl()
{
	while (1)
	{
		lift(vexRT[Btn6U] ? 127 : (vexRT[Btn6D] ? -127 : 0));
		claw(vexRT[Btn5U] ? -127 : (vexRT[Btn5D] ? 127 : 0));
	}
}

void driveX (int pwr, int clicks)
{
	pwr = (clicks > 0 ? pwr : (-pwr));
	SensorValue[ELF] = 0;
	while (clicks  > abs(SensorValue[ELF])
	{
		motor[LF] = motor[RF] = pwr;
		motor[RB] = motor[LB] = -pwr;
	}
}

void driveY (int pwr, int clicks)
{
	pwr = (clicks > 0 ? pwr : (-pwr));
	SensorValue[ELF] = 0;
	while (clicks  > abs(SensorValue[ELF])
	{
		motor[LF] = motor[LB] = pwr;
		motor[RF] = motor[RB] = -pwr;
	}
}

void turn (int pwr, int deg)
{
	pwr = (deg > 0 ? pwr : (-pwr));
	while (deg < abs(SensorValue[gyro]))
	{
		motor[LB] = motor[RB] = motor[RF] = motor[LF] = pwr;
	}
}

void dump ()
{
	stopTask(LiftControl);
	lift(127);
	wait1Msec(1500);
	lift(0);
	claw(-1217);
	wait1Msec(1000);
	claw(0);
	startTask(LiftControl);
}
