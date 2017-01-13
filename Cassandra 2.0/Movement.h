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
		if (vexRT[Btn6U] && vexRT[Btn5D]) claw(50);
		else claw(vexRT[Btn5U] ? -127 : (vexRT[Btn5D] ? 127 : 0));
		wait1Msec(20);
	}
}

void setDrive (int pwr)
{
	motor[LF] = motor[RF] = motor[LB] = motor[RB] = 0;
}
void driveXTime (int pwr, int time)
{
		motor[LF] = motor[RF] = pwr;
		motor[RB] = motor[LB] = -pwr;
		wait1Msec(time);
		setDrive(0);
}

void driveYTime (int pwr, int time)
{
		motor[LF] = motor[LB] = pwr;
		motor[RF] = motor[RB] = -pwr;
		wait1Msec(time);
		setDrive(0);
}
void driveX (int pwr, int clicks)
{
	pwr = (clicks > 0 ? pwr : (-pwr));
	SensorValue[ELF] = 0;
	while (clicks  > abs(SensorValue[ELF])
	{
		motor[LF] = motor[RF] = pwr;
		motor[RB] = motor[LB] = -pwr;
		wait1Msec(20);
	}
	setDrive(0);

}

void driveY (int pwr, int clicks)
{
	pwr = (clicks > 0 ? pwr : (-pwr));
	SensorValue[ELF] = 0;
	while (clicks  > abs(SensorValue[ELF])
	{
		motor[LF] = motor[LB] = pwr;
		motor[RF] = motor[RB] = -pwr;
		wait1Msec(20);
	}
	setDrive(0);
}

void turn (int pwr, int deg)
{
	pwr = (deg > 0 ? pwr : (-pwr));
	while (deg < abs(SensorValue[gyro]))
	{
		motor[LB] = motor[RB] = motor[RF] = motor[LF] = pwr;
		wait1Msec(20);
	}
	setDrive(0);
}

void dump ()
{
	stopTask(LiftControl);

	// Pulse claw closed twice
	for(int i = 0; i < 2; i++){
		claw(127);
		wait1Msec(150);
		claw(0);
		wait1Msec(100);
	}
	claw(50);

	// Lift up dump
	lift(127);
	wait1Msec(250);
	// drive backwards
	wait1Msec(750);
	// stop drive
	claw(-127);
	wait1Msec(300);
	lift(0);
	wait1Msec(500);
	claw(0);

	lift(-127);
	wait1Msec(1000);
	claw(127)
	wait1Msec(250);

	startTask(LiftControl);
}

void dumping ()
{
	stopTask(LiftControl);
	clearTimer(T1);
	while (SensorValue[limit] == 1 && time1[T1] < 1200)
	{
		Lift(127);
		claw(50);
		wait1Msec(20);
	}
	claw(-127);
	wait1Msec(200);
	Lift(0);
	wait1Msec(750);
	claw(0);
	clearTimer(T1);
	while (SensorValue[bottom] == 1 && time1[T1] < 1200)
	{
		Lift(-100);
		wait1Msec(20);
	}
	Lift(0);
	claw(127);
	wait1Msec(150);
	claw(0);
	startTask(LiftControl);
}
void startLift ()
{
	lift(127);
	wait1Msec(250);
	lift(0);
	wait1Msec(250);
	lift(-127);
	wait1Msec(250);
	lift(0);
}
void auto1 () // pick up three in middle
{
	stopAllTasks();
	claw(127);
	wait1Msec(500);
	claw(0);
	driveYTime(80, 500);
	claw(80);
	driveYTime(80, 1000);
	claw(0);
	startTask(DriveControl);
	startTask(LiftControl);
}

void auto2 () // knock four in middle
{
	stopAllTasks();
	claw(127);
	wait1Msec(500);
	claw(0);
	driveYTime(80, 800);
	driveXTime(80, 500);
	lift(127);
	wait1Msec(600);
	lift(0);
	driveYTime(80, 200);
	startTask(DriveControl);
	startTask(LiftControl);
}
