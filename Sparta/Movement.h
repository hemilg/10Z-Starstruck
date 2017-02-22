float driveMult = 1.0;
int dumpMode = 1;

int potZero = 0;
int bottomHeight = 850; // 850
int holdHeight = 1100; // 950
int topHeight = 2000; // 2900, 3100, 3300
int hangHeight = 920;  // 920

const unsigned int linearSpeed[128] =
{
 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
 0, 21, 21, 21, 22, 22, 22, 23, 24, 24,
 25, 25, 25, 25, 26, 27, 27, 28, 28, 28,
 28, 29, 30, 30, 30, 31, 31, 32, 32, 32,
 33, 33, 34, 34, 35, 35, 35, 36, 36, 37,
 37, 37, 37, 38, 38, 39, 39, 39, 40, 40,
 41, 41, 42, 42, 43, 44, 44, 45, 45, 46,
 46, 47, 47, 48, 48, 49, 50, 50, 51, 52,
 52, 53, 54, 55, 56, 57, 57, 58, 59, 60,
 61, 62, 63, 64, 65, 66, 67, 67, 68, 70,
 71, 72, 72, 73, 74, 76, 77, 78, 79, 79,
 80, 81, 83, 84, 84, 86, 86, 87, 87, 88,
 88, 89, 89, 90, 90,127,127,127
};

void gyroCalibrate()
{
    SensorType[gyro] = sensorNone;
    wait1Msec(1000);
    SensorType[gyro] = sensorGyro;
    wait1Msec(2000);
    SensorValue[gyro] = 0;
}

void lift (int pwr)
{
	motor[ORL] = motor[TRL] = motor[BRL] = motor[OLL] = motor[TLL] = motor[BLL] = pwr;
}

void setDrive (int pwr)
{
	motor[BRD] = motor[MRD] = motor[MLD] = motor[BLD] = pwr;
}

void rightDrive (int pwr)
{
	motor[BRD] = motor[MRD] = pwr;
}

void leftDrive (int pwr)
{
	motor[BLD] = motor[MLD] = pwr;
}

void claw ()
{
	SensorValue[Claw1] = (abs(SensorValue[Claw1] - 1));
	SensorValue[Claw2] = (abs(SensorValue[Claw2] - 1));
}

int cubicMap (int pwr)
{
	return linearSpeed[abs(pwr)] * sgn(pwr);
}

task DriveControl()
{
	while (1)
	{
		leftDrive((abs(vexRT[Ch3]) > 5) ? cubicMap((int)(vexRT[Ch3] * driveMult)) : 0);
		rightDrive((abs(vexRT[Ch2]) > 5) ? cubicMap((int)(vexRT[Ch2] * driveMult)) : 0);
		if (vexRT[Btn7U])
		{
			while (vexRT[Btn7U]) wait1Msec(20);
			driveMult = (driveMult == 1.0) ? 0.5 : 1.0;
		}
		wait1Msec(20);
	}
}

task LiftControl()
{
	int prev = 100;
	while (1)
	{
		if (vexRT[Btn5U] && SensorValue[pot] < (topHeight + 300))
		{
			lift((int)(127 * driveMult));
			prev = SensorValue[pot];
			dumpMode = 1;
		}
		else if (vexRT[Btn5D])
		{
			lift((int)(-127 * driveMult));
			prev = SensorValue[pot];
			dumpMode = 1;
		}
		else lift(-15);    // used to be 0
		/*{
			if (prev > holdHeight)	lift(15);
			else lift(0);
		}*/
		wait1Msec(20);
	}
}

task IntakeControl()
{
	while (1)
	{
	if (vexRT[Btn6D])
			{
				while (vexRT[Btn6D]) wait1Msec(20);
				SensorValue[Claw1] = SensorValue[Claw2] = abs(SensorValue[Claw1] - 1);
			}
			//wait1Msec(20);
		}
}

task hold()
{
	dumpMode = 2;
	stopTask(LiftControl);
	while (!(vexRT[Btn5D] || vexRT[Btn5U] || vexRT[Btn8U]))
	{
		while (SensorValue[pot] < holdHeight)
		{
			lift(127);
			wait1Msec(20);
		}
		lift(15);
		wait1Msec(20);
	}
	startTask(LiftControl);
}

void dumping()
{
	stopTask(LiftControl);
	stopTask(hold);
	while (SensorValue[pot] < topHeight)
	{
		lift(127); // 80, 60
		wait1Msec(20);
	}
	claw();
	wait1Msec(200);
	while (SensorValue[pot] > bottomHeight)
	{
		lift(-60);
		wait1Msec(20);
	}
	dumpMode = 1;
	startTask(LiftControl);
}

void hanging()
{
	stopTask(LiftControl);
	stopTask(DriveControl);
	while (SensorValue[pot] > hangHeight)
	{
		leftDrive(127);
		rightDrive(127);
		lift(-127);
		wait1Msec(20);
	}
	leftDrive(0);
	rightDrive(0);
	lift(0);
	SensorValue[HangLock] = abs(SensorValue[HangLock] - 1);
	startTask(DriveControl);
	startTask(LiftControl);
}
