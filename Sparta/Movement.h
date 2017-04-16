float driveMult = 1.0;
int dumpMode = 1;

int bottomHeight = 980; // 850
int holdHeight = 1100; // 950
int clawHeight = 2500; // 2000
int topHeight = 3400;
int hangHeight = 1000;  // 1300

int hangEnter = 3440;

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

const int SLEW_LENGTH = 10;

int Slew[SLEW_LENGTH];
task dumping();
task dumpOverride();

void potValues(int zero)
{
	int offset = zero + 180 - (bottomHeight); // +100
	bottomHeight += offset;
	holdHeight += offset;
	clawHeight += offset;
	topHeight += offset;
	hangHeight += offset;
	hangEnter += offset;
}

void fillSlew(int val)
{
	for(int j = 0; j < SLEW_LENGTH; j++)
	{
		Slew[j] = val;
	}
}

void resetEncoders()
{
	SensorValue[LeftEnc] = 0;
	SensorValue[RightEnc] = 0;
}

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

void slewLift(int pwr)
{
	int sum;
	for(int j = 1; j < SLEW_LENGTH; j++)
	{
		Slew[j-1] = Slew[j];
	}
	Slew[SLEW_LENGTH-1] = pwr;
	for(int j = 0; j < SLEW_LENGTH; j++)
	{
		sum += Slew[j];
	}
	lift(sum/SLEW_LENGTH);
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

void claw (int val)
{
	SensorValue[Claw1] = SensorValue[Claw2] = val;
}

int cubicMap (int pwr)
{
	return pwr;
	//return linearSpeed[abs(pwr)] * sgn(pwr);
}

task DriveControl()
{
	while (1)
	{
		leftDrive((abs(vexRT[Ch3]) > 15) ? cubicMap((int)(vexRT[Ch3] * driveMult)) : 0);
		rightDrive((abs(vexRT[Ch2]) > 15) ? cubicMap((int)(vexRT[Ch2] * driveMult)) : 0);
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
		else
		{
			if (SensorValue[pot] > holdHeight) lift(8);
			else lift(-20);/*lift(SensorValue[pot] > (bottomHeight + 10) ? -20 : 0); // -8*/
		}
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
	while (!(vexRT[Btn5D] || vexRT[Btn5U] || vexRT[Btn6U]))
	{
		while (SensorValue[pot] < holdHeight)
		{
			lift(127);
			wait1Msec(20);
		}
		lift(8);
		wait1Msec(20);
	}
	startTask(LiftControl);
}

task dumpOverride()
{
	while (1)
	{
		if (vexRT[Btn5U] || vexRT[Btn5D])
		{
			stopTask(dumping);
			dumpMode = 1;
			startTask(LiftControl);
			stopTask(dumpOverride);
		}
		wait1Msec(20);
	}
}
task dumping()
{
	stopTask(hold);
	stopTask(LiftControl);
	startTask(dumpOverride);
	while (SensorValue[pot] < clawHeight)
	{
		lift(127);
		wait1Msec(20);
	}
	claw(1);
	while(SensorValue[pot] < topHeight) wait1Msec(20);
	lift(0);
	wait1Msec(500);
	while (SensorValue[pot] > (holdHeight + 200))
	{
		lift(-80);
		wait1Msec(20);
	}
	while (SensorValue[pot] > (bottomHeight + 50))  // 70
	{
		lift(-30);
		wait1Msec(20);
	}
	lift(0);
	stopTask(dumpOverride);
	dumpMode = 1;
	startTask(LiftControl);
	stopTask(dumping);
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

void lift (int target, int pwr)
{
	int dir = sgn(target - SensorValue[pot]);
	while (abs(SensorValue[pot] - target) > 20)
	{
		lift(pwr * dir);
	}
	lift(0);
}
