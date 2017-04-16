float driveMult = 1.0;
int dumpMode = 1;

int bottomHeight = 1900; // 850
int holdHeight = 1100; // 950
int clawHeight = 800;
int topHeight = 300;


int clawTarget = 0;
int clawOpen = -800; // 1400
int clawClosed = 250; // 2450
int lclaw = 0;
int lclawstart = 0;
int rclaw = 0;
int rclawstart = 0;

int liftHolding = -12;


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

int min (int a, int b)
{
	return (a < b) ? a : b;
}

int max (int a, int b)
{
	return (a > b) ? a : b;
}

int linSpeed (int pwr)
{
	return linearSpeed[abs(pwr)] * sgn(pwr);
}

const int SLEW_LENGTH = 10;

int Slew[SLEW_LENGTH];
task dumping();
task dumpOverride();

void potValues(int zero)
{
	int offset = zero - (bottomHeight); // +100
	bottomHeight += offset;
	holdHeight += offset;
	clawHeight += offset;
	topHeight += offset;
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
	SensorValue[gyro] = 0;
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
	motor[TRL] = motor[TLL] = motor[BRL] = motor[BLL] = pwr;
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
		if (vexRT[Btn5U] /*&& SensorValue[pot] > (topHeight + 300)*/)
		{
			lift((int)(127 * driveMult));
			prev = SensorValue[pot];
			dumpMode = 1;
		}
		else if (vexRT[Btn5D] /*&& SensorValue[pot] < holdHeight*/)
		{
			lift((int)(-127 * driveMult));
			prev = SensorValue[pot];
			dumpMode = 1;
		}
		else
		{
			lift(liftHolding);/*lift(SensorValue[pot] > (bottomHeight + 10) ? -20 : 0); // -8*/
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
				clawTarget = (clawTarget == clawOpen ? clawClosed : clawOpen);
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
		while (SensorValue[pot] > holdHeight)
		{
			lift(30);
			wait1Msec(20);
		}
		lift(-12);
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
	lift(127);
	while (SensorValue[pot] > clawHeight)
	{
		wait1Msec(20);
	}
	clawTarget = clawOpen;
	while(SensorValue[pot] > topHeight) wait1Msec(20);
	lift(0);
	wait1Msec(500);
	while (SensorValue[pot] < (holdHeight + 200))
	{
		lift(-127);
		wait1Msec(20);
	}
	while (SensorValue[pot] < (bottomHeight - 30))  // 70
	{
		lift(-60);
		wait1Msec(20);
	}
	lift(0);
	stopTask(dumpOverride);
	dumpMode = 1;
	startTask(LiftControl);
	stopTask(dumping);
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

task PotValues()
{
	lclaw = 0;
	lclawstart = SensorValue[leftclaw];
	rclaw = 0;
	rclawstart = SensorValue[rightclaw];
	while (1)
	{
		wait1Msec(20);
		lclaw = (lclawstart - SensorValue[leftclaw]);
		rclaw = (SensorValue[rightclaw] - rclawstart);
	}
}

task clawSync()
{
	while (1)
	{
		float lOffset = 1.1;
		if (abs(lclaw - (clawTarget * lOffset)) > 70)
		{
			int lpwr = (abs(clawTarget * lOffset - lclaw) > 700) ? sgn(clawTarget * lOffset - lclaw) * 127 : (clawTarget * lOffset - lclaw) / 8;
			lpwr = (abs(lpwr) < 10 ? sgn(lpwr) * 10 : lpwr);
			motor[ClawL] = lpwr;
		}
		else motor[ClawL] = 0;
		if (abs(rclaw - clawTarget) > 70)
		{
			int rpwr = (abs(clawTarget - rclaw) > 700) ? sgn(clawTarget - rclaw) * 127 : (clawTarget - rclaw) / 8;
			rpwr = (abs(rpwr) < 10 ? sgn(rpwr) * 10 : rpwr);
			motor[ClawR] = rpwr;
		}
		else motor[ClawR] = 0;
		wait1Msec(20);
	}
}
