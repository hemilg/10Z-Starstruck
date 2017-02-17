float driveMult = 1.0;
int dumpMode = 1;

int bottomHeight = 900;
int holdHeight = 1000;
int topHeight = 1500;

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
	return linearSpeed(pwr);
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
		if (vexRT[Btn5U])
		{
			lift(127);
			prev = SensorValue[pot];
			dumpMode = 1;
		}
		else if (vexrt[Btn5D])
		{
			lift(-127);
			prev = SensorValue[pot];
			dumpMode = 1;
		}
		else
		{
			if (prev > holdHeight)	lift(15);
			else lift(0);
		}
		wait1Msec(20);
	}
}

task IntakeControl()
{
	while (1)
	{
	if (vexRT[Btn6U])
			{
				while (vexRT[Btn6U]) wait1Msec(20);
				SensorValue[Claw1] = 1;
				SensorValue[Claw2] = 1;
			}
			else if (vexRT[Btn6D])
			{
				while (vexRT[Btn6D]) wait1Msec(20);
				SensorValue[Claw1] = 0;
				SensorValue[Claw2] = 0;
			}

			wait1Msec(20);
		}
}

task hold()
{
	dumpMode = 2;
	stopTask(LiftControl);
	while (!(vexRT[Btn5D] || vexRT[Btn5U] || vexRT[Btn8U])
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
	while (SensorValue[pot] < topHeight)
	{
		lift(127);
		wait1Msec(20);
	}
	claw();
	while (SensorValue[pot] > bottomHeight)
	{
		lift(-127);
		wait1Msec(20);
	}
	dumpMode = 1;
	startTask(LiftControl);
}
