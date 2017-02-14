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
	return pwr;
}

task DriveControl()
{
	while (1)
	{
		leftDrive((abs(vexRT[Ch3]) > 5) ? cubicMap(vexRT[Ch3]) : 0);
		rightDrive((abs(vexRT[Ch2]) > 5) ? cubicMap(vexRT[Ch2]) : 0);
		wait1Msec(20);
	}
}
