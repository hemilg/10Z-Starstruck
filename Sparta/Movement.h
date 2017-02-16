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

task LiftControl()
{
	while (1)
	{
		if (vexRT[Btn5U]) lift(127);
		else if (vexrt[Btn5D]) lift(-127);
		else lift(0);

		wait1Msec(20);
	}
}

task IntakeControl()
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
