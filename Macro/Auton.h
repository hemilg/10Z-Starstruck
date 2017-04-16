int target;
float kp, ki, kd;
int minPwr;
bool taskOver = false;

task pidL()
{
	float p, i, d;
	float prevError, error, integral;

	float kp2;
	int pwr;
	taskOver = false;

	while (abs(SensorValue[LeftEnc]) < abs(target))
	{
		error = target - SensorValue[LeftEnc];
		integral += error;
		p = (abs(error) > 400 ? (127 * sgn(error)) : (kp * error));
		i = ki * integral;
		d = kd * (error - prevError);
		pwr = p + i + d;
		pwr = (pwr > 0) ? min(pwr, 127) : max(pwr, -127);
		pwr = linSpeed(pwr);
		leftDrive(abs(pwr) > minPwr ? pwr : sgn(pwr) * minPwr);
		rightDrive(abs(pwr) > minPwr ? pwr : sgn(pwr) * minPwr);
	}
	leftDrive(0);
	rightDrive(0);
	taskOver = true;
	stopTask(pidL);
}

task pidR ()
{
	float p, i, d;
	float prevError, error, integral;

	float kp2;
	int pwr;

	while (abs(SensorValue[RightEnc]) < abs(target))
	{
		error = target - SensorValue[RightEnc];
		integral += error;
		p = (abs(error) > 400 ? (127 * sgn(error)) : (kp * error));
		i = ki * integral;
		d = kd * (error - prevError);
		pwr = p + i + d;
		pwr = (pwr > 0) ? min(pwr, 127) : max(pwr, -127);
		pwr = linSpeed(pwr);
		rightDrive(abs(pwr) > minPwr ? pwr : sgn(pwr) * minPwr);
	}
	rightDrive(0);
	stopTask(pidR);
}

void drive (int target, int pwr)
{
	stopTask(pidL);
	resetEncoders();
	while (abs(SensorValue[LeftEnc]) < abs(target))
	{
		rightDrive(pwr * sgn(target));
		leftDrive(pwr * sgn(target));
		wait1Msec(20);
	}
	rightDrive(0);
	leftDrive(0);
}
void drive (int time, int pwr, bool timeBased)
{
	stopTask(pidL);
	rightDrive(pwr);
	leftDrive(pwr);
	wait1Msec(time);
	rightDrive(0);
	leftDrive(0);
}
void drive (int targetI, int minPwrI, float kpI, float kiI, float kdI)
{
	stopTask(pidL);
	target = targetI;
	minPwr = minPwrI;
	kp = kpI;
	ki = kiI;
	kd = kdI;
	resetEncoders();
	startTask(pidL);
}

void drive (int targetI, int minPwrI, float kpI, float kiI, float kdI, bool method)
{
	stopTask(pidL);
	target = targetI;
	minPwr = minPwrI;
	kp = kpI;
	ki = kiI;
	kd = kdI;
	resetEncoders();
	taskOver = false;
	startTask(pidL);
	while (!taskOver) wait1Msec(20);
}

void drive (int targetI, int minPwrI, float kpI, float kiI, float kdI, int timeMax)
{
	target = targetI;
	minPwr = minPwrI;
	kp = kpI;
	ki = kiI;
	kd = kdI;
	resetEncoders();
	startTask(pidL);
	startTask(pidR);
	clearTimer(T1);
	while (time1[T1] < timeMax) wait1Msec(20);
	stopTask(pidL);
	stopTask(pidR);
}
void turn (int deg, int pwr, int mpwr, float kp)
{
	deg *= -1;
	stopTask(pidL);
	SensorValue[gyro] = 0;
	int minPow = mpwr; // 35
	while (abs(SensorValue[gyro]) < abs(deg))
	{
		int error = deg - SensorValue[gyro];
		int pow = (abs(error) > 900) ? 127 * sgn(error) : kp * error;
		pow = (pow > 0) ? min(pow, 127) : max(pow, -127);
		pow = (abs(pow) > minPow ? pow : sgn(pow) * minPow);
		leftDrive(-linSpeed(pow));
		rightDrive(linSpeed(pow));
	}
}

void turn (int deg, int pwr, int mpwr)
{
	deg *= -1;
	stopTask(pidL);
	SensorValue[gyro] = 0;
	float kp = 0.15;
	int minPow = mpwr; // 35
	while (abs(SensorValue[gyro]) < abs(deg))
	{
		int error = deg - SensorValue[gyro];
		int pow = (abs(error) > 900) ? 127 * sgn(error) : kp * error;
		pow = (pow > 0) ? min(pow, 127) : max(pow, -127);
		pow = (abs(pow) > minPow ? pow : sgn(pow) * minPow);
		leftDrive(-linSpeed(pow));
		rightDrive(linSpeed(pow));
	}
}

void flipout()
{
	stopTask(DriveControl);
	stopTask(IntakeControl);
	stopTask(clawSync);
	drive(400, -90, true);
	motor[ClawL] = motor[ClawR] = -127;
	wait1Msec(400);
	motor[ClawL] = motor[ClawR] = 0;
}
