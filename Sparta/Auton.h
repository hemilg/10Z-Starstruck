#include <Movement.h>

int target;
float kp, ki, kd;
int minPwr;

int min (int a, int b)
{
	return (a < b) ? a : b;
}

int max (int a, int b)
{
	return (a > b) ? a : b;
}

void gyroCalibrate()
{
    SensorType[gyro] = sensorNone;
    wait1Msec(1000);
    SensorType[gyro] = sensorGyro;
    wait1Msec(2000);
    SensorValue[gyro] = 0;
}

int linSpeed (int pwr)
{
	return linearSpeed[abs(pwr)] * sgn(pwr);
}
task pidL()
{
	float p, i, d;
	float prevError, error, integral;

	float kp2;
	int pwr;

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
	SensorValue[RightEnc] = 0;
	while (abs(SensorValue[RightEnc]) < target)
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
	rightDrive(pwr);
	leftDrive(pwr);
	wait1Msec(time);
	rightDrive(0);
	leftDrive(0);
}
void drive (int targetI, int minPwrI, float kpI, float kiI, float kdI)
{
	target = targetI;
	minPwr = minPwrI;
	kp = kpI;
	ki = kiI;
	kd = kdI;
	SensorValue[RightEnc] = 0;
	SensorValue[LeftEnc] = 0;
	startTask(pidL);
	//startTask(pidR);
}
void drive (int targetI, int minPwrI, float kpI, float kiI, float kdI, int timeMax)
{
	target = targetI;
	minPwr = minPwrI;
	kp = kpI;
	ki = kiI;
	kd = kdI;
	SensorValue[RightEnc] = 0;
	SensorValue[LeftEnc] = 0;
	startTask(pidL);
	startTask(pidR);
	clearTimer(T1);
	while (time1[T1] < timeMax) wait1Msec(20);
	stopTask(pidL);
	stopTask(pidR);
}
void turn (int deg, int pwr)
{
	SensorValue[gyro] = 0;
	float kp = 0.15;
	int minPow = 35;
	while (abs(SensorValue[gyro]) < deg)
	{
		int error = deg - SensorValue[gyro];
		int pow = (abs(error) > 900) ? 127 * sgn(error) : kp * error;
		pow = (pow > 0) ? min(pow, 127) : max(pow, -127);
		pow = (abs(pow) > minPow ? pow : sgn(pow) * minPow);
		leftDrive(linSpeed(pow));
		rightDrive(-linSpeed(pow));
	}
}
