#include <Movement.h>

int target;
float kp, ki, kd;
int minPwr;

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
		leftDrive(linSpeed(pow);
		rightDrive(-linSpeed(pow));
	}
}
