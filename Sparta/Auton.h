int target;
int m1Val;

int min (int a, int b)
{
	return (a < b) ? a : b;
}

int max (int a, int b)
{
	return (a > b) ? a : b;
}

task pidm1()
{
	float kp, ki, kd;
	float p, i, d;
	float prevError, error, integral;

	float kp2;

	while (abs(SensorValue[s1]) < abs(target))
	{
		error = target - SensorValue[s1];
		integral += error;
		p = kp * error;
		i = ki * integral;
		d = kd * (error - prevError);
		m1Val = p + i + d;
		m1Val = (m1Val > 0) ? min(m1Val, 127) : max(m1Val, -127);

		m1Val = m1Val - ((SensorValue[s1] > SensorValue[s2]) ? (SensorValue[s1] - SensorValue[s2]) * kp2 : 0);
	}
}
