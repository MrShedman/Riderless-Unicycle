#include "Utility.h"

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int32_t applyDeadband(int32_t value, int32_t deadband)
{
	if (abs(value) < deadband)
	{
		value = 0;
	}
	else if (value > 0)
	{
		value -= deadband;
	}
	else if (value < 0)
	{
		value += deadband;
	}

	return value;
}