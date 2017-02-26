#include "PID.h"
#include "Utility.h"

pidProfile_t pid_profile;

PID axisPID[PID_ITEM_COUNT];

void PID::update(float input, float setpoint, float throttle, float max_throttle)
{
	const float error_temp = input - setpoint;

	float kp = p_gain;
	float ki = i_gain;
	float kd = d_gain;

	if (throttle > tpa_breakpoint)
	{
		float attenuation = mapf(throttle, (float)tpa_breakpoint, max_throttle, 100.0f, (float)tpa) / 100.0f;
		kp *= attenuation;
		ki *= attenuation;
		kd *= attenuation;
	}

	i_mem += ki * error_temp;
	i_mem = constrain(i_mem, -max_i, max_i);

	output = kp * error_temp + i_mem + kd * (error_temp - last_d_error);
	output = constrain(output, -max_output, max_output);

	last_d_error = error_temp;
}