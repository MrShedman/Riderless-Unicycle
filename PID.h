
#pragma once

#include "Arduino.h"

class PID
{
public:

	void setup(float kp, float ki, float kd, int16_t max_i, int16_t max_output, int16_t tpa, int16_t tpa_breakpoint)
	{
		p_gain = kp;
		i_gain = ki;
		d_gain = kd;
		this->max_i = max_i;
		this->max_output = max_output;
		this->tpa = tpa;
		this->tpa_breakpoint = tpa_breakpoint;

		reset();
	}

	void update(float input, float setpoint, int16_t throttle, int16_t max_throttle)
	{
		const float error_temp = input - setpoint;

		float kp = p_gain;
		float ki = i_gain;
		float kd = d_gain;

		if (throttle > tpa_breakpoint)
		{
			float attenuation = float(map(throttle, tpa_breakpoint, max_throttle, 100, tpa)) / 100.0;
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

	void reset()
	{
		i_mem = 0;
		last_d_error = 0;
	}

	float getOutput() const
	{
		return output;
	}

private:

	int16_t tpa;
	int16_t tpa_breakpoint;

	float p_gain;                //Gain setting for the pitch P-controller. //4.0
	float i_gain;                //Gain setting for the pitch I-controller. //0.02
	float d_gain;                //Gain setting for the pitch D-controller.

	int16_t max_output;          //Maximum output of the PID-controller (+/-)
	int16_t max_i;               //Maximum value of I term of the PID-controller (+/-)  

	float i_mem;
	float output;
	float last_d_error;
};