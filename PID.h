
#pragma once

#include "Arduino.h"
#include "Filter.h"

typedef enum
{
	PIDROLL,
	PIDPITCH,
	PIDYAW,
	PID_ITEM_COUNT
}
pidIndex_e;

typedef struct pidProfile_s
{
	float kp;
	float ki;
	float kd;

	float max_I;
	float max_Out;

	uint16_t tpa;
	uint16_t tpa_breakpoint;

	float dterm_lpf_hz;
}pidProfile_t;

class PID
{
public:

	void setup(const pidProfile_t* profile)//float kp, float ki, float kd, float max_i, float max_output, uint8_t tpa, uint8_t tpa_breakpoint)
	{
		p_gain = profile->kp;
		i_gain = profile->ki;
		d_gain = profile->kd;
		this->max_i = profile->max_I;
		this->max_output = profile->max_Out;
		this->tpa = constrain(profile->tpa, 0, 100);
		this->tpa_breakpoint = constrain(profile->tpa_breakpoint, 1000, 2000);

		if (profile->dterm_lpf_hz > 0.0f)
		{
			pt1FilterInit(&dterm_filter, profile->dterm_lpf_hz, 1.0f / 1000.f);
		}

		reset();
	}

	void update(float input, float setpoint, float throttle, float max_throttle);

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

	uint16_t tpa;
	uint16_t tpa_breakpoint;

	float p_gain;                //Gain setting for the pitch P-controller. //4.0
	float i_gain;                //Gain setting for the pitch I-controller. //0.02
	float d_gain;                //Gain setting for the pitch D-controller.

	float max_output;          //Maximum output of the PID-controller (+/-)
	float max_i;               //Maximum value of I term of the PID-controller (+/-)  

	float i_mem;
	float output;
	float last_d_error;

	pt1Filter_t dterm_filter;
};

extern pidProfile_t pid_profile[PID_ITEM_COUNT];

extern PID axisPID[PID_ITEM_COUNT];