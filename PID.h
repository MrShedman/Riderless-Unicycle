
#pragma once

#include "Arduino.h"

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
	float kp[PID_ITEM_COUNT];
	float ki[PID_ITEM_COUNT];
	float kd[PID_ITEM_COUNT];

	float max_I[PID_ITEM_COUNT];
	float max_Out[PID_ITEM_COUNT];

	uint8_t tpa[PID_ITEM_COUNT];
	uint8_t tpa_breakpoint[PID_ITEM_COUNT];
}pidProfile_t;

class PID
{
public:

	void setup(float kp, float ki, float kd, float max_i, float max_output, uint8_t tpa, uint8_t tpa_breakpoint)
	{
		p_gain = kp;
		i_gain = ki;
		d_gain = kd;
		this->max_i = max_i;
		this->max_output = max_output;
		this->tpa = constrain(tpa, 0, 100);
		this->tpa_breakpoint = constrain(tpa_breakpoint, 0, 100);

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

	uint8_t tpa;
	uint8_t tpa_breakpoint;

	float p_gain;                //Gain setting for the pitch P-controller. //4.0
	float i_gain;                //Gain setting for the pitch I-controller. //0.02
	float d_gain;                //Gain setting for the pitch D-controller.

	float max_output;          //Maximum output of the PID-controller (+/-)
	float max_i;               //Maximum value of I term of the PID-controller (+/-)  

	float i_mem;
	float output;
	float last_d_error;
};

extern pidProfile_t pid_profile;

extern PID axisPID[PID_ITEM_COUNT];