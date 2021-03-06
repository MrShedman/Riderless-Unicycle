#pragma once

#include "PID.h"

void resetPidProfile(pidProfile_t profile[PID_ITEM_COUNT])
{
	// ROLL

	profile[PIDROLL].kp = 50.0f;//12.0f;
	profile[PIDROLL].ki = 0.01f;//0.02f;//0.25f;
	profile[PIDROLL].kd = 30.0f;//50.0f;//3.0f;

	profile[PIDROLL].max_I = 400.0f;
	profile[PIDROLL].max_Out = 400.0f;

	profile[PIDROLL].tpa = 100;
	profile[PIDROLL].tpa_breakpoint = 1500;

	profile[PIDROLL].dterm_lpf_hz = 10;
	profile[PIDROLL].lpf_dT = 1.0f / 1000.0f;

	// PITCH

	profile[PIDPITCH].kp = 8.0f;
	profile[PIDPITCH].ki = 0.0f;
	profile[PIDPITCH].kd = 0.0f;

	profile[PIDPITCH].max_I = 400.0f;
	profile[PIDPITCH].max_Out = 200.0f;

	profile[PIDPITCH].tpa = 100;
	profile[PIDPITCH].tpa_breakpoint = 2000;

	profile[PIDPITCH].dterm_lpf_hz = 5;
	profile[PIDPITCH].lpf_dT = 1.0f / 1000.0f;

	// YAW

	profile[PIDYAW].kp = 4.5f;
	profile[PIDYAW].ki = 0.0f;
	profile[PIDYAW].kd = 0.0f;

	profile[PIDYAW].max_I = 400.0f;
	profile[PIDYAW].max_Out = 200.0f;

	profile[PIDYAW].tpa = 100;
	profile[PIDYAW].tpa_breakpoint = 2000;

	profile[PIDYAW].dterm_lpf_hz = 5;
	profile[PIDYAW].lpf_dT = 1.0f / 1000.0f;
}