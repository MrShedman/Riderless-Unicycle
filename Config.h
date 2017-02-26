#pragma once

#include "PID.h"

void resetPidProfile(pidProfile_t profile[PID_ITEM_COUNT])
{
	// ROLL

	profile[PIDROLL].kp = 80.0f;//12.0f;
	profile[PIDROLL].ki = 0.02f;//0.25f;
	profile[PIDROLL].kd = 100.0f;//3.0f;

	profile[PIDROLL].max_I = 400.0f;
	profile[PIDROLL].max_Out = 400.0f;

	profile[PIDROLL].tpa = 80;
	profile[PIDROLL].tpa_breakpoint = 1500;

	profile[PIDROLL].dterm_lpf_hz = 5;

	// PITCH

	profile[PIDPITCH].kp = 8.0f;
	profile[PIDPITCH].ki = 0.0f;
	profile[PIDPITCH].kd = 1.0f;

	profile[PIDPITCH].max_I = 400.0f;
	profile[PIDPITCH].max_Out = 400.0f;

	profile[PIDPITCH].tpa = 100;
	profile[PIDPITCH].tpa_breakpoint = 2000;

	profile[PIDPITCH].dterm_lpf_hz = 5;

	// YAW

	profile[PIDYAW].kp = 4.0f;
	profile[PIDYAW].ki = 0.0f;
	profile[PIDYAW].kd = 0.0f;

	profile[PIDYAW].max_I = 400.0f;
	profile[PIDYAW].max_Out = 400.0f;

	profile[PIDYAW].tpa = 100;
	profile[PIDYAW].tpa_breakpoint = 2000;

	profile[PIDYAW].dterm_lpf_hz = 5;
}