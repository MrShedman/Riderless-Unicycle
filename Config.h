#pragma once

#include "PID.h"

void resetPidProfile(pidProfile_t* profile)
{
	// ROLL

	profile->kp[PIDROLL] = 100.0f;//12.0f;
	profile->ki[PIDROLL] = 0.02f;//0.25f;
	profile->kd[PIDROLL] = 70.0f;//3.0f;

	profile->max_I[PIDROLL] = 400.0f;
	profile->max_Out[PIDROLL] = 400.0f;

	profile->tpa[PIDROLL] = 100.0f;
	profile->tpa_breakpoint[PIDROLL] = 100.0f;

	// PITCH

	profile->kp[PIDPITCH] = 3.0f;
	profile->ki[PIDPITCH] = 0.0f;
	profile->kd[PIDPITCH] = 0.0f;

	profile->max_I[PIDPITCH] = 400.0f;
	profile->max_Out[PIDPITCH] = 400.0f;

	profile->tpa[PIDPITCH] = 100.0f;
	profile->tpa_breakpoint[PIDPITCH] = 100.0f;

	// YAW

	profile->kp[PIDYAW] = 4.0f;
	profile->ki[PIDYAW] = 0.0f;
	profile->kd[PIDYAW] = 0.0f;

	profile->max_I[PIDYAW] = 400.0f;
	profile->max_Out[PIDYAW] = 400.0f;

	profile->tpa[PIDYAW] = 100.0f;
	profile->tpa_breakpoint[PIDYAW] = 100.0f;
}