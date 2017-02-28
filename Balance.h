#pragma once

#include "Arduino.h"

#include "Pins.h"
#include "Servo.h"
#include "Filter.h"
#include "PID.h"

class Balance
{
public:

	void begin();

	void setPropSpeed(float speed);

	void setServoPositions(float pos1, float pos2);

	float updateRPM();

	void interrupt_handler();

private:
	
	pidProfile_t profile;
	PID rpmPID;

	float rpm_cut_off_hz = 5.0f;
	pt1Filter_t rpm_filter;
	float m_sensed_rpm;

	Servo m_servo1;
	Servo m_servo2;
	Servo m_motor;

	const float m_s1_max = 1170.0f;
	const float m_s1_min = 1550.0f;

	const float m_s2_max = 1890.0f;
	const float m_s2_min = 1500.0f;
};

extern Balance balance;