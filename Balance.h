#pragma once

#include "Arduino.h"

#include "Pins.h"
#include "Servo.h"
#include "Filter.h"

class Balance
{
public:

	void begin();

	void setPropSpeed(float speed);

	void setServoPositions(float pos1, float pos2);

	void printRPM();

	void interrupt_handler();

private:
	
	float rpm_cut_off_hz = 5.0f;
	pt1Filter_t rpm_filter;
	uint32_t m_sensed_rpm;

	Servo m_servo1;
	Servo m_servo2;
	Servo m_motor;

	uint32_t m_servo1_pos;
	uint32_t m_servo2_pos;
	uint32_t m_prop_speed;
};

extern Balance balance;