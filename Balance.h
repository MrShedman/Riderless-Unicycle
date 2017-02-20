#pragma once

#include "Arduino.h"

#include "Pins.h"
#include "Servo.h"

class Balance
{
public:

	void begin()
	{
		pinMode(PROP_RPM_PIN, INPUT);
		pinMode(PROP_ESC_PIN, OUTPUT);
		pinMode(SERVO1_PIN, OUTPUT);
		pinMode(SERVO2_PIN, OUTPUT);

		m_servo1.begin(SERVO1_PIN, 1000, 2000);
		m_servo2.begin(SERVO2_PIN, 1000, 2000);

		m_motor.begin(PROP_ESC_PIN, 1000, 2000);
		m_motor.write(1000);
	}

	void setPropSpeed(uint32_t speed)
	{
		m_prop_speed = speed;
		m_motor.write(speed);
	}

	void setServoPositions(uint32_t pos1, uint32_t pos2)
	{
		m_servo1_pos = map(pos2, 1000, 2000, 1580, 1170);
		m_servo2_pos = map(pos1, 1000, 2000, 1480, 1890);

		m_servo1.write(m_servo1_pos);
		m_servo2.write(m_servo2_pos);
	}

private:
	
	Servo m_servo1;
	Servo m_servo2;
	Servo m_motor;

	uint32_t m_servo1_pos;
	uint32_t m_servo2_pos;
	uint32_t m_prop_speed;
};

extern Balance balance;