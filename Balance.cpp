#include "Balance.h"
#include "Utility.h"

Balance balance;

extern float m_filter_hz;

namespace
{
	volatile uint32_t duration = 0; // accumulates pulse width
	volatile uint32_t pulsecount = 0;
	volatile uint32_t previousMicros = 0;

	void rpm_interrupt()
	{
		balance.interrupt_handler();
	}
}

void Balance::begin()
{
	pinMode(PROP_ESC_PIN, OUTPUT);
	pinMode(SERVO1_PIN, OUTPUT);
	pinMode(SERVO2_PIN, OUTPUT);

	m_servo1.begin(SERVO1_PIN, 1000, 2000);
	m_servo2.begin(SERVO2_PIN, 1000, 2000);

	m_motor.begin(PROP_ESC_PIN, 1000, 2000);
	setPropSpeed(1000);

	pt1FilterInit(&rpm_filter, rpm_cut_off_hz, 1.0f / m_filter_hz);

	pinMode(PROP_RPM_PIN, INPUT);
	attachInterrupt(PROP_RPM_PIN, rpm_interrupt, CHANGE);
}

void Balance::setPropSpeed(float speed)
{
	m_prop_speed = speed;
	m_motor.write(m_prop_speed);
}

void Balance::setServoPositions(float pos1, float pos2)
{
	m_servo1_pos = mapf(pos2, 1000, 2000, 1550, 1170);
	m_servo2_pos = mapf(pos1, 1000, 2000, 1500, 1890);

	m_servo1_pos = constrain(m_servo1_pos, 1170, 1550);
	m_servo2_pos = constrain(m_servo2_pos, 1500, 1890);

	m_servo1.write(m_servo1_pos);
	m_servo2.write(m_servo2_pos);
}

void Balance::printRPM()
{
	const uint32_t _duration = duration;
	const uint32_t _pulsecount = pulsecount;
	duration = 0; // clear counters
	pulsecount = 0;
	
	float freq = 0.0f;
	
	if (_duration != 0)
	{
		freq = 1e6 / float(_duration) * _pulsecount;
	}
	
	Serial.print(freq);
	Serial.print("\t");

	m_sensed_rpm = pt1FilterApply(&rpm_filter, freq);
	
	Serial.println(m_sensed_rpm);
}

void Balance::interrupt_handler()
{
	const uint32_t currentMicros = micros();
	duration += currentMicros - previousMicros;
	previousMicros = currentMicros;
	pulsecount++;
}