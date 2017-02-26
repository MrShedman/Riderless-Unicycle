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

	pidProfile_t profile;
}

void Balance::begin()
{
	profile.kp = 4.0f;
	profile.ki = 0.0f;
	profile.kd = 2.0f;
	profile.max_I = 200.0f;
	profile.max_Out = 200.0f;
	profile.tpa = 100;
	profile.tpa_breakpoint = 2000;
	profile.dterm_lpf_hz = 5.0f;

	rpmPID.setup(&profile);

	pinMode(PROP_ESC_PIN, OUTPUT);
	pinMode(SERVO1_PIN, OUTPUT);
	pinMode(SERVO2_PIN, OUTPUT);

	m_servo1.begin(SERVO1_PIN, m_s1_min, m_s1_max);
	m_servo2.begin(SERVO2_PIN, m_s2_min, m_s2_max);

	m_motor.begin(PROP_ESC_PIN, 1000, 2000);
	setPropSpeed(1000);

	pt1FilterInit(&rpm_filter, rpm_cut_off_hz, 1.0f / m_filter_hz);

	pinMode(PROP_RPM_PIN, INPUT);
	attachInterrupt(PROP_RPM_PIN, rpm_interrupt, CHANGE);
}

void Balance::setPropSpeed(float speed)
{
	m_motor.write(speed);

	Serial.print(m_sensed_rpm);
	Serial.print("\t");
	//Serial.println(speed);
	//Serial.print("\t");

	float input = mapf(m_sensed_rpm, 1000.0f, 3000.0f, 1140.0f, 1500.0f);

	rpmPID.update(input, speed, speed, 2000);

	float s = rpmPID.getOutput();
	s = constrain(s, 1000, 1400);
	Serial.println(s);

	//m_motor.write(s);
}

void Balance::setServoPositions(float pos1, float pos2)
{
	m_servo1.write(pos1);
	m_servo2.write(pos2);
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
	
	//Serial.print(freq);
	//Serial.print("\t");

	m_sensed_rpm = pt1FilterApply(&rpm_filter, freq);
	
	//Serial.println(m_sensed_rpm);
}

void Balance::interrupt_handler()
{
	const uint32_t currentMicros = micros();
	duration += currentMicros - previousMicros;
	previousMicros = currentMicros;
	pulsecount++;
}