#include "Balance.h"
#include "Utility.h"

Balance balance;

extern float main_loop_hz;

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
	profile.kp = 5.0f;
	profile.ki = 0.2f;
	profile.kd = 0.0f;
	profile.max_I = 3000.0f;
	profile.max_Out = 6000.0f;
	profile.tpa = 100;
	profile.tpa_breakpoint = 2000;
	profile.dterm_lpf_hz = 5.0f;
	profile.lpf_dT = 1.0f / main_loop_hz;

	rpmPID.setup(&profile);

	pinMode(PROP_ESC_PIN, OUTPUT);
	pinMode(SERVO1_PIN, OUTPUT);
	pinMode(SERVO2_PIN, OUTPUT);

	m_servo1.begin(SERVO1_PIN, m_s1_min, m_s1_max);
	m_servo2.begin(SERVO2_PIN, m_s2_min, m_s2_max);

	m_motor.begin(PROP_ESC_PIN, 1000, 2000);
	setPropSpeed(1000);

	pt1FilterInit(&rpm_filter, rpm_cut_off_hz, profile.lpf_dT);

	pinMode(PROP_RPM_PIN, INPUT);
	attachInterrupt(PROP_RPM_PIN, rpm_interrupt, CHANGE);
}


void Balance::setPropSpeed(float speed)
{
	// found empirically
	// relates throttle (1-2ms pulse) with rpm
	const float m = 9.3192f;
	const float c = -9585.5f;

	float setpoint = m * speed + c;

	if (setpoint < 0.0f)
	{
		setpoint = 0.0f;
	}

	rpmPID.update(setpoint, updateRPM(), speed, 2000);

	float s = (rpmPID.getOutput() - c) / m;

	//Serial.println(s);

	s = constrain(s, 1000, 1400);

	//Serial.println(s);

	// dont use PID as rpm measurement is noisy at low rpms
	if (speed > 1150)
	{
		m_motor.write(s);
	}
	else
	{
		m_motor.write(speed);
		rpmPID.reset();
	}
}

void Balance::setServoPositions(float pos1, float pos2)
{
	m_servo1.write(pos1);
	m_servo2.write(pos2);
}

float Balance::updateRPM()
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

	m_sensed_rpm = pt1FilterApply(&rpm_filter, freq);
	
	Serial.print(freq);
	Serial.print("\t");
	Serial.print(m_sensed_rpm);
	Serial.print("\n");

	return m_sensed_rpm;
}

void Balance::interrupt_handler()
{
	const uint32_t currentMicros = micros();
	duration += currentMicros - previousMicros;
	previousMicros = currentMicros;
	pulsecount++;
}