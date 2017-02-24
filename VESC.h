
#pragma once

#include "Arduino.h" 
#include "datatypes.h"

class VESC
{
public:

	void begin()
	{
		Serial1.setRX(27);
		Serial1.setTX(26);
		Serial1.begin(256800);

		m_read_delay = 5000; //us
		m_update_rate = 100; //ms
	}

	void update()
	{
		if (m_update_timer > m_update_rate)
		{
			requestValues();
			m_update_timer = 0;
		}
	}

	void print(const mc_values& values);

	void print(uint8_t* data, int len);

	void requestValues();

	void setCurrent(float current);

	void setCurrentBrake(float brakeCurrent);
	
	float getMaxMOSFETTemp() const
	{
		float maxt = 0.0f;

		maxt = max(maxt, motor_values.temp_mos1);
		maxt = max(maxt, motor_values.temp_mos2);
		maxt = max(maxt, motor_values.temp_mos3);
		maxt = max(maxt, motor_values.temp_mos4);
		maxt = max(maxt, motor_values.temp_mos5);
		maxt = max(maxt, motor_values.temp_mos6);

		return maxt;
	}

	void getValues()
	{
		uint8_t payload[256];

		int lenPayload = receiveMessage(payload);
		if (lenPayload > 55)
		{
			processReadPacket(payload, motor_values, lenPayload);
		}

		m_read_timer.end();
	}

	mc_values motor_values;

private:

	uint32_t t;

	int packSendPayload(uint8_t* payload, int lenPay);
	int receiveMessage(uint8_t* payloadReceived);

	bool unpackPayload(uint8_t* message, int lenMes, uint8_t* payload, int lenPa);

	bool processReadPacket(uint8_t* message, mc_values& values, int len);

	uint32_t count;

	uint32_t m_update_rate;
	elapsedMillis m_update_timer;

	uint32_t m_read_delay;
	IntervalTimer m_read_timer;
};

extern VESC vesc;