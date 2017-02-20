
#pragma once

#include "Arduino.h"
 
#include "datatypes.h"
#include "local_datatypes.h"

class VESC
{
public:

	void begin()
	{
		Serial5.begin(256800);
	}

	void update()
	{
		if (millis() - t > 100)
		{
			t = millis();
		}
		else
		{
			return;
		}

		if (getValue(motor_values))
		{
			//Serial.print("Loop: "); Serial.println(count++);
			//print(motor_values);
		}
		else
		{
			Serial.println("Failed to get data!");
		}
	}

	void print(const struct bldcMeasure& values);

	void print(const mc_values& values);

	void print(uint8_t* data, int len);

	bool getValue(struct bldcMeasure& values);

	bool getValue(mc_values& values);

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

	mc_values motor_values;

private:

	uint32_t t;

	int packSendPayload(uint8_t* payload, int lenPay);
	int receiveMessage(uint8_t* payloadReceived);

	bool unpackPayload(uint8_t* message, int lenMes, uint8_t* payload, int lenPa);
	bool processReadPacket(uint8_t* message, bldcMeasure& values, int len);
	bool processReadPacket(uint8_t* message, mc_values& values, int len);

	uint32_t count;
	struct bldcMeasure measuredValues;

};

extern VESC vesc;