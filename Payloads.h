#pragma once

#include "Radio.h"

enum armedState : uint8_t
{
	DISARMED,
	PENDING,
	ARMED
};

typedef struct armedStateTableEntry_s
{
	armedState mode;
	const char *name;
}
armedStateTableEntry_t;

static const armedStateTableEntry_t armedStateTable[] = {
	{ DISARMED,	"DISARMED" },
	{ PENDING,	"PENDING" },
	{ ARMED,	"ARMED" }
};

struct QuadPayload : public Radio::Payload
{
	void* data() override
	{ 
		return &throttle;
	}

	uint32_t size() override
	{
		return 8;
	}

	void reset() override
	{
		throttle = 1000;
		yaw = 1500;
		pitch = 1500;
		roll = 1500;
	}

	int16_t throttle;
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
};

struct QuadAckPayload : public Radio::Payload
{
	void* data() override
	{
		return &armed_status;
	}

	uint32_t size() override
	{
		return 32;
	}

	void reset() override
	{
		armed_status = DISARMED;
		packets_per_second = 0;
		bat_voltage = 0.0f;
		yaw = 0.0f;
		pitch = 0.0f;
		roll = 0.0f;
	}

	uint8_t armed_status;
	uint16_t packets_per_second;
	float bat_voltage;
	float roll;
	float pitch;
	float yaw;

	// VESC
	float temp_mos_avg;
	float current_motor;
	uint32_t tachometer_abs;
};

extern QuadPayload payload;
extern QuadAckPayload ackPayload;