#pragma once

#include <SPI.h>
#include <AMIS30543.h>

class ParkingLegs
{
public:

	void begin();

	void setRPM(float rpm);

	void stepperDriver();

private:

	const uint16_t max_step_rate = 1600;	//steps/second
	const uint16_t drive_frequency = 2500;	//Hz
	const uint16_t num_steps = 200;			//steps/rev

	IntervalTimer timer;

	AMIS30543 stepper;
};

extern ParkingLegs parkingLegs;