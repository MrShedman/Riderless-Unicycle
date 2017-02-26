#include "ParkingLegs.h"
#include "Pins.h"

ParkingLegs parkingLegs;

namespace
{
	volatile float phase, freq;

	void stepper_driver()
	{
		parkingLegs.stepperDriver();
	}
}

void ParkingLegs::begin()
{
	SPI2.begin();
	stepper.init(AMIS_CS_PIN);

	// Drive the NXT/STEP and DIR pins low initially.
	digitalWriteFast(AMIS_STP_PIN, LOW);
	pinMode(AMIS_STP_PIN, OUTPUT);
	digitalWriteFast(AMIS_DIR_PIN, LOW);
	pinMode(AMIS_DIR_PIN, OUTPUT);

	// Give the driver some time to power up.
	delay(1);

	// Reset the driver to its default settings.
	stepper.resetSettings();

	// Set the current limit.  You should change the number here to
	// an appropriate value for your particular system.
	stepper.setCurrentMilliamps(1000);

	// Set the number of microsteps that correspond to one full step.
	stepper.setStepMode(1);

	// Enable the motor outputs.
	//stepper.enableDriver();

	timer.begin(stepper_driver, 1e6 / drive_frequency);
	timer.priority(255);
}

void ParkingLegs::stepperDriver()
{
	phase += freq;
	if (phase >= drive_frequency)
	{
		digitalWriteFast(AMIS_STP_PIN, HIGH);
		delayMicroseconds(2);
		digitalWriteFast(AMIS_STP_PIN, LOW);
		digitalWriteFast(AMIS_DIR_PIN, HIGH);
		phase -= drive_frequency;
	}
	else if (phase < 0)
	{
		digitalWriteFast(AMIS_STP_PIN, HIGH);
		delayMicroseconds(2);
		digitalWriteFast(AMIS_STP_PIN, LOW);
		digitalWriteFast(AMIS_DIR_PIN, LOW);
		phase += drive_frequency;
	}
}

void ParkingLegs::setRPM(float rpm)
{
	if (freq != 0.0f && rpm == 0.0f)
	{
		stepper.sleep();
	}

	if (freq == 0.0f && rpm != 0.0f)
	{
		stepper.sleepStop();
	}

	freq = constrain((rpm / 60.0f) * (float)num_steps, -(float)max_step_rate, (float)max_step_rate);
}
