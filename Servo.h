#pragma once


class Servo
{
public:

	void begin(int pin, int min, int max)
	{
		if (!digitalPinHasPWM(pin))
		{
			Serial.print("Pin: ");
			Serial.print(pin);
			Serial.println(" does not support PWM!");
		}

		m_pin = pin;
		m_min = min;
		m_max = max;
		setFrequency(200);

		digitalWriteFast(m_pin, LOW);
		pinMode(m_pin, OUTPUT);
	}

	void setFrequency(float freq)
	{
		m_frequency = freq;
		analogWriteFrequency(m_pin, m_frequency);
	}

	void write(float usec)
	{
		usec = constrain(usec, m_min, m_max);

		uint32_t duty = (uint32_t)(usec / (1e6 / m_frequency) * 4096.0f);

		analogWriteResolution(12);
		analogWrite(m_pin, duty);
	}

private:

	float m_frequency;

	uint8_t m_pin;
	float m_min;
	float m_max;
};