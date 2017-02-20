#include "Radio.h"
#include "Pins.h"

#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <printf.h>

Radio radio;

namespace
{
	volatile bool rx_available = false;

	void radio_interrupt()
	{
		radio.interruptHandler();
	}
}

Radio::Radio()
	:
	rf24(RF24_CE_PIN, RF24_CS_PIN),
	m_payload(nullptr),
	m_ackPayload(nullptr),
	m_restart_delay(2000),
	m_timeout(1000),
	m_update_rate(50),
	m_RSSI(0)
{}

void Radio::interruptHandler()
{
	bool tx, fail, rx;
	rf24.whatHappened(tx, fail, rx);

	if (tx)
	{
		//Serial.println("Send:OK");
	}
	if (fail)
	{
		//Serial.println("Send:FAIL");
	}
	if (rx)
	{ 
		rx_available = true;
	}
}

bool Radio::hasConnection()
{
	return ackCounter(m_timeout) > 0;
}

uint16_t Radio::ackCounter(uint32_t dt_ms)
{
	const uint32_t* h = m_fifo.head();
	const uint32_t dt = millis() - dt_ms;

	uint16_t counter = 0;

	for (uint8_t i = 0; i < m_fifo.size(); ++i)
	{
		if (*h > dt)
		{
			counter++;

			h = m_fifo.previous(h);
		}
		else
		{
			break;
		}
	}

	return counter;
}

void Radio::calculateRSSI()
{
	const uint16_t temp = (uint16_t)ceilf(0.8f * (float)m_update_rate + 6.86f);

	m_RSSI = constrain(ackCounter(1000) * 100 / temp, 0, 100);
}

void Radio::update()
{
	if (rx_available)
	{
		rf24.read(m_payload->data(), m_payload->size());
		m_fifo.push(millis());

		rf24.writeAckPayload(1, m_ackPayload->data(), m_ackPayload->size());

		rx_available = false;
	}

	calculateRSSI();

	if (!hasConnection())
	{
		m_payload->reset();
		m_ackPayload->reset();

		if (m_restart_timer > m_restart_delay)
		{
			enable(false);
			begin();

			m_restart_timer = 0;
		}
	}
}

void Radio::enable(bool flag)
{
	if (flag)
	{
		attachInterrupt(RF24_IRQ_PIN, radio_interrupt, LOW);
	}
	else
	{
		detachInterrupt(RF24_IRQ_PIN);
	}
}

void Radio::begin()
{
	rf24.begin();
	//radio.setChannel(125);
	rf24.setPALevel(RF24_PA_MAX);
	rf24.enableAckPayload();
	//radio.enableDynamicPayloads();
	rf24.setDataRate(RF24_1MBPS);
	rf24.setRetries(4, 1);
	rf24.openWritingPipe(address[1]);
	rf24.openReadingPipe(1, address[0]);
	rf24.startListening();

	rf24.printDetails();

	rf24.writeAckPayload(1, m_ackPayload->data(), m_ackPayload->size());

	m_fifo.clear();

	pinMode(RF24_IRQ_PIN, INPUT);
	enable(true);
}