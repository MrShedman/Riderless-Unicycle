#include "SDCard.h"

#include "SdFat.h"

//SdFatSdio sd;

SdFatSdioEX sdEx;

uint32_t t = 0;
const uint32_t dt = 5000;

void sd_begin()
{
	if (millis() - t > dt)
	{
		t = millis();
	}
	else
	{
		return;
	}

	uint32_t t1 = micros();

	if (!sdEx.begin())
	{
		Serial.println("error");
		//sdEx.initErrorHalt("SdFatSdioEX begin() failed");
	}

	Serial.println("init time: ");
	Serial.print(micros() - t1);
	Serial.println("us");
}