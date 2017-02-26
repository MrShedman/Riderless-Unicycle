
#include "VESC.h"
#include "buffer.h"
#include "crc.h"

VESC vesc;

namespace
{
	void read_interrupt()
	{
		vesc.getValues();
	}
}

//#define DEBUG

int VESC::receiveMessage(uint8_t* payloadReceived)
{
	//Messages <= 255 start with 2. 2nd byte is length
	//Messages >255 start with 3. 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF
	 
	int counter = 0;
	int endMessage = 256;
	bool messageRead = false;
	uint8_t messageReceived[256];
	int lenPayload = 0;

	while (Serial1.available())
	{
		messageReceived[counter++] = Serial1.read();

		if (counter == 2) //case if state of 'counter' with last read 1
		{
			switch (messageReceived[0])
			{
			case 2:
				endMessage = messageReceived[1] + 5; //Payload size + 2 for sice + 3 for SRC and End.
				lenPayload = messageReceived[1];
				break;
			case 3:
				//ToDo: Add Message Handling > 255 (starting with 3)
				break;
			default:
				break;
			}
		}
		if (counter >= sizeof(messageReceived))
		{
			break;
		}

		//+1: Because of counter++ state of 'counter' with last read = "endMessage"
		if (counter == endMessage && messageReceived[endMessage - 1] == 3) 
		{
			messageReceived[endMessage] = 0;
#ifdef DEBUG
			Serial.println("End of message reached!");
#endif			
			messageRead = true;
			break; //Exit if end of message is reached, even if there is still more data in buffer. 
		}
	}
	bool unpacked = false;
	if (messageRead) 
	{
		unpacked = unpackPayload(messageReceived, endMessage, payloadReceived, messageReceived[1]);
	}
	if (unpacked)
	{
		return lenPayload; //Message was read
	}
	else
	{
		return 0; //No Message Read
	}
}

bool VESC::unpackPayload(uint8_t* message, int lenMes, uint8_t* payload, int lenPay)
{
	uint16_t crcMessage = 0;
	uint16_t crcPayload = 0;
	//Rebuild src:
	crcMessage = message[lenMes - 3] << 8;
	crcMessage &= 0xFF00;
	crcMessage += message[lenMes - 2];
#ifdef DEBUG
	Serial.print("SRC received: "); Serial.println(crcMessage);
#endif // DEBUG

	//Extract payload:
	memcpy(payload, &message[2], message[1]);

	crcPayload = crc16(payload, message[1]);
#ifdef DEBUG
	Serial.print("SRC calc: "); Serial.println(crcPayload);
#endif
	if (crcPayload == crcMessage)
	{
#ifdef DEBUG
		//Serial.print("Received: "); SerialPrint(message, lenMes); Serial.println();
		//Serial.print("Payload :      "); SerialPrint(payload, message[1] - 1); Serial.println();
#endif // DEBUG

		return true;
	}
	else
	{
		return false;
	}
}

int VESC::packSendPayload(uint8_t* payload, int lenPay)
{
	uint16_t crcPayload = crc16(payload, lenPay);
	int count = 0;
	uint8_t messageSend[256];

	if (lenPay <= 256)
	{
		messageSend[count++] = 2;
		messageSend[count++] = lenPay;
	}
	else
	{
		messageSend[count++] = 3;
		messageSend[count++] = (uint8_t)(lenPay >> 8);
		messageSend[count++] = (uint8_t)(lenPay & 0xFF);
	}
	memcpy(&messageSend[count], payload, lenPay);

	count += lenPay;
	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;
	messageSend[count] = NULL;

#ifdef DEBUG
	Serial.print("UART package send: ");// SerialPrint(messageSend, count);

#endif // DEBUG

	//Sending package
	Serial1.write(messageSend, count);

	//Returns number of send bytes
	return count;
}

bool VESC::processReadPacket(uint8_t* message, mc_values& values, int len)
{
	COMM_PACKET_ID packetId = (COMM_PACKET_ID)message[0];
	int32_t ind = 0;

	message++;//Eliminates the message id
	len--;

	switch (packetId)
	{
	case COMM_GET_VALUES:
		values.temp_mos1 =			buffer_get_float16(message, 1e1, &ind);
		values.temp_mos2 =			buffer_get_float16(message, 1e1, &ind);
		values.temp_mos3 =			buffer_get_float16(message, 1e1, &ind);
		values.temp_mos4 =			buffer_get_float16(message, 1e1, &ind);
		values.temp_mos5 =			buffer_get_float16(message, 1e1, &ind);
		values.temp_mos6 =			buffer_get_float16(message, 1e1, &ind);
		values.temp_pcb =			buffer_get_float16(message, 1e1, &ind);
		values.current_motor =		buffer_get_float32(message, 1e2, &ind);
		values.current_in =			buffer_get_float32(message, 1e2, &ind);
		values.duty_now =			buffer_get_float16(message, 1e3, &ind);
		values.rpm =				buffer_get_float32(message, 1e0, &ind);
		values.v_in =				buffer_get_float16(message, 1e1, &ind);
		values.amp_hours =			buffer_get_float32(message, 1e4, &ind);
		values.amp_hours_charged =	buffer_get_float32(message, 1e4, &ind);
		values.watt_hours =			buffer_get_float32(message, 1e4, &ind);
		values.watt_hours_charged = buffer_get_float32(message, 1e4, &ind);
		values.tachometer =			buffer_get_int32(message, &ind);
		values.tachometer_abs =		buffer_get_int32(message, &ind);
		values.fault_code =			(mc_fault_code)message[ind++];
		
		return true;
		break;

	default:
		return false;
		break;
	}
}

void VESC::requestValues()
{
	uint8_t command[1] = { COMM_GET_VALUES };
	packSendPayload(command, 1);

	m_read_timer.begin(read_interrupt, m_read_delay);
	m_read_timer.priority(255); // Lowest priority
}

void VESC::setRPM(float rpm)
{
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_RPM;
	buffer_append_float32(payload, rpm, 1e0, &index);
	packSendPayload(payload, 5);
}

void VESC::setDuty(float duty)
{
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_DUTY;
	buffer_append_float32(payload, duty, 100000.0, &index);
	packSendPayload(payload, 5);
}

void VESC::setCurrent(float current)
{
	int32_t index = 0;
	uint8_t payload[5];
		
	payload[index++] = COMM_SET_CURRENT;
	buffer_append_float32(payload, current, 1e3, &index);
	packSendPayload(payload, 5);
}

void VESC::setCurrentBrake(float brakeCurrent)
{
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT_BRAKE;
	buffer_append_float32(payload, brakeCurrent, 1e3, &index);
	packSendPayload(payload, 5);
}

void VESC::print(uint8_t* data, int len)
{
	for (int i = 0; i <= len; i++)
	{
		Serial.print(data[i]);
		Serial.print(" ");
	}
	Serial.println("");
}

void VESC::print(const mc_values& values)
{
	Serial.print("avgMotorCurrent: ");	Serial.println(values.current_motor);
	Serial.print("avgInputCurrent: ");	Serial.println(values.current_in);
	Serial.print("dutyCycleNow: ");		Serial.println(values.duty_now);
	Serial.print("rpm: ");				Serial.println(values.rpm);
	Serial.print("inputVoltage: ");		Serial.println(values.v_in);
	Serial.print("ampHours: ");			Serial.println(values.amp_hours);
	Serial.print("ampHoursCharges: ");	Serial.println(values.amp_hours_charged);
	Serial.print("tachometer: ");		Serial.println(values.tachometer);
	Serial.print("tachometerAbs: ");	Serial.println(values.tachometer_abs);
	//Serial.print("MOSFET Temps: ");	Serial.println(values.tachometer_abs);
}