
uint8_t motor_pin = B00000100; // 2
uint8_t servo1_pin = B00001000; // 3
uint8_t servo2_pin = B00010000; // 4

#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

#include "Settings.h"
#include "Config.h"

#include "Gyro.h"
#include "Radio.h"
#include "PID.h"

Gyro gyro;
Radio radio;
PID pid;

float gyro_output;

int16_t receiver_input_roll, receiver_input_pitch, receiver_input_throttle, receiver_input_yaw;

uint16_t motor_speed, servo1_speed, servo2_speed;

float pid_setpoint;

const float deadzoneBuffer = 8.0;
const float deadzoneMin = 1500 - deadzoneBuffer;
const float deadzoneMax = 1500 + deadzoneBuffer;
const float maxRollRate = 164.0;
const float setpoint = (500 - deadzoneBuffer) / maxRollRate;

uint32_t loop_counter;
uint32_t loop_start_time;

enum _armedState
{
	DISARMED,
	PENDING,   // yaw stick has been moved to the side but not yet returned to center
	ARMED
};

_armedState armed_state;

void calibrateESCs()
{
	int dt = 15;

	for (int i = 0; i < dt; ++i)
	{
		Serial.print(dt - i);
		Serial.println("...");
		delay(1000);
	}

	Serial.println("Beginning ESC Calibration...");

	// Give ESCs a high pulse for approximately 5 seconds
	for (int i = 0; i < 1000; i++)
	{
		PORTD |= motor_pin;
		delayMicroseconds(config.max_esc_pulse);
		PORTD &= ~motor_pin;
		delay(3);
	}
	// Give ESCs a low pulse for approximately 5 seconds
	for (int i = 0; i < 1000; i++)
	{
		PORTD |= motor_pin;
		delayMicroseconds(config.min_esc_pulse);
		PORTD &= ~motor_pin;
		delay(3);
	}

	Serial.println("ESC Calibration finished");

	while (true);
}

void armESCs()
{
	for (int i = 0; i < 1000; i++)
	{
		PORTD |= motor_pin;
		delayMicroseconds(config.min_esc_pulse);
		PORTD &= ~motor_pin;

		delayMicroseconds(3000);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
	Serial.begin(115200);

	setupConfig();

	pid.setup(config.pid_p_gain,
		config.pid_i_gain,
		config.pid_d_gain,
		config.pid_max,
		config.pid_max);

	gyro.setup(config);

	radio.setup(config);

	DDRD |= (motor_pin | servo1_pin | servo2_pin);

	//calibrateESCs();

	armESCs();

	gyro.calibrate(config);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
	if (!radio.update())
	{
		armed_state = DISARMED;
	}

	gyro.update();

	gyro_output = (gyro_output * 0.8f) + (gyro.getPitch() * 0.2f);

	//Serial.println(gyro_output);

	receiver_input_throttle = radio.getData().throttle;
	receiver_input_yaw = radio.getData().yaw;
	receiver_input_pitch = radio.getData().pitch;
	receiver_input_roll = radio.getData().roll;

	//For starting the motors: throttle low and yaw left (step 1).
	if (receiver_input_throttle < 1050 && receiver_input_yaw < 1050)
	{
		armed_state = PENDING;
	}

	//When yaw stick is back in the center position start the motors (step 2).
	if (armed_state == PENDING && receiver_input_throttle < 1050 && receiver_input_yaw > 1450)
	{
		armed_state = ARMED;

		//Reset the pid controllers for a bumpless start.
		pid.reset();
	}

	//Stopping the motors: throttle low and yaw right.
	if (receiver_input_throttle < 1050 && receiver_input_yaw > 1950)
	{
		armed_state = DISARMED;
	}

	pid_setpoint = 0;
	//We need a little dead band of 16us for better results.
	if (receiver_input_pitch > deadzoneMax)pid_setpoint = (receiver_input_pitch - deadzoneMax) / setpoint;
	else if (receiver_input_pitch < deadzoneMin)pid_setpoint = (receiver_input_pitch - deadzoneMin) / setpoint;

	pid_setpoint *= config.rc_pitch_rate * config.rc_invert_pitch;

	pid.update(gyro_output, pid_setpoint);

	Serial.println(pid.getOutput());

	motor_speed = 1000;

	if (armed_state == ARMED && receiver_input_throttle > 1050)
	{
		motor_speed = map(receiver_input_throttle, 1000, 2000, config.min_esc_pulse, config.max_esc_pulse);
	}

	servo1_speed = map(pid.getOutput(), -config.pid_max, config.pid_max, config.min_servo_pulse, config.max_servo_pulse);
	servo2_speed = map(pid.getOutput(), -config.pid_max, config.pid_max, config.max_servo_pulse, config.min_servo_pulse);
	//servo1_speed = map(receiver_input_pitch, 1000, 2000, config.min_servo_pulse, config.max_servo_pulse);
	//servo2_speed = map(receiver_input_pitch, 1000, 2000, config.max_servo_pulse, config.min_servo_pulse); 

	while (micros() - loop_start_time  < 1e6 / config.loop_freq);

	uint32_t loop_start_time = micros();

	PORTD |= (motor_pin | servo1_pin | servo2_pin);

	uint32_t tc_1 = motor_speed + loop_start_time;
	uint32_t tc_2 = servo1_speed + loop_start_time;
	uint32_t tc_3 = servo2_speed + loop_start_time;

	uint8_t cnt = 0;

	while (cnt < 3)
	{
		cnt = 0;
		uint32_t esc_loop_start_time = micros();
		if (tc_1 <= esc_loop_start_time) { PORTD &= ~motor_pin;  cnt++; }
		if (tc_2 <= esc_loop_start_time) { PORTD &= ~servo1_pin; cnt++; }
		if (tc_3 <= esc_loop_start_time) { PORTD &= ~servo2_pin; cnt++; }
	}

	loop_counter++;
}