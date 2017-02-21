

#include "Pins.h"
#include "Beeper.h"
#include "Radio.h"
#include "Payloads.h"

QuadPayload payload;
QuadAckPayload ackPayload;

#include "ParkingLegs.h"
#include "VESC.h"
#include "MPU9250.h"
#include "MadgwickAHRS.h"
#include "Balance.h"
#include "PID.h"

PID pid;

MPU9250 IMU(IMU_CS_PIN);

Madgwick filter;
float m_filter_hz = 200;

float ax, ay, az, gx, gy, gz, hx, hy, hz, t;
int beginStatus, setFiltStatus;
volatile int i = 0;

volatile bool imu_data_ready = false;

void imu_interrupt()
{
	imu_data_ready = true;
}

float pid_setpoint;

const float deadzoneBuffer = 8.0;
const float deadzoneMin = 1500 - deadzoneBuffer;
const float deadzoneMax = 1500 + deadzoneBuffer;
const float maxRollRate = 164.0;
const float setpoint = (500 - deadzoneBuffer) / maxRollRate;

uint32_t loop_counter;
uint32_t loop_start_time;

armedState armed_state;

float kp = 30.0f;
float ki = 0.01f;// 0.5f;// 0.1f;
float kd = 0.0f;// 5.0f;
float pid_max = 400.0f;

void setup() 
{
	// serial to display data
	Serial.begin(115200);

	vesc.begin();

	pid.setup(kp, ki, kd, pid_max, pid_max);

	payload.reset();
	ackPayload.reset();

	radio.setPayload(&payload, Radio::Payload::Normal);
	radio.setPayload(&ackPayload, Radio::Payload::Ack);

	balance.begin();

	parkingLegs.begin();

	filter.begin(m_filter_hz);

	beginStatus = IMU.begin(ACCEL_RANGE_4G, GYRO_RANGE_250DPS);

	if (beginStatus < 0) 
	{
		delay(1000);
		Serial.println("IMU initialization unsuccessful");
		while (1) {};
	}

	float SRD = (1000.0f / m_filter_hz) - 1.0f;
	setFiltStatus = IMU.setFilt(DLPF_BANDWIDTH_92HZ, SRD);
	
	pinMode(IMU_IRQ_PIN, INPUT);
	attachInterrupt(IMU_IRQ_PIN, imu_interrupt, RISING);

	radio.begin();

	pinMode(BEEP_PIN, OUTPUT);
	beeper(BEEPER_2_LONG);
}

float calibrated_values[3];
//transformation(float uncalibrated_values[3]) is the function of the magnetometer data correction 
//uncalibrated_values[3] is the array of the non calibrated magnetometer data
//uncalibrated_values[3]: [0]=Xnc, [1]=Ync, [2]=Znc
void transformation(float uncalibrated_values[3])
{
	//calibration_matrix[3][3] is the transformation matrix
	//replace M11, M12,..,M33 with your transformation matrix data
	float calibration_matrix[3][3] =
	{
		{ 0.537, -0.31, 0.444 },
		{ 0, 0.039, 0.088 },
		{ 0.001, -4.971, 0.006 }
	};
	//bias[3] is the bias
	//replace Bx, By, Bz with your bias data
	float bias[3] =
	{
		-25.699,
		-0.399,
		1.883
	};
	//calculation
	for (int i = 0; i<3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
	float result[3] = { 0, 0, 0 };
	for (int i = 0; i<3; ++i)
		for (int j = 0; j<3; ++j)
			result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
	for (int i = 0; i<3; ++i) calibrated_values[i] = result[i];
}

void loop() 
{
	radio.update();

	if (!radio.hasConnection())
	{
		armed_state = DISARMED;
	}

	//For starting the motors: throttle low and yaw left (step 1).
	if (payload.throttle < 1050 && payload.yaw < 1050)
	{
		armed_state = PENDING;
	}

	//When yaw stick is back in the center position start the motors (step 2).
	if (armed_state == PENDING && payload.throttle < 1050 && payload.yaw > 1450)
	{
		armed_state = ARMED;

		//Reset the pid controllers for a bumpless start.
		pid.reset();
	}

	//Stopping the motors: throttle low and yaw right.
	if (payload.throttle < 1050 && payload.yaw > 1950)
	{
		armed_state = DISARMED;
	}

	pid_setpoint = 0;
	//We need a little dead band of 16us for better results.
	//if (payload.pitch > deadzoneMax)pid_setpoint = (payload.pitch - deadzoneMax) / setpoint;
	//else if (payload.pitch < deadzoneMin)pid_setpoint = (payload.pitch - deadzoneMin) / setpoint;

	//pid_setpoint *= config.rc_pitch_rate * config.rc_invert_pitch;

	if (imu_data_ready)
	{
		IMU.getMotion10(&ax, &ay, &az, &gx, &gy, &gz, &hx, &hy, &hz, &t);
		imu_data_ready = false;
	}

	for (uint8_t i = 0; i < 20; ++i)
	{
		filter.update(gx, gy, gz, ax, ay, az, hx, hy, hz);
	}

	ackPayload.armed_status = armed_state;
	ackPayload.roll = filter.getRoll();
	ackPayload.pitch = filter.getPitch();
	ackPayload.yaw = filter.getYaw();

	pid.update(filter.getPitch(), pid_setpoint);

	Serial.print(filter.getPitch());
	Serial.print("\t");
	Serial.println(pid.getOutput());

	float motor_speed = 1000;

	if (armed_state == ARMED && payload.throttle > 1050)
	{
		motor_speed = payload.throttle;
	}

	float servo1_speed = map(pid.getOutput(), -pid_max, pid_max, 1000, 2000);
	float servo2_speed = map(pid.getOutput(), -pid_max, pid_max, 1000, 2000);

	balance.setPropSpeed(motor_speed);
	balance.setServoPositions(servo1_speed, servo2_speed);

	float rpm = (float)map(payload.roll, 1000, 2000, -100, 100);
	if (abs(rpm) < 5.0f)
	{
		rpm = 0.0f;
	}

	parkingLegs.setRPM(rpm);
	
	vesc.update();

	ackPayload.bat_voltage = vesc.motor_values.v_in;
	ackPayload.temp_mos_avg = vesc.getMaxMOSFETTemp();
	ackPayload.current_motor = vesc.motor_values.current_motor;
	ackPayload.tachometer_abs = vesc.motor_values.tachometer_abs;

	float current = (payload.pitch - 1500.0f) * 0.001f * 15;
	
	if (abs(current) < 1.5f)
	{
		current = 0.0f;
	}
	vesc.setCurrent(current);

	//printData();

	while (micros() - loop_start_time < 1e6 / m_filter_hz);

	loop_start_time = micros();
}

void printData()
{

	Serial.print(filter.getRoll());
	Serial.print("\t");
	Serial.print(filter.getPitch());
	Serial.print("\t");
	Serial.println(filter.getYaw());

/*
	// print the data
	Serial.print(ax, 6);
	Serial.print("\t");
	Serial.print(ay, 6);
	Serial.print("\t");
	Serial.print(az, 6);
	Serial.print("\t");

	Serial.print(gx, 6);
	Serial.print("\t");
	Serial.print(gy, 6);
	Serial.print("\t");
	Serial.print(gz, 6);
	Serial.print("\t");

	Serial.print(hx, 6);
	Serial.print("\t");
	Serial.print(hy, 6);
	Serial.print("\t");
	Serial.print(hz, 6);
	Serial.print("\t");

	Serial.println(t, 6);
	*/
}

/*
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
}*/