
#include "Pins.h"
#include "Config.h"
#include "Beeper.h"
#include "Radio.h"
#include "Payloads.h"

QuadPayload payload;
QuadAckPayload ackPayload;

#include "ParkingLegs.h"
#include "VESC.h"
#include "IMU.h"
#include "Balance.h"
#include "PID.h"
#include "Filter.h"
#include "SDCard.h"
#include "Utility.h"

pt1Filter_t pitch_filter;

pt1Filter_t rc_filters[4];

typedef enum
{
	THROTTLE,
	ROLL, 
	PITCH,
	YAW
} 
rcIndex_e;

void initPIDS(const pidProfile_t profile[PID_ITEM_COUNT])
{
	for (uint8_t i = 0; i < PID_ITEM_COUNT; ++i)
	{
		axisPID[i].setup(&profile[i]);
	}
}

float pid_setpoint[PID_ITEM_COUNT];

const float deadzoneBuffer = 8.0;
const float deadzoneMin = 1500 - deadzoneBuffer;
const float deadzoneMax = 1500 + deadzoneBuffer;
const float maxRollRate = 60.0f;
const float setpoint = (500 - deadzoneBuffer) / maxRollRate;

float rcCommand[4];

IMU imu(IMU_CS_PIN);

float main_loop_hz = 200;

volatile int i = 0;

volatile bool imu_data_ready = false;
volatile uint32_t int_count = 0;

void imu_interrupt()
{
	int_count++;
	imu.update();
	auto& d = imu.get_data();

	axisPID[PIDROLL].update(d.roll, pid_setpoint[PIDROLL], rcCommand[THROTTLE], 2000);
	axisPID[PIDPITCH].update(d.pitch, pid_setpoint[PIDPITCH], rcCommand[THROTTLE], 2000);
	axisPID[PIDYAW].update(d.gz, pid_setpoint[PIDYAW], rcCommand[THROTTLE], 2000);
}

uint32_t loop_counter;
uint32_t loop_start_time;

armedState armed_state = DISARMED;

/*void calibrateIMU()
{
	int32_t calibration_count = 2000;
	double scale = 1.0f / (double)calibration_count;

	while (calibration_count > 0)
	{
		if (imu_data_ready)
		{
			//imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &hx, &hy, &hz);
			gx *= 57.2958f;
			gy *= 57.2958f;
			gz *= 57.2958f;

			gyroBias[0] += gx;
			gyroBias[1] += gy;
			gyroBias[2] += gz;

			imu_data_ready = false;
			calibration_count--;
		}
	}

	gyroBias[0] *= scale;
	gyroBias[1] *= scale;
	gyroBias[2] *= scale;

	Serial.println("GYRO BIAS:");
	Serial.print(gyroBias[0], 6);
	Serial.print("\t");
	Serial.print(gyroBias[1], 6);
	Serial.print("\t");
	Serial.println(gyroBias[2], 6);
}*/

void setup()
{
	Serial.begin(115200);
	
	vesc.begin();

	resetPidProfile(pid_profile);
	initPIDS(pid_profile);

	payload.reset();
	ackPayload.reset();

	radio.setPayload(&payload, Radio::Payload::Normal);
	radio.setPayload(&ackPayload, Radio::Payload::Ack);

	balance.begin();

	parkingLegs.begin();

	imu.begin();

	pinMode(IMU_IRQ_PIN, INPUT);
	attachInterrupt(IMU_IRQ_PIN, imu_interrupt, RISING);

	//calibrateIMU();

	float rc_cut_off = 10.0f;

	for (uint8_t i = 0; i < 4; ++i)
	{
		pt1FilterInit(&rc_filters[i], rc_cut_off, 1.0f / main_loop_hz);
	}

	pt1FilterInit(&pitch_filter, 1.0f, 1.0f / main_loop_hz);

	radio.begin();

	//begin();
	//pinMode(BEEP_PIN, OUTPUT);
	//beeper(BEEPER_GYRO_CALIBRATED);
}

bool radio_bug = false;
QuadPayload bug_payload;
uint32_t bug_cc = 0;

const uint32_t settle_time = 1000;
const uint32_t sample_rate = 200;
const uint32_t increment = 10;
const uint32_t limit = 1500;

void loop() 
{
	//sd_begin();

	radio.update();

	if (payload.throttle < 1000 || payload.throttle > 2000 ||
		payload.roll < 1000 || payload.roll > 2000 ||
		payload.pitch < 1000 || payload.pitch > 2000 ||
		payload.yaw < 1000 || payload.yaw > 2000)
	{
		bug_payload = payload;
		radio_bug = true;
	}

	if (radio_bug)
	{
		if (bug_cc < 10)
		{
		Serial.print("Radio bug!!!");
		Serial.print(",");
		Serial.print(bug_payload.throttle);
		Serial.print(",");
		Serial.print(bug_payload.roll);
		Serial.print(",");
		Serial.print(bug_payload.pitch);
		Serial.print(",");
		Serial.println(bug_payload.yaw);
		}
		//delay(100);
		bug_cc++;

		//return;
	}
	else
	{
		//Serial.print(payload.throttle);
		//Serial.print(",");
		//Serial.print(payload.roll);
		//Serial.print(",");
		//Serial.print(payload.pitch);
		//Serial.print(",");
		//Serial.println(payload.yaw);
	}

	payload.throttle = constrain(payload.throttle, 1000, 2000);
	payload.roll = constrain(payload.roll, 1000, 2000);
	payload.pitch = constrain(payload.pitch, 1000, 2000);
	payload.yaw = constrain(payload.yaw, 1000, 2000);


	//Serial.print(payload.throttle);
	//Serial.print(",");

	rcCommand[THROTTLE] = pt1FilterApply(&rc_filters[0], payload.throttle);
	rcCommand[ROLL] = pt1FilterApply(&rc_filters[1], payload.roll);
	rcCommand[PITCH] = pt1FilterApply(&rc_filters[2], payload.pitch);
	rcCommand[YAW] = pt1FilterApply(&rc_filters[3], payload.yaw);

	/*
	//const uint32_t settle_time = 1000;
	//const uint32_t sample_rate = 100;
	//const uint32_t increment = 10;
	//const uint32_t limit = 1500;
	static bool test = false;
	static float avg = 0.0f;
	static uint32_t tc = 0;
	static float set_rpm = 1000.0f;
	static float mes_rpm = 0.0f;

	if (rcCommand[ROLL] > 1800)
	{
		test = true;
	}

	if (rcCommand[ROLL] < 1200)
	{
		test = false;
		avg = 0.0f;
		tc = 0;
		set_rpm = 1000.0f;
		mes_rpm = 0.0f;
	}
	
	if (test)
	{
		balance.setPropSpeed(set_rpm);// rcCommand[THROTTLE]);
		
		balance.setServoPositions(1500.0f, 1500.0f);

		avg += balance.printRPM();
		tc++;

		if (tc > settle_time)
		{
			mes_rpm = avg / (float)tc;
			tc = 0;
			avg = 0.0f;
			set_rpm += increment;

			Serial.print(rcCommand[THROTTLE]);
			Serial.print("\t");
			Serial.print(mes_rpm);
			Serial.print("\t");
			Serial.println(set_rpm);
		}

	}

	while (micros() - loop_start_time < 1e6 / main_loop_hz);

	loop_start_time = micros();

	return;

	*/

	//Serial.println(rcCommand[THROTTLE]);

	if (!radio.hasConnection())
	{
		armed_state = DISARMED;
	}

	//For starting the motors: throttle low and yaw left (step 1).
	if (rcCommand[THROTTLE] < 1050 && rcCommand[YAW] < 1050)
	{
		armed_state = PENDING;
	}

	//When yaw stick is back in the center position start the motors (step 2).
	if (armed_state == PENDING && rcCommand[THROTTLE] < 1050 && rcCommand[YAW] > 1450)
	{
		armed_state = ARMED;

		//Reset the pid controllers for a bumpless start.
		axisPID[PIDROLL].reset();
		axisPID[PIDPITCH].reset();
		axisPID[PIDYAW].reset();
	}

	//Stopping the motors: throttle low and yaw right.
	if (rcCommand[THROTTLE] < 1050 && rcCommand[YAW] > 1950)
	{
		armed_state = DISARMED;
	}

	pid_setpoint[PIDROLL] = 0;
	//We need a little dead band of 16us for better results.
	if (rcCommand[ROLL] > deadzoneMax)pid_setpoint[PIDROLL] = (rcCommand[ROLL] - deadzoneMax) / setpoint;
	else if (rcCommand[ROLL] < deadzoneMin)pid_setpoint[PIDROLL] = (rcCommand[ROLL] - deadzoneMin) / setpoint;

	// offset due to CoM being slightly off-centre
	// found from CAD model
	pid_setpoint[PIDROLL] += 0.62f;

	pid_setpoint[PIDPITCH] = 0;
	//We need a little dead band of 16us for better results.
	if (rcCommand[PITCH] > deadzoneMax)pid_setpoint[PIDPITCH] = (rcCommand[PITCH] - deadzoneMax) / setpoint;
	else if (rcCommand[PITCH] < deadzoneMin)pid_setpoint[PIDPITCH] = (rcCommand[PITCH] - deadzoneMin) / setpoint;

	pid_setpoint[PIDYAW] = 0;
	//We need a little dead band of 16us for better results.
	if (rcCommand[YAW] > deadzoneMax)pid_setpoint[PIDYAW] = (rcCommand[YAW] - deadzoneMax) / setpoint;
	else if (rcCommand[YAW] < deadzoneMin)pid_setpoint[PIDYAW] = (rcCommand[YAW] - deadzoneMin) / setpoint;

	float motor_speed = 1000;

	if (armed_state == ARMED && rcCommand[THROTTLE] > 1050)
	{
		motor_speed = rcCommand[THROTTLE];
	}

	float servo1_speed = axisPID[PIDROLL].getOutput() - axisPID[PIDYAW].getOutput();
	float servo2_speed = axisPID[PIDROLL].getOutput() + axisPID[PIDYAW].getOutput();

	servo1_speed = mapf(servo1_speed, -pid_profile[PIDROLL].max_Out, pid_profile[PIDROLL].max_Out, 1000, 2000);
	servo2_speed = mapf(servo2_speed, -pid_profile[PIDROLL].max_Out, pid_profile[PIDROLL].max_Out, 1000, 2000);

	balance.setPropSpeed(motor_speed);
	balance.setServoPositions(servo1_speed, servo2_speed);

	float rpm = 0.0f;// (float)mapf(rcCommand[ROLL] , 1000, 2000, -300, 300);
	
	if (abs(rpm) < 5.0f)
	{
		rpm = 0.0f;
	}

	parkingLegs.setRPM(rpm);
	
	vesc.update();

	ackPayload.armed_status = armed_state;
	ackPayload.roll = imu.get_data().roll;
	ackPayload.pitch = imu.get_data().pitch;
	ackPayload.yaw = imu.get_data().yaw;
	ackPayload.bat_voltage = vesc.motor_values.v_in;
	ackPayload.temp_mos_avg = vesc.getMaxMOSFETTemp();
	ackPayload.current_motor = vesc.motor_values.current_motor;
	ackPayload.tachometer_abs = vesc.motor_values.tachometer_abs;

	float current_pid = axisPID[PIDPITCH].getOutput();

	//Serial.print(current_pid);
	//Serial.print("\t");

	float current = pt1FilterApply(&pitch_filter, current_pid);
	
	//Serial.print(current);
	//Serial.print("\t");

	if (armed_state != ARMED)
	{
		current = 0.0f;
	}

	const float max_current = 10.0f;
	const float max_duty = 0.3f;

	//current = mapf(current, -pid_profile[PIDPITCH].max_Out, pid_profile[PIDPITCH].max_Out, -max_current, max_current);
	current = mapf(current, -pid_profile[PIDPITCH].max_Out, pid_profile[PIDPITCH].max_Out, -max_duty, max_duty);

	//Serial.println(current);
//	Serial.print("\t");

	if (abs(current) < 0.05f)
	{
		current = 0.0f;
	}

	//vesc.setCurrent(current * -1.0f);// current*0.03f);
	//vesc.setDuty(current * -1.0f);
	//printData();
	
	while (micros() - loop_start_time < 1e6 / main_loop_hz);

	loop_start_time = micros();
}

void printData()
{
	///*
	Serial.print(imu.get_data().roll);
	Serial.print("\t");
	Serial.println(imu.get_data().pitch);
	//Serial.println(axisPID[PIDROLL].getOutput());// filter.getPitch());
	//Serial.print("\t");
	//Serial.println(filter.getYaw());
	//*/
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