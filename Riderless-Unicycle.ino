
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

pt1Filter_t rc_filters[4];

pt1Filter_t pitch_filter;

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
//const float deadzoneMin = 1500 - deadzoneBuffer;
//const float deadzoneMax = 1500 + deadzoneBuffer;
//const float maxRollRate = 0.1f;
//const float setpoint = (500 - deadzoneBuffer) / maxRollRate;

float rcCommand[4];
float rcRates[4];

float calcSetpoints(float rc, float rate, float deadzone)
{
	const float deadzoneMin = 1500 - deadzone;
	const float deadzoneMax = 1500 + deadzone;

	float setpoint = 0.0f;

	if (rc > deadzoneMax)
	{
		setpoint = (rc - deadzoneMax) / (500 - deadzone)  * rate;
	}
	else if (rc < deadzoneMin)
	{
		setpoint = (rc - deadzoneMin) / (500 - deadzone) *  rate;
	}

	return setpoint;
}

void rcCommandFilter()
{
	payload.throttle = constrain(payload.throttle, 1000, 2000);
	payload.roll = constrain(payload.roll, 1000, 2000);
	payload.pitch = constrain(payload.pitch, 1000, 2000);
	payload.yaw = constrain(payload.yaw, 1000, 2000);

	rcCommand[THROTTLE] = pt1FilterApply(&rc_filters[0], payload.throttle);
	rcCommand[ROLL] = pt1FilterApply(&rc_filters[1], payload.roll);
	rcCommand[PITCH] = pt1FilterApply(&rc_filters[2], payload.pitch);
	rcCommand[YAW] = pt1FilterApply(&rc_filters[3], payload.yaw);
}

IMU imu(IMU_CS_PIN);

void imu_interrupt()
{
	imu.update();
	auto& d = imu.get_data();

	axisPID[PIDROLL].update(d.roll, pid_setpoint[PIDROLL], rcCommand[THROTTLE], 2000);
	axisPID[PIDPITCH].update(d.pitch, pid_setpoint[PIDPITCH], rcCommand[THROTTLE], 2000);
	axisPID[PIDYAW].update(-d.gz, pid_setpoint[PIDYAW], rcCommand[THROTTLE], 2000);
}

float main_loop_hz = 200;
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

	for (uint8_t i = 0; i < main_loop_hz; ++i)
	{
		radio.update();
		rcCommandFilter();

		delayMicroseconds(1e6 / main_loop_hz);
	}

	rcRates[THROTTLE] = 1.0f;
	rcRates[ROLL] = 5.0f;
	rcRates[PITCH] = 1.0f;
	rcRates[YAW] = 50.0f;

	//begin();
	//pinMode(BEEP_PIN, OUTPUT);
	//beeper(BEEPER_GYRO_CALIBRATED);
}

void loop()
{
	//sd_begin();

	radio.update();

	rcCommandFilter();

	//Serial.println(payload.buttons);

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

		imu.resetYaw();
	}

	//Stopping the motors: throttle low and yaw right.
	if (rcCommand[THROTTLE] < 1050 && rcCommand[YAW] > 1950)
	{
		armed_state = DISARMED;
	}

	pid_setpoint[PIDROLL] = calcSetpoints(rcCommand[ROLL], rcRates[ROLL], deadzoneBuffer);
	pid_setpoint[PIDPITCH] = calcSetpoints(rcCommand[PITCH], rcRates[PITCH], deadzoneBuffer);
	pid_setpoint[PIDYAW] = calcSetpoints(rcCommand[YAW], rcRates[YAW], deadzoneBuffer);

	/*
	pid_setpoint[PIDROLL] = 0;
	//We need a little dead band of 16us for better results.
	if (rcCommand[ROLL] > deadzoneMax)pid_setpoint[PIDROLL] = (rcCommand[ROLL] - deadzoneMax) / (500 - deadzoneBuffer)  *  rcRates[ROLL];
	else if (rcCommand[ROLL] < deadzoneMin)pid_setpoint[PIDROLL] = (rcCommand[ROLL] - deadzoneMin) / (500 - deadzoneBuffer) *  rcRates[ROLL];

	// offset due to CoM being slightly off-centre
	// found from CAD model
	pid_setpoint[PIDROLL] += 0.4f;//0.62f;

	pid_setpoint[PIDPITCH] = 0;
	//We need a little dead band of 16us for better results.
	if (rcCommand[PITCH] > deadzoneMax)pid_setpoint[PIDPITCH] = (rcCommand[PITCH] - deadzoneMax) / (500 - deadzoneBuffer) *  rcRates[PITCH];
	else if (rcCommand[PITCH] < deadzoneMin)pid_setpoint[PIDPITCH] = (rcCommand[PITCH] - deadzoneMin) / (500 - deadzoneBuffer) *  rcRates[PITCH];

	pid_setpoint[PIDYAW] = 0;
	//We need a little dead band of 16us for better results.
	if (rcCommand[YAW] > deadzoneMax)pid_setpoint[PIDYAW] = (rcCommand[YAW] - deadzoneMax) / (500 - deadzoneBuffer) * rcRates[YAW];
	else if (rcCommand[YAW] < deadzoneMin)pid_setpoint[PIDYAW] = (rcCommand[YAW] - deadzoneMin) / (500 - deadzoneBuffer) *  rcRates[YAW];

	*/

	float motor_speed = 1000;

	if (armed_state == ARMED && rcCommand[THROTTLE] > 1050)
	{
		motor_speed = rcCommand[THROTTLE];
	}

	float servo1_speed = axisPID[PIDROLL].getOutput();
	float servo2_speed = axisPID[PIDROLL].getOutput();

	if (payload.buttons == 0)
	{
		servo1_speed -= axisPID[PIDYAW].getOutput();
		servo2_speed += axisPID[PIDYAW].getOutput();

		//pid_setpoint[PIDROLL] = pid_setpoint[PIDYAW] * 0.01f;
	}

	Serial.println(pid_setpoint[PIDROLL]);

	servo1_speed = mapf(servo1_speed, -pid_profile[PIDROLL].max_Out, pid_profile[PIDROLL].max_Out, 1000, 2000);
	servo2_speed = mapf(servo2_speed, -pid_profile[PIDROLL].max_Out, pid_profile[PIDROLL].max_Out, 1000, 2000);

	//servo1_speed = rcCommand[PITCH];
	//servo2_speed = rcCommand[PITCH];

	balance.setPropSpeed(motor_speed);
	balance.setServoPositions(servo1_speed, servo2_speed);

	float rpm = mapf(rcCommand[ROLL] , 1000, 2000, -300, 300);
	
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

	float current = current_pid;// pt1FilterApply(&pitch_filter, current_pid);
	
	//Serial.println(pid_setpoint[PIDROLL]);

	//Serial.print(current);
	//Serial.print("\t");

	if (armed_state != ARMED)
	{
		current = 0.0f;
	}

	const float max_current = 10.0f;
	const float max_duty = 0.2f;

	//current = mapf(current, -pid_profile[PIDPITCH].max_Out, pid_profile[PIDPITCH].max_Out, -max_current, max_current);
	current = mapf(current, -pid_profile[PIDPITCH].max_Out, pid_profile[PIDPITCH].max_Out, -max_duty, max_duty);

	float in = mapf(rcCommand[PITCH], 1000, 2000, -1.0f, 1.0f);

	//in = applyDeadbandf(in, 0.05f);

	float duty_rc = 0.0f;

//	Serial.println(in * 100);

	if (in > 0.05f)
	{
		duty_rc = mapf(in, 0.00f, 1.0f, 0.05f, max_duty);
	}
	else if (in < -0.05f)
	{
		duty_rc = mapf(in, -1.0f, 0.00f, -max_duty, -0.05f);
	}

	//float duty_rc = mapf(in, 1000, 2000, -max_duty, max_duty);

	if (armed_state != ARMED)
	{
		duty_rc = 0.0f;
	}

	vesc.setDuty(duty_rc);

	//printData();
	
	while (micros() - loop_start_time < 1e6 / main_loop_hz);

	loop_start_time = micros();
}

void printData()
{
	Serial.println(imu.get_data().yaw);
	return;
	/*auto &d = imu.get_data();

	Serial.print(d.axis[0]);
	Serial.print("\t");
	Serial.print(d.axis[1]);
	Serial.print("\t");
	Serial.print(d.axis[2]);
	Serial.print("\t");
	Serial.print(d.axis[3]);
	Serial.print("\t");
	Serial.print(d.axis[4]);
	Serial.print("\t");
	Serial.println(d.axis[5]);

	return;*/

	///*
	Serial.print(imu.get_data().roll);
	Serial.print("\t");
	Serial.println(imu.get_data().pitch);
	//Serial.println(axisPID[PIDROLL].getOutput());// filter.getPitch());
	//Serial.print("\t");
	//Serial.println(filter.getYaw());
	//*/
	//Serial.println(imu.get_data().gx, 6);
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