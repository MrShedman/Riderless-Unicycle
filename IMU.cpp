
#include "IMU.h"
#include "SPI.h"

#define DEFAULT_SAMPLE_FREQ	512.0f	// sample frequency in Hz
#define twoKpDef	(2.0f * 5.0f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.1f)	// 2 * integral gain


//============================================================================================
// Functions

//-------------------------------------------------------------------------------------------
// AHRS algorithm update

//Mahony::Mahony()
//{
//	twoKp = twoKpDef;	// 2 * proportional gain (Kp)
//	twoKi = twoKiDef;	// 2 * integral gain (Ki)


IMU::IMU(uint8_t csPin)
{
	_csPin = csPin; // SPI CS Pin
	_useSPIHS = false; // defaul to low speed SPI transactions until data reads start to occur

	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	integralFBx = 0.0f;
	integralFBy = 0.0f;
	integralFBz = 0.0f;
	invSampleFreq = 1.0f / sample_rate;
}

void IMU::begin()
{
	pinMode(_csPin, OUTPUT);

	// setting CS pin high
	digitalWriteFast(_csPin, HIGH);

	SPI.setMOSI(11);
	SPI.setMISO(12);
	SPI.setSCK(13);
	SPI.begin();

	//calibrate();
	
	writeRegister(PWR_MGMT_1, CLOCK_SEL_PLL);			// select clock source to gyro
	writeRegister(USER_CTRL, I2C_MST_EN);				// enable I2C master mode
	writeRegister(I2C_MST_CTRL, I2C_MST_CLK);			// set the I2C bus speed to 400 kHz
	writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);	// set AK8963 to Power Down
	writeRegister(PWR_MGMT_1, PWR_RESET);				// reset the MPU9250
	
	delay(1);

	writeAK8963Register(AK8963_CNTL2, AK8963_RESET);	// reset the AK8963
	writeRegister(PWR_MGMT_1, CLOCK_SEL_PLL);			// select clock source to gyro
	writeRegister(PWR_MGMT_2, SEN_ENABLE);				// enable accelerometer and gyro
	writeRegister(GYRO_CONFIG, gyroRange << 3);			// setup the gyro ranges
	writeRegister(ACCEL_CONFIG, accelRange << 3);		// setup the accel ranges

	_gyroScale = powf(2.0f, (float)gyroRange + 1.0f) * 125.0f / 32768.0f;
	_accelScale = powf(2.0f, (float)accelRange + 1.0f) / 32768.0f;

	writeRegister(USER_CTRL, I2C_MST_EN);				// enable I2C master mode
	writeRegister(I2C_MST_CTRL, I2C_MST_CLK);			// set the I2C bus speed to 400 kHz
	writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);	// set AK8963 to Power Down

	delay(100);
	
	writeAK8963Register(AK8963_CNTL1, AK8963_FUSE_ROM);	// set AK8963 to FUSE ROM access

	delay(100);
	
	uint8_t buff[3];
				
	// read the AK8963 ASA registers and compute magnetometer scale factors
	readAK8963Registers(AK8963_ASA, sizeof(buff), &buff[0]);
	_magScaleX = ((((float)buff[0]) - 128.0f) / (256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
	_magScaleY = ((((float)buff[1]) - 128.0f) / (256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
	_magScaleZ = ((((float)buff[2]) - 128.0f) / (256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla 
	
	writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);	// set AK8963 to Power Down

	delay(100);

	writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS2);// set AK8963 to 16 bit resolution, 100 Hz update rate

	delay(100);

	writeRegister(PWR_MGMT_1, CLOCK_SEL_PLL);			// select clock source to gyro

	// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
	uint8_t data[7];
	readAK8963Registers(AK8963_HXL, sizeof(data), &data[0]);

	writeRegister(CONFIG, dlpf);		// dlpf_cfg
	writeRegister(ACCEL_CONFIG2, dlpf);	// dlpf_cfg

	uint8_t srd = floorf((1000.0f / sample_rate) - 1.0f);

	writeRegister(SMPLRT_DIV, srd);	// sample rate divider

	for (uint8_t i = 0; i < 3; ++i)
	{
		pt1FilterInit(&ptfilters[i], accel_cut_off, 1.0f / sample_rate);
	}

	for (uint8_t i = 3; i < 6; ++i)
	{
		pt1FilterInit(&ptfilters[i], gyro_cut_off, 1.0f / sample_rate);
	}

	writeRegister(INT_PIN_CFG, 0x10);	// INT_ANYRD_2CLEAR
	writeRegister(INT_ENABLE, 0x01);	// INT enable
}

void IMU::calibrate()
{
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	// reset device, reset all registers, clear gyro and accelerometer bias registers
	writeRegister(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	delay(100);

	// get stable time source
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	writeRegister(PWR_MGMT_1, 0x01);
	writeRegister(PWR_MGMT_2, 0x00);
	delay(200);

	// Configure device for bias calculation
	writeRegister(INT_ENABLE, 0x00);   // Disable all interrupts
	writeRegister(FIFO_EN, 0x00);      // Disable FIFO
	writeRegister(PWR_MGMT_1, 0x00);   // Turn on internal clock source
	writeRegister(I2C_MST_CTRL, 0x00); // Disable I2C master
	writeRegister(USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	writeRegister(USER_CTRL, 0x0C);    // Reset FIFO and DMP
	delay(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	writeRegister(CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	writeRegister(SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	writeRegister(GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeRegister(ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t gyrosensitivity = 131;   // = 131 LSB/degrees/sec
	uint16_t accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeRegister(USER_CTRL, 0x40);   // Enable FIFO  
	writeRegister(FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
	delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	writeRegister(FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	readRegisters(FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count / 12;// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++)
	{
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
		readRegisters(FIFO_R_W, 12, &data[0]); // read data for averaging

		accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
		accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
		gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
		gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
		gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

		accel_bias[0] += (int32_t)accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t)accel_temp[1];
		accel_bias[2] += (int32_t)accel_temp[2];
		gyro_bias[0] += (int32_t)gyro_temp[0];
		gyro_bias[1] += (int32_t)gyro_temp[1];
		gyro_bias[2] += (int32_t)gyro_temp[2];
	}

	accel_bias[0] /= (int32_t)packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t)packet_count;
	accel_bias[2] /= (int32_t)packet_count;
	gyro_bias[0] /= (int32_t)packet_count;
	gyro_bias[1] /= (int32_t)packet_count;
	gyro_bias[2] /= (int32_t)packet_count;

	// Remove gravity from the z-axis accelerometer bias calculation
	if (accel_bias[2] > 0L)
	{
		accel_bias[2] -= (int32_t)accelsensitivity;
	}
	else
	{
		accel_bias[2] += (int32_t)accelsensitivity;
	}

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0] / 4) & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
	data[3] = (-gyro_bias[1] / 4) & 0xFF;
	data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
	data[5] = (-gyro_bias[2] / 4) & 0xFF;

	/*
	// Push gyro biases to hardware registers
	writeRegister8(MPU6050_ADDRESS, XG_OFFS_USRH, data[0]);// might not be supported in MPU6050
	writeRegister8(MPU6050_ADDRESS, XG_OFFS_USRL, data[1]);
	writeRegister8(MPU6050_ADDRESS, YG_OFFS_USRH, data[2]);
	writeRegister8(MPU6050_ADDRESS, YG_OFFS_USRL, data[3]);
	writeRegister8(MPU6050_ADDRESS, ZG_OFFS_USRH, data[4]);
	writeRegister8(MPU6050_ADDRESS, ZG_OFFS_USRL, data[5]);
	*/

	gyroBias[0] = (float)gyro_bias[0] / (float)gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
	gyroBias[1] = (float)gyro_bias[1] / (float)gyrosensitivity;
	gyroBias[2] = (float)gyro_bias[2] / (float)gyrosensitivity;


	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = { 0, 0, 0 }; // A place to hold the factory accelerometer trim biases
	readRegisters(XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int16_t)((int16_t)data[0] << 8) | data[1];
	readRegisters(YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int16_t)((int16_t)data[0] << 8) | data[1];
	readRegisters(ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int16_t)((int16_t)data[0] << 8) | data[1];

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = { 0, 0, 0 }; // Define array to hold mask bit for each accelerometer bias axis

	for (ii = 0; ii < 3; ii++)
	{
		if (accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0]) & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1]) & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2]) & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	/*
	// Push accelerometer biases to hardware registers
	writeRegister8(MPU6050_ADDRESS, XA_OFFSET_H, data[0]); // might not be supported in MPU6050
	writeRegister8(MPU6050_ADDRESS, XA_OFFSET_L_TC, data[1]);
	writeRegister8(MPU6050_ADDRESS, YA_OFFSET_H, data[2]);
	writeRegister8(MPU6050_ADDRESS, YA_OFFSET_L_TC, data[3]);
	writeRegister8(MPU6050_ADDRESS, ZA_OFFSET_H, data[4]);
	writeRegister8(MPU6050_ADDRESS, ZA_OFFSET_L_TC, data[5]);
	*/

	// Output scaled accelerometer biases for manual subtraction in the main program
	accelBias[0] = (float)accel_bias[0] / (float)accelsensitivity;
	accelBias[1] = (float)accel_bias[1] / (float)accelsensitivity;
	accelBias[2] = (float)accel_bias[2] / (float)accelsensitivity;
}

void IMU::read_raw()
{
	uint8_t buff[21];
	_useSPIHS = true; // use the high speed SPI for data readout

	readRegisters(ACCEL_XOUT_H, sizeof(buff), &buff[0]); // grab the data from the MPU9250

	data.axis[0] = (int16_t)(((int16_t)buff[0]) << 8) | buff[1];
	data.axis[1] = (int16_t)(((int16_t)buff[2]) << 8) | buff[3];
	data.axis[2] = (int16_t)(((int16_t)buff[4]) << 8) | buff[5];

	data.axis[3] = (int16_t)(((int16_t)buff[8]) << 8) | buff[9];
	data.axis[4] = (int16_t)(((int16_t)buff[10]) << 8) | buff[11];
	data.axis[5] = (int16_t)(((int16_t)buff[12]) << 8) | buff[13];

	data.axis[6] = (int16_t)(((int16_t)buff[15]) << 8) | buff[14];
	data.axis[7] = (int16_t)(((int16_t)buff[17]) << 8) | buff[16];
	data.axis[8] = (int16_t)(((int16_t)buff[19]) << 8) | buff[18];
}

void IMU::apply_calibration()
{
	/*
	data.axis[0] -= accelBias[0];
	data.axis[1] -= accelBias[1];
	data.axis[2] -= accelBias[2];
	data.axis[3] -= gyroBias[0];
	data.axis[4] -= gyroBias[1];
	data.axis[5] -= gyroBias[2];
	*/
}

void IMU::apply_inversion_and_scale()
{
	/*
	Serial.print(data.axis[0]);
	Serial.print("\t");
	Serial.print(data.axis[1]);
	Serial.print("\t");
	Serial.print(data.axis[2]);
	Serial.print("\t");
	Serial.print(data.axis[3]);
	Serial.print("\t");
	Serial.print(data.axis[4]);
	Serial.print("\t");
	Serial.println(data.axis[5]);
	*/

///*
	data.axis[0] *= _accelScale;
	data.axis[1] *= _accelScale;
	data.axis[2] *= _accelScale;

	data.axis[3] *= _gyroScale;
	data.axis[4] *= _gyroScale;
	data.axis[5] *= _gyroScale;

	data.axis[6] *= _magScaleX;
	data.axis[7] *= _magScaleY;
	data.axis[8] *= _magScaleZ;

	data.axis[3] -= gyroBias[0];
	data.axis[4] -= gyroBias[1];
	data.axis[5] -= gyroBias[2];

	//*/
	// custom adjustments
	// calibration appears broken at least for gyro
	//data.axis[3] += 10.4f;
	//data.axis[4] += 8.6f;
	//data.axis[5] += 8.3f;

	/*
	Serial.print(accelBias[0]);
	Serial.print("\t");
	Serial.print(accelBias[1]);
	Serial.print("\t");
	Serial.print(accelBias[2]);
	Serial.print("\t");
	Serial.print(gyroBias[0]);
	Serial.print("\t");
	Serial.print(gyroBias[1]);
	Serial.print("\t");
	Serial.println(gyroBias[2]);
	*/
	/*
	Serial.print(data.axis[0]);
	Serial.print("\t");
	Serial.print(data.axis[1]);
	Serial.print("\t");
	Serial.print(data.axis[2]);
	Serial.print("\t");
	Serial.print(data.axis[3]);
	Serial.print("\t");
	Serial.print(data.axis[4]);
	Serial.print("\t");
	Serial.println(data.axis[5]);
	*/
}

void IMU::apply_filters()
{
	for (uint8_t i = 0; i < 6; ++i)
	{
		data.axis[i] = pt1FilterApply(&ptfilters[i], data.axis[i]);
	}
}

void IMU::update_fusion()
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Convert gyroscope degrees/sec to radians/sec
	data.gx *= 0.0174533f;
	data.gy *= 0.0174533f;
	data.gz *= 0.0174533f;
	
	// Compute feedback only if accelerometer measurement valid
	// (avoids NaN in accelerometer normalisation)
	if (!((data.ax == 0.0f) && (data.ay == 0.0f) && (data.az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = 1.0f / sqrtf(data.ax * data.ax + data.ay * data.ay + data.az * data.az);
		data.ax *= recipNorm;
		data.ay *= recipNorm;
		data.az *= recipNorm;

		// Estimated direction of gravity
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		// Error is sum of cross product between estimated
		// and measured direction of gravity
		halfex = (data.ay * halfvz - data.az * halfvy);
		halfey = (data.az * halfvx - data.ax * halfvz);
		halfez = (data.ax * halfvy - data.ay * halfvx);

		// Compute and apply integral feedback if enabled
		if (twoKi > 0.0f)
		{
			// integral error scaled by Ki
			integralFBx += twoKi * halfex * invSampleFreq;
			integralFBy += twoKi * halfey * invSampleFreq;
			integralFBz += twoKi * halfez * invSampleFreq;
			data.gx += integralFBx;	// apply integral feedback
			data.gy += integralFBy;
			data.gz += integralFBz;
		}
		else 
		{
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		data.gx += twoKp * halfex;
		data.gy += twoKp * halfey;
		data.gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	data.gx *= (0.5f * invSampleFreq);		// pre-multiply common factors
	data.gy *= (0.5f * invSampleFreq);
	data.gz *= (0.5f * invSampleFreq);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * data.gx - qc * data.gy - q3 * data.gz);
	q1 += (qa * data.gx + qc * data.gz - q3 * data.gy);
	q2 += (qa * data.gy - qb * data.gz + q3 * data.gx);
	q3 += (qa * data.gz + qb * data.gy - qc * data.gx);

	// Normalise quaternion
	recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

void IMU::calculate_angles()
{	
	// compute angles
	data.roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	data.pitch = asinf(-2.0f * (q1*q3 - q0*q2));
	data.yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);

	// rad to deg
	data.roll *= 57.29578f;
	data.pitch *= 57.29578f;
	data.yaw *= 57.29578f;
}

bool IMU::writeRegister(uint8_t subAddress, uint8_t data)
{
	uint8_t buff[1];

	SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
	digitalWriteFast(_csPin, LOW); // select the MPU9250 chip
	SPI.transfer(subAddress); // write the register address
	SPI.transfer(data); // write the data
	digitalWriteFast(_csPin, HIGH); // deselect the MPU9250 chip
	SPI.endTransaction(); // end the transaction

	delay(10); // need to slow down how fast I write to MPU9250

	/* read back the register */
	readRegisters(subAddress, sizeof(buff), &buff[0]);

	/* check the read back register against the written register */
	if (buff[0] == data) {
		return true;
	}
	else
	{
		Serial.print("Error in writing register:");
		Serial.print(subAddress);
		Serial.print("\t");
		Serial.println(subAddress);
		return false;
	}
}

void IMU::readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
	// begin the transaction
	if (_useSPIHS) 
	{
		SPI.beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
	}
	else 
	{
		SPI.beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
	}

	digitalWriteFast(_csPin, LOW); // select the MPU9250 chip

	SPI.transfer(subAddress | SPI_READ); // specify the starting register address

	for (uint8_t i = 0; i < count; i++) 
	{
		dest[i] = SPI.transfer(0x00); // read the data
	}

	digitalWriteFast(_csPin, HIGH); // deselect the MPU9250 chip
	SPI.endTransaction(); // end the transaction
}

bool IMU::writeAK8963Register(uint8_t subAddress, uint8_t data)
{
	uint8_t count = 1;
	uint8_t buff[1];

	writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR); // set slave 0 to the AK8963 and set for write
	writeRegister(I2C_SLV0_REG, subAddress); // set the register to the desired AK8963 sub address
	writeRegister(I2C_SLV0_DO, data); // store the data for write
	writeRegister(I2C_SLV0_CTRL, I2C_SLV0_EN | count); // enable I2C and send 1 byte
	
	// read the register and confirm
	readAK8963Registers(subAddress, sizeof(buff), &buff[0]);

	if (buff[0] == data) {
		return true;
	}
	else {
		return false;
	}
}

void IMU::readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
	writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG); // set slave 0 to the AK8963 and set for read
	writeRegister(I2C_SLV0_REG, subAddress); // set the register to the desired AK8963 sub address
	writeRegister(I2C_SLV0_CTRL, I2C_SLV0_EN | count); // enable I2C and request the bytes
	delayMicroseconds(100); // takes some time for these registers to fill
	readRegisters(EXT_SENS_DATA_00, count, dest); // read the bytes off the MPU9250 EXT_SENS_DATA registers
}

uint8_t IMU::whoAmI()
{
	uint8_t buff[1];

	// read the WHO AM I register
	readRegisters(WHO_AM_I_MPU9250, sizeof(buff), &buff[0]);

	// return the register value
	return buff[0];
}

uint8_t IMU::whoAmIAK8963()
{
	uint8_t buff[1];

	// read the WHO AM I register
	readAK8963Registers(AK8963_WHO_AM_I, sizeof(buff), &buff[0]);

	// return the register value
	return buff[0];
}