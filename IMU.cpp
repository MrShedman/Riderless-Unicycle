
#include "IMU.h"
#include "SPI.h"

IMU::IMU(uint8_t csPin)
{
	_csPin = csPin; // SPI CS Pin
	_useSPIHS = false; // defaul to low speed SPI transactions until data reads start to occur
}

void IMU::begin(mpu9250_accel_range accelRange, mpu9250_gyro_range gyroRange)
{
	pinMode(_csPin, OUTPUT);

	// setting CS pin high
	digitalWriteFast(_csPin, HIGH);

	SPI.setMOSI(11);
	SPI.setMISO(12);
	SPI.setSCK(13);
	SPI.begin();

	//calibrate();

	writeRegister(PWR_MGMT_1, 0x00);
	delay(100);

	// select clock source to gyro
	writeRegister(PWR_MGMT_1, 0x01);

	// dlpf_cfg
	writeRegister(CONFIG, 0x03);

	writeRegister(SMPLRT_DIV, 0x04);

	// setup the accel and gyro ranges
	writeRegister(GYRO_CONFIG, gyroRange << 3);
	writeRegister(ACCEL_CONFIG, accelRange << 3);

	_gyroScale = powf(2.0f, (float)gyroRange + 1.0f) * 125.0f / 32768.0f;
	_accelScale = powf(2.0f, (float)accelRange + 1.0f) / 32768.0f;

	writeRegister(USER_CTRL, 0x20);

	// set the I2C bus speed to 400 kHz
	writeRegister(I2C_MST_CTRL, 0x0D);

	// set AK8963 to Power Down
	writeAK8963Register(AK8963_CNTL1, 0x00);

	delay(10); // long wait between AK8963 mode changes

	// set AK8963 to FUSE ROM access
	writeAK8963Register(AK8963_CNTL1, 0x0F);

	delay(10); // long wait between AK8963 mode changes

	uint8_t buff[3];

	// read the AK8963 ASA registers and compute magnetometer scale factors
	readAK8963Registers(AK8963_ASAX, sizeof(buff), &buff[0]);
	_magScaleX = (((float)buff[0] - 128.0f) / 256.0f + 1.0f) * 10.f * 4912.0f / 32760.0f;
	_magScaleY = (((float)buff[1] - 128.0f) / 256.0f + 1.0f) * 10.f * 4912.0f / 32760.0f;
	_magScaleZ = (((float)buff[2] - 128.0f) / 256.0f + 1.0f) * 10.f * 4912.0f / 32760.0f;
																						   
	// set AK8963 to Power Down
	writeAK8963Register(AK8963_CNTL1, 0x00);

	delay(10); // long wait between AK8963 mode changes  

	// set AK8963 to 16 bit resolution, 100 Hz update rate
	writeAK8963Register(AK8963_CNTL1, 0x16);

	delay(10); // long wait between AK8963 mode changes

	writeRegister(INT_PIN_CFG, 0x10); //INT_ANYRD_2CLEAR
	writeRegister(INT_ENABLE, 0x01);
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

	data.ax = float(((int16_t)buff[0] << 8) | buff[1]);
	data.ay = float(((int16_t)buff[2] << 8) | buff[3]);
	data.az = float(((int16_t)buff[4] << 8) | buff[5]);

	data.gx = float(((int16_t)buff[8] << 8) | buff[9]);
	data.gy = float(((int16_t)buff[10] << 8) | buff[11]);
	data.gz = float(((int16_t)buff[12] << 8) | buff[13]);

	data.mx = float(((int16_t)buff[15] << 8) | buff[14]);
	data.my = float(((int16_t)buff[17] << 8) | buff[16]);
	data.mz = float(((int16_t)buff[19] << 8) | buff[18]);
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

	//data.axis[3] -= 60.0f;
	//data.axis[4] -= 65500.0f;
	//data.axis[5] -= 80.0f;

	data.axis[0] *= _accelScale;
	data.axis[1] *= _accelScale;
	data.axis[2] *= _accelScale;

	data.axis[3] *= _gyroScale;
	data.axis[4] *= _gyroScale;
	data.axis[5] *= _gyroScale;

	data.axis[6] *= _magScaleX;
	data.axis[7] *= _magScaleY;
	data.axis[8] *= _magScaleZ;

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
	///*
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
	//*/
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
	else {
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