#pragma once

#include "Arduino.h"
#include "Filter.h"

//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L	 0x03  // data
#define AK8963_XOUT_H	 0x04
#define AK8963_YOUT_L	 0x05
#define AK8963_YOUT_H	 0x06
#define AK8963_ZOUT_L	 0x07
#define AK8963_ZOUT_H	 0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL1     0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_CNTL2     0x0B  // Soft Reset
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00                  
#define SELF_TEST_Y_GYRO 0x01                                                                          
#define SELF_TEST_Z_GYRO 0x02

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E    
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E   
#define WOM_THR          0x1F   

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL   0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

enum mpu9250_gyro_range
{
	GYRO_RANGE_250DPS,
	GYRO_RANGE_500DPS,
	GYRO_RANGE_1000DPS,
	GYRO_RANGE_2000DPS
};

enum mpu9250_accel_range
{
	ACCEL_RANGE_2G,
	ACCEL_RANGE_4G,
	ACCEL_RANGE_8G,
	ACCEL_RANGE_16G
};

enum mpu9250_dlpf_bandwidth
{
	DLPF_BANDWIDTH_250HZ,
	DLPF_BANDWIDTH_184HZ,
	DLPF_BANDWIDTH_92HZ,
	DLPF_BANDWIDTH_41HZ,
	DLPF_BANDWIDTH_20HZ,
	DLPF_BANDWIDTH_10HZ,
	DLPF_BANDWIDTH_5HZ,
	DLPF_BANDWIDTH_3600HZ
};

class IMU 
{
public:

	IMU(uint8_t csPin);

	void begin();

	void calibrate();

	void update()
	{
		read_raw();
		apply_calibration();
		apply_inversion_and_scale();
		apply_filters();
		update_fusion();
		calculate_angles();
	}

	union sensor_data
	{
		struct
		{
			float ax;
			float ay;
			float az;

			float gx;
			float gy;
			float gz;

			float mx;
			float my;
			float mz;

			float roll;
			float pitch;
			float yaw;
		};

		float axis[12];
	};


	const sensor_data& get_data() const
	{
		return data;
	}

private:

	void read_raw();
	void apply_calibration();
	void apply_inversion_and_scale();
	void apply_filters();
	void update_fusion();
	void calculate_angles();

	const float twoKp = 10.0f;		// 2 * proportional gain (Kp)
	const float twoKi = 0.2f;		// 2 * integral gain (Ki)
	
	float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
	float integralFBx, integralFBy, integralFBz;  // integral error terms scaled by Ki
	float invSampleFreq;

	pt1Filter_t ptfilters[6];
	const float gyro_cut_off = 15.0f;//8.0f;
	const float accel_cut_off = 5.0f;//5.0f;

	sensor_data data;
	
	float gyroBias[3] = { 0.15f, -0.21f, 1.27f };
	float accelBias[3] = { 0, 0, 0 };

	const float sample_rate = 1000.0f;
	const mpu9250_accel_range accelRange = ACCEL_RANGE_4G;
	const mpu9250_gyro_range gyroRange = GYRO_RANGE_250DPS;
	const mpu9250_dlpf_bandwidth dlpf = DLPF_BANDWIDTH_250HZ;

	uint8_t _csPin;
	bool _useSPIHS;
	float _accelScale;
	float _gyroScale;
	float _magScaleX, _magScaleY, _magScaleZ;
	const float _tempScale = 333.87f;
	const float _tempOffset = 21.0f;

	// SPI constants
	const uint8_t SPI_READ = 0x80;
	const uint32_t SPI_LS_CLOCK = 1000000; // 1 MHz
	const uint32_t SPI_HS_CLOCK = 20000000; // 20 MHz

	// constants
	const uint8_t INT_DISABLE = 0x00;
	const uint8_t INT_PULSE_50US = 0x00;
	const uint8_t INT_RAW_RDY_EN = 0x01;

	const uint8_t PWR_MGMNT_1 = 0x6B;
	const uint8_t PWR_RESET = 0x80;
	const uint8_t CLOCK_SEL_PLL = 0x01;

	const uint8_t PWR_MGMNT_2 = 0x6C;
	const uint8_t SEN_ENABLE = 0x00;

	const uint8_t I2C_MST_EN = 0x20;
	const uint8_t I2C_MST_CLK = 0x0D;

	const uint8_t I2C_SLV0_EN = 0x80;
	const uint8_t I2C_READ_FLAG = 0x80;

	const uint8_t WHO_AM_I = 0x75;

	// AK8963 registers
	const uint8_t AK8963_I2C_ADDR = 0x0C;
	const uint8_t AK8963_HXL = 0x03;
	const uint8_t AK8963_PWR_DOWN = 0x00;
	const uint8_t AK8963_CNT_MEAS1 = 0x12;
	const uint8_t AK8963_CNT_MEAS2 = 0x16;
	const uint8_t AK8963_FUSE_ROM = 0x0F;
	const uint8_t AK8963_RESET = 0x01;
	const uint8_t AK8963_ASA = 0x10;

	bool writeRegister(uint8_t subAddress, uint8_t data);
	void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
	bool writeAK8963Register(uint8_t subAddress, uint8_t data);
	void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest);
	uint8_t whoAmI();
	uint8_t whoAmIAK8963();
};