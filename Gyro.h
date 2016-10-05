
#ifndef __GYRO_H__
#define __GYRO_H__

#include "Settings.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Gryo Config
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// *          |   ACCELEROMETER    |           GYROSCOPE
// * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
// * ---------+-----------+--------+-----------+--------+-------------
// * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
// * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
// * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
// * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
// * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
// * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
// * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
// * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// * 0 = +/- 250 degrees/sec
// * 1 = +/- 500 degrees/sec
// * 2 = +/- 1000 degrees/sec
// * 3 = +/- 2000 degrees/sec
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class Gyro
{
public:

  enum low_pass_filter_reg : uint8_t
  {
    FILTER_256HZ_NOLPF2 = 0,
    FILTER_188HZ,
    FILTER_98HZ,
    FILTER_42HZ,
    FILTER_20HZ,
    FILTER_10HZ,
    FILTER_5HZ,
    FILTER_2100HZ_NOLPF,
    FILTER
  };
    
  enum full_scale_range_reg : uint8_t
  {
    FSR_250DPS = 0,
    FSR_500DPS,
    FSR_1000DPS,
    FSR_2000DPS,
    NUM_GYRO_FSR
  };
   
  uint8_t lpf;
  uint8_t fsr;

  const float fsr_div[4] = {131.0, 65.5, 32.8, 16.4};

  void setup(Settings& config)
  {
    Wire.begin(); //Start the I2C as master.

    delay(500);

    lpf = config.gyro_filter;
    fsr = config.gyro_fs_range;
    
    i2c_write_reg(0x68, 0x6B, 0x80);
    delay(100);
    
    i2c_write_reg(0x68, 0x6B, 0x03);
    delay(20);
    
    i2c_write_reg(0x68, 0x1A, lpf);
    i2c_write_reg(0x68, 0x1B, fsr << 3);

    pitch_out = &axis[config.gyro_pitch_order - 1];
    roll_out = &axis[config.gyro_roll_order - 1];
    yaw_out = &axis[config.gyro_yaw_order - 1];

    invert[config.gyro_pitch_order - 1] = config.gyro_invert_pitch;
    invert[config.gyro_roll_order - 1] = config.gyro_invert_roll;
    invert[config.gyro_yaw_order - 1] = config.gyro_invert_yaw;
  }

  void update()
  {
    read_raw();
    apply_calibration();
    apply_inversion_and_scale();
  }

  inline void read_raw()
  {   
    //sei();
    
    Wire.beginTransmission(0x68);  //Start communication with the gyro
    Wire.write(0x43);              //Start reading and auto increment with every read
    Wire.endTransmission();           //End the transmission
    Wire.requestFrom(0x68, 6u);     //Request 6 bytes from the gyro
  
    while (Wire.available() < 6);     //Wait until the 6 bytes are received
    
    axis[0] = read_int();
    axis[1] = read_int();
    axis[2] = read_int();

    //cli();
  }

  inline int16_t read_int() 
  {
    uint8_t highByte = Wire.read();
    uint8_t lowByte = Wire.read();
  
    return (highByte << 8) | lowByte;
  }
  
  inline void apply_calibration() 
  {
    axis[0] -= axis_calibration[0];
    axis[1] -= axis_calibration[1];
    axis[2] -= axis_calibration[2];
  }
  
  inline void apply_inversion_and_scale() 
  {
    float lsb_scale = fsr_div[fsr]; // 57.29577
    
    axis[0] *= invert[0] / lsb_scale;
    axis[1] *= invert[1] / lsb_scale;
    axis[2] *= invert[2] / lsb_scale;
  }

  void calibrate(Settings& config)
  {
    for (int i = 0; i < config.gyro_cal_count ; i++) 
    {
      read_raw();
      
      axis_calibration[0] += axis[0];
      axis_calibration[1] += axis[1];
      axis_calibration[2] += axis[2];
      
      delayMicroseconds(500);
    }

    axis_calibration[0] /= config.gyro_cal_count;
    axis_calibration[1] /= config.gyro_cal_count;
    axis_calibration[2] /= config.gyro_cal_count;
  }

  inline float getPitch()
  {
    return *pitch_out;
  }
   
  inline float getRoll()
  {
    return *roll_out;
  }

  inline float getYaw()
  {
    return *yaw_out;
  }  

private:

  float* pitch_out;
  float* roll_out;
  float* yaw_out;
  
  float axis[3];
  float axis_calibration[3];
  int8_t invert[3];

  void i2c_write_reg(int address, uint8_t  reg, uint8_t  val) 
  {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
  }  
};

#endif
