
#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "Settings.h"
#include "Gyro.h"

Settings config;

void setupConfig()
{
  config.min_esc_pulse = 1000;
  config.max_esc_pulse = 2000;
  config.esc_pulse_margin = 200;

  config.min_servo_pulse = 1100;
  config.max_servo_pulse = 1850;
    
  config.gyro_filter = Gyro::FILTER_98HZ;//2;
  config.gyro_fs_range = Gyro::FSR_250DPS;//1;
  config.gyro_cal_count = 1000;
  
  config.gyro_pitch_order = 1;
  config.gyro_roll_order = 2;
  config.gyro_yaw_order = 3;
  
  config.gyro_invert_pitch = -1;
  config.gyro_invert_roll = -1;
  config.gyro_invert_yaw = -1;
  
  config.pid_p_gain = 5.0;
  config.pid_i_gain = 0.1;
  config.pid_d_gain = 10.0;
  config.pid_max = 400;

  config.loop_freq = 250;

  config.nrf24_ack_payload = false;
  config.nrf24_ce_pin = 9;
  config.nrf24_csn_pin = 10;
  
  config.rc_invert_pitch = -1;
  config.rc_invert_roll = 1;
  config.rc_invert_yaw = 1;
  
  config.rc_pitch_rate = 1.5;
  config.rc_roll_rate = 1.5;
  config.rc_yaw_rate = 1.5;
}

#endif
