
#ifndef __SETTINGS_H__
#define __SETTINGS_H__

struct Settings
{
  float pid_p_gain;
  float pid_i_gain;
  float pid_d_gain;
  int16_t pid_max;
  
  uint16_t loop_freq;
  
  uint8_t nrf24_ce_pin;
  uint8_t nrf24_csn_pin;
  bool nrf24_ack_payload;
  
  int8_t rc_invert_pitch;
  int8_t rc_invert_roll;
  int8_t rc_invert_yaw;
  
  float rc_pitch_rate;
  float rc_roll_rate;
  float rc_yaw_rate;
  
  uint16_t min_esc_pulse;
  uint16_t max_esc_pulse;
  uint16_t esc_pulse_margin;

  uint16_t min_servo_pulse;
  uint16_t max_servo_pulse;
  
  uint8_t gyro_filter;
  uint8_t gyro_fs_range;
  uint16_t gyro_cal_count;
  
  uint8_t gyro_pitch_order;
  uint8_t gyro_roll_order;
  uint8_t gyro_yaw_order;
  
  int8_t gyro_invert_pitch;
  int8_t gyro_invert_roll;
  int8_t gyro_invert_yaw;
};

#endif
