
#ifndef __RADIO_H__
#define __RADIO_H__

#include "Settings.h"

#include <RF24.h>
#include <nRF24L01.h>

class Radio
{
public:

  struct Data
  {
    int16_t throttle;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
  };
  
  struct AckPayload 
  {
    uint8_t armed_status;
    float voltage;
    int16_t gyro_pitch;
    int16_t gyro_roll;
    int16_t gyro_yaw;
  };

  void resetData() 
  {
    data.throttle = 1000;
    data.yaw = 1500;
    data.pitch = 1500;
    data.roll = 1500;
  }

  void resetAckPayload() 
  {
    ack_payload.armed_status = 0;
    ack_payload.voltage = 0;
    ack_payload.gyro_pitch = 0;
    ack_payload.gyro_roll = 0;
    ack_payload.gyro_yaw = 0;
  }
  
  ~Radio()
  {
    if (nrf24)
    {
      delete nrf24;
    }
  }
  
  void setup(Settings& config)
  {
    time_out = config.loop_freq;
    use_ack_payload = config.nrf24_ack_payload;
    loops_since_last_recv = 0;
    rx_signal = false;

    resetData();
    
    nrf24 = new RF24(config.nrf24_ce_pin, config.nrf24_csn_pin);
    
    nrf24->begin();
    nrf24->setDataRate(RF24_250KBPS);
    nrf24->setAutoAck(use_ack_payload);

    if (use_ack_payload)
    {
      nrf24->enableAckPayload();
      resetAckPayload(); 
    }
    
    nrf24->openReadingPipe(1,pipe);
    nrf24->startListening();
  }

  bool update()
  {
    loops_since_last_recv++;
  
    while (nrf24->available()) 
    {
      if (use_ack_payload)
      {
        nrf24->writeAckPayload(1, &ack_payload, sizeof(AckPayload));
      }
      
      nrf24->read(&data, sizeof(Data));
      
      loops_since_last_recv = 0;
      rx_signal = true;
    }
  
    // Usually at most a single packet is received each time, in which case this function takes 220us.
    // If no packet is received this function takes around 31us, so we'll wait 220 - 31 us to keep the time constant.
    if (loops_since_last_recv != 0) 
    {
      delayMicroseconds(189); // 220 - 31
    }
  
    if ( loops_since_last_recv > time_out ) 
    {
      // signal lost?
      resetData();
      rx_signal = false;
    }

    return rx_signal;
  }

  void setAckPayload(AckPayload payload)
  {
    ack_payload = payload;
  }
  
  Data& getData()
  {
    return data;
  }

private:

  // Single radio pipe address for the 2 nodes to communicate.
  static const uint64_t pipe = 0xE8E8F0F0E1LL;

  Data data;
  AckPayload ack_payload;

  bool use_ack_payload;
  bool rx_signal;
  uint32_t loops_since_last_recv;
  uint16_t time_out;
 
  RF24* nrf24;
};

#endif
