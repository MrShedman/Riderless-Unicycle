
#ifndef __PID_H__
#define __PID_H__

class PID
{
  public:

  void setup(float kp, float ki, float kd, int16_t max_i, int16_t max_output)
  {
    p_gain = kp;
    i_gain = ki;
    d_gain = kd;
    this->max_i = max_i;
    this->max_output = max_output;
        
    reset();
  }
  
  void update(float input, float setpoint)
  {
    const float error_temp = input - setpoint;
    
    i_mem += i_gain * error_temp;
    i_mem = constrain(i_mem, -max_i, max_i);
    
    output = p_gain * error_temp + i_mem + d_gain * (error_temp - last_d_error);
    output = constrain(output, -max_output, max_output);

    last_d_error = error_temp;
  }

  void reset()
  {
    i_mem = 0;
    last_d_error = 0;
  }

  float getOutput() const
  {
    return output;
  }
  
  private:
  
  float p_gain;                //Gain setting for the pitch P-controller. //4.0
  float i_gain;                //Gain setting for the pitch I-controller. //0.02
  float d_gain;                //Gain setting for the pitch D-controller.
  
  int16_t max_output;          //Maximum output of the PID-controller (+/-)
  int16_t max_i;               //Maximum value of I term of the PID-controller (+/-)  
  
  float i_mem;
  float output;
  float last_d_error;
};

#endif
