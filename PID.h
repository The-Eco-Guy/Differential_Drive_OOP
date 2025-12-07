#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include "Input.h"
#include "DCEnc.h"
#include "Constants.h"

class PID {
  private:
    Input input; // Generic Input object
    DCEnc dcEnc; // Generic Encoder object
    
    float Kp;
    float Ki;
    float Kd;
    
    float error;
    float error_diff;
    float error_prev = 0;
    float error_sum = 0;
    int u;

    float target_rpm = 0;
    float current_rpm = 0;

    void calc_error();
    void calc_error_sum();
    void calc_error_diff();
    void debug();

  public:
    // Constructor accepts side and specific PID constants
    PID(int side, float kp, float ki, float kd);
    
    void calculate_output();
};

#endif
