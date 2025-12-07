#ifndef PID_H
#define PID_H

#include "DCEnc.h"
#include "InputLeft.h"
#include "InputRight.h"
#include "config.h"

// PID for Left Motor
class PIDLeft {
private:
    InputLeft iL;
    DCEnc dcEL;
    float Kp;
    float Ki;
    float Kd;
    float error;
    float error_diff;
    float error_prev;
    float error_sum;
    int u;
    float target_rpm;
    float current_rpm;

    void calc_error();
    void calc_error_sum();
    void calc_error_diff();
    void debug();

public:
    PIDLeft();
    void calculate_output();
};

// PID for Right Motor
class PIDRight {
private:
    InputRight iR;
    DCEnc dcER;
    float Kp;
    float Ki;
    float Kd;
    float error;
    float error_diff;
    float error_prev;
    float error_sum;
    int u;
    float target_rpm;
    float current_rpm;

    void calc_error();
    void calc_error_sum();
    void calc_error_diff();
    void debug();

public:
    PIDRight();
    void calculate_output();
};

#endif
