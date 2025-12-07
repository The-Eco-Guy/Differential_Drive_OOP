#ifndef INPUT_RIGHT_H
#define INPUT_RIGHT_H

#include "config.h"

class InputRight {
private:
    float omega;
    float desired_rpm;
    float dir;
    float vel;

    void map_dir();
    void map_vel();
    void calculate_omega();

public:
    InputRight();
    float calc_desired_rpm();
};

#endif
