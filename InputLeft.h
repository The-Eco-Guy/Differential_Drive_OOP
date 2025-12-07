#ifndef INPUT_LEFT_H
#define INPUT_LEFT_H

#include "config.h"

class InputLeft {
private:
    float omega;
    float desired_rpm;
    float dir;
    float vel;

    void map_dir();
    void map_vel();
    void calculate_omega();

public:
    InputLeft();
    float calc_desired_rpm();
};

#endif
