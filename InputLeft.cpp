#include "InputLeft.h"

// Helper function for floating-point mapping
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

InputLeft::InputLeft() : omega(0), desired_rpm(0), dir(0), vel(0) {}

void InputLeft::map_dir() {
    dir = map_float(analogRead(D_PIN), 0, 1023, 1.57, -1.57);
}

void InputLeft::map_vel() {
    vel = (analogRead(V_PIN) / 1023.0) * MAX_VELOCITY;
}

void InputLeft::calculate_omega() {
    map_dir();
    map_vel();
    omega = (vel - (AXLE_LENGTH * dir) / 2) / WHEEL_RADIUS;
}

float InputLeft::calc_desired_rpm() {
    calculate_omega();
    desired_rpm = omega * 9.5493;
    return desired_rpm;
}
