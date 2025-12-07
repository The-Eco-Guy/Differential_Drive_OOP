#include "Input.h"

Input::Input(int side) {
  this->side_multiplier = side;
}

void Input::init(int side) {
  this->side_multiplier = side;
}

float Input::map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Input::map_dir() {
  dir = map_float(analogRead(d_pin), 0, 1023, 1.57, -1.57);
}

void Input::map_vel() {
  vel = (analogRead(v_pin) / 1023.0) * MAX_VEL;
}

void Input::calculate_omega() {
  map_dir();
  map_vel();
  
  // Unified formula:
  // Left (side -1): (vel + (-1 * ...)) -> (vel - ...)
  // Right (side 1): (vel + (1 * ...))  -> (vel + ...)
  omega = (vel + (side_multiplier * (AXL * dir) / 2)) / R;
}

float Input::calc_desired_rpm() {
  calculate_omega();
  desired_rpm = omega * 9.5493;
  return desired_rpm;
}
