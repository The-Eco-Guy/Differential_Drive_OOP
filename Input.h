#ifndef INPUT_H
#define INPUT_H

#include <Arduino.h>
#include "Constants.h"

class Input {
  private:
    const int v_pin = A0; // Velocity pot pin
    const int d_pin = A1; // Direction pot pin
    float omega;
    float desired_rpm;
    float dir; // Direction in radians/second
    float vel; // Velocity in meters/second
    
    // Multiplier to distinguish Left (-1) vs Right (1) calculations
    int side_multiplier; 

    // Helper function for float mapping
    float map_float(float x, float in_min, float in_max, float out_min, float out_max);
    
    void map_dir();
    void map_vel();
    void calculate_omega();

  public:
    Input() {}; // Default constructor
    
    // Constructor accepts side (-1 for Left, 1 for Right)
    Input(int side);
    
    // Initialization method if using default constructor (optional)
    void init(int side);

    float calc_desired_rpm();
};

#endif
