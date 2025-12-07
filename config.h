#ifndef CONFIG_H
#define CONFIG_H

// Motor Control Pins
#define EN 5
#define IN4 6
#define IN3 7

// Encoder Pins
#define ENC_PIN_A 2
#define ENC_PIN_B 3

// Analog Input Pins
#define V_PIN A0  // Velocity potentiometer
#define D_PIN A1  // Direction potentiometer

// Physical Parameters
#define WHEEL_RADIUS 0.05          // meters
#define AXLE_LENGTH 0.2             // meters
#define PULSES_PER_ROTATION 160     // PPR of encoder
#define MIN_VELOCITY 0              // m/s
#define MAX_VELOCITY 0.1665         // m/s

// PID Parameters
#define KP 2.0
#define KI 0.0
#define KD 0.0

// Sampling Interval
#define SAMPLING_INTERVAL 100       // milliseconds

#endif
