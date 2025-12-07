# Differential-Drive Robot Motor Control

A modular Arduino-based motor control system for differential-drive robots using **PID control** and **encoder feedback** for precise velocity regulation.

## Overview

This project implements independent motor control for a two-wheeled differential-drive robot using two Arduino Uno microcontrollers (one per motor). Each Arduino runs a **PID feedback loop** to track desired velocities from analog input controls while maintaining accurate motor speed via encoder feedback.

### Key Features

- **Dual-Arduino Architecture**: Separate controllers for left and right motors enable independent control
- **PID Motor Control**: Real-time feedback control adjusts motor speed based on encoder readings
- **Analog Input Joystick**: Dual potentiometers control forward velocity and steering direction
- **Encoder Feedback**: Quadrature encoder integration for precise RPM tracking
- **Modular Design**: Clean object-oriented C++ with reusable classes for easy integration into larger projects
- **Configurable Constants**: All physical parameters and pin mappings centralized in a single header file

## Project Structure

```
├── DifferentialDrive.ino    # Main sketch - upload to both Arduinos
├── PID.h                     # PID controller class definition
├── PID.cpp                   # PID implementation with motor output
├── Input.h                   # Analog input processing class
├── Input.cpp                 # Joystick mapping and omega calculation
├── DCEnc.h                   # Encoder reading class definition
├── DCEnc.cpp                 # Encoder RPM calculation
└── Constants.h               # Shared physical parameters and pin definitions
```

## Hardware Requirements

### Microcontroller
- 2x Arduino Uno (or compatible)

### Motors & Sensors
- 2x DC Motors with encoders (160 PPR - pulses per rotation)
- 2x Quadrature Encoders (pins 2-3 for each motor)

### Control Electronics
- 2x Motor Driver Module (L298N or similar)
  - EN pin: Arduino pin 5 (PWM)
  - IN3 pin: Arduino pin 7 (direction control)
  - IN4 pin: Arduino pin 6 (direction control)

### Input Devices
- 2x Potentiometers (10kΩ recommended)
  - Velocity control: Analog pin A0
  - Direction control: Analog pin A1

## Pin Configuration

| Purpose | Arduino Pin | Notes |
|---------|-------------|-------|
| Motor PWM (Enable) | 5 | PWM output for speed control |
| Motor Direction 1 | 6 | Digital output |
| Motor Direction 2 | 7 | Digital output |
| Encoder A | 2 | Quadrature input |
| Encoder B | 3 | Quadrature input |
| Velocity Input | A0 | Analog potentiometer |
| Direction Input | A1 | Analog potentiometer |

## Installation & Setup

### Step 1: Hardware Assembly
1. Connect DC motors with encoders to the motor driver module
2. Wire the motor driver to Arduino pins 5, 6, and 7
3. Connect quadrature encoders to digital pins 2 and 3
4. Connect potentiometers to analog pins A0 and A1
5. Upload this code to **both Arduino Unos**

### Step 2: Configuration
Edit `DifferentialDrive.ino` to set which motor this Arduino controls:

```cpp
// Set this to true for Left Motor code, false for Right Motor code
bool isLeftMotor = true;
```

### Step 3: Tuning PID Gains
Motor-specific PID constants are defined in `DifferentialDrive.ino`:

```cpp
if (isLeftMotor) {
    // Left Motor: Kp=2.0, Ki=0.001, Kd=0.001
    motorPID = new PID(-1, 2.0, 0.001, 0.001);
} else {
    // Right Motor: Kp=2.0, Ki=0.0, Kd=0.0
    motorPID = new PID(1, 2.0, 0.0, 0.0);
}
```

Adjust these values empirically based on your robot's motor characteristics and desired response.

## Class Architecture

### `PID` - Motor Control
Implements the main PID feedback loop:
- Reads desired RPM from joystick input
- Compares with actual RPM from encoder feedback
- Outputs corrected PWM signal to motor driver
- Runs at 100ms intervals for stability

**Public Methods:**
- `PID(int side, float kp, float ki, float kd)` - Constructor
- `void calculate_output()` - Main control loop (call in Arduino `loop()`)

### `Input` - Joystick Processing
Converts potentiometer readings to motor commands:
- Maps velocity potentiometer (0-1023) to velocity range
- Maps direction potentiometer (0-1023) to steering angle
- Calculates left/right motor angular velocities based on kinematics
- Supports both left (-1) and right (1) motor configurations

**Public Methods:**
- `Input(int side)` - Constructor
- `float calc_desired_rpm()` - Returns desired RPM for this motor

### `DCEnc` - Encoder Feedback
Measures actual motor speed:
- Reads quadrature encoder on pins 2-3
- Calculates RPM at 100ms intervals
- Handles encoder pulse counting and timing

**Public Methods:**
- `DCEnc()` - Constructor
- `float calc_actual_rpm()` - Returns measured RPM
- `bool isUpdated()` - Checks if 100ms interval has passed

## Tuning Guide

### Initial Calibration
1. Set all PID gains to 0: `PID(side, 0, 0, 0)`
2. Gradually increase Kp until the motor responds to desired velocity
3. Add Ki if there's steady-state error at constant velocities
4. Add Kd if there's oscillation around the setpoint

### Tips
- Start conservative with gains; too-high Kp causes instability
- Left and right motors may need different tuning
- Monitor serial output (`Target:`, `Actual:`, `Output:`) for feedback
- The system samples encoder feedback every 100ms; adjust gains accordingly

## Usage Example

Once uploaded and configured, the robot will:
1. Read velocity input from A0 (max 0.1665 m/s)
2. Read steering input from A1 (±1.57 radians/second)
3. Calculate left/right motor target speeds using differential kinematics
4. Apply PID control to track targets despite load variations
5. Print debug data to Serial (9600 baud) for monitoring

## Physical Parameters

All robot dimensions are defined in `Constants.h`:

| Parameter | Value | Unit |
|-----------|-------|------|
| Wheel Radius (R) | 0.05 | meters |
| Axle Length (AXL) | 0.2 | meters |
| Encoder PPR | 160 | pulses/rotation |
| Max Velocity | 0.1665 | m/s |

Modify these values to match your robot's specifications.

## Debugging

### Serial Monitor Output
Connect to the Arduino at 9600 baud to see real-time feedback:
```
Target:50.25,Actual:48.90
Output:210
```

This shows:
- **Target**: Desired RPM calculated from joystick input
- **Actual**: Measured RPM from encoder feedback
- **Output**: PWM value (0-255) sent to motor driver

### Common Issues
- **Motor not responding**: Check pin connections and motor driver power
- **Encoder not reading**: Verify encoder wiring to pins 2-3
- **Oscillating speed**: Reduce Kp or increase Kd
- **Slow response**: Increase Kp or decrease Ki damping

## License

This project is provided as-is for educational and personal use.

## Future Improvements

- Implement simultaneous two-motor synchronization
- Add wireless control (Bluetooth/RF)
- Integrate IMU for heading hold
- Path planning and trajectory tracking
- ROS integration for autonomous navigation

---

**Happy roboticizing!** 🤖
