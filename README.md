# Differential Drive Robot - OOP Arduino Implementation

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![Arduino](https://img.shields.io/badge/Arduino-Uno-blue)
![Language](https://img.shields.io/badge/Language-C%2B%2B-brightgreen)

A modular, object-oriented Arduino implementation of a **PID-based differential-drive robot controller** with independent left and right motor control using encoder feedback.

## 🎯 Overview

This project demonstrates professional embedded systems development with clean architecture, separating concerns across multiple classes. Two Arduino Uno microcontrollers independently control left and right motors using closed-loop PID control with encoder feedback for accurate velocity tracking.

**Perfect for:** Robotics enthusiasts, embedded systems learners, and anyone building a differential-drive robot platform.

---

## ⚡ Quick Start

### Hardware Requirements
- 2× Arduino Uno microcontrollers
- 2× DC motors with encoders (160 PPR)
- 2× Motor drivers (L298N or similar) with PWM capability
- 2× Potentiometers (velocity and direction input)
- Shared power supply



## 📁 Project Structure

```
differential-drive-robot/
├── firmware/                      # Main Arduino code
│   ├── left_motor/               # Left motor controller (complete project)
│   │   ├── DifferentialDrive_Left.ino
│   │   ├── config.h              # Shared configuration
│   │   ├── DCEnc.h / DCEnc.cpp  # Encoder reading (shared)
│   │   ├── InputLeft.h / .cpp   # Left motor input processing
│   │   └── PID.h / PID.cpp      # PID controller (shared structure)
│   │
│   └── right_motor/              # Right motor controller (complete project)
│       ├── DifferentialDrive_Right.ino
│       ├── config.h
│       ├── DCEnc.h / DCEnc.cpp
│       ├── InputRight.h / .cpp
│       └── PID.h / PID.cpp

```

---



### Class Hierarchy

```
firmware/
│
├── DCEnc (Encoder Reading)
│   ├── isUpdated()           → Check sampling interval
│   └── calc_actual_rpm()     → Calculate motor RPM
│
├── InputLeft / InputRight (Motor Input Processing)
│   └── calc_desired_rpm()    → Convert pot inputs to desired RPM
│
└── PIDLeft / PIDRight (Motor Control)
    └── calculate_output()    → Apply PID, drive motor
```

### Control Flow

```
1. Read Potentiometers (A0, A1)
   ↓
2. Calculate Desired RPM (InputLeft/InputRight)
   ↓
3. Read Encoder (Pins 2, 3)
   ↓
4. Calculate Actual RPM (DCEnc)
   ↓
5. Calculate Error (desired - actual)
   ↓
6. Apply PID Controller
   ↓
7. Drive Motor (PWM on Pin 5)
```

### Key Differences: Left vs Right Motor

**InputLeft omega calculation:**
```cpp
omega = (vel - (axl * dir) / 2) / r;  // Subtract direction component
```

**InputRight omega calculation:**
```cpp
omega = (vel + (axl * dir) / 2) / r;  // Add direction component
```

This implements differential drive kinematics where the two motors calculate velocities based on desired linear velocity and angular velocity.

---

## 🔧 Configuration

All parameters are centralized in `config.h`:

### Pin Configuration
```cpp
#define EN 5          // Motor enable (PWM)
#define IN4 6         // Motor input 4
#define IN3 7         // Motor input 3
#define ENC_PIN_A 2   // Encoder A
#define ENC_PIN_B 3   // Encoder B
#define V_PIN A0      // Velocity potentiometer
#define D_PIN A1      // Direction potentiometer
```

### Physical Parameters
```cpp
#define WHEEL_RADIUS 0.05        // meters (5 cm)
#define AXLE_LENGTH 0.2          // meters (20 cm)
#define PULSES_PER_ROTATION 160  // Encoder PPR
#define MAX_VELOCITY 0.1665      // m/s
```

### PID Tuning
```cpp
#define KP 2.0    // Proportional gain
#define KI 0.0    // Integral gain (disabled)
#define KD 0.0    // Derivative gain (disabled)
```

### Control Parameters
```cpp
#define SAMPLING_INTERVAL 100    // milliseconds
```

**To modify:** Edit `config.h` and re-upload to both Arduinos.

---

## 📊 Motor Physics

### Conversion Factors
```
RPM → rad/s:     ω = RPM × π/30
rad/s → m/s:     v = ω × radius
m/s to RPM ratio: 9.5493 (derived from above)
```

### Differential Drive Kinematics
For a robot with two independently controlled wheels:

```
v_left  = (v_linear) - (L/2) × ω_angular
v_right = (v_linear) + (L/2) × ω_angular

where:
  L = axle length (wheel-to-wheel distance)
  v_linear = desired linear velocity
  ω_angular = desired angular velocity
```

### Parameter Reference

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| Wheel Radius (r) | 0.05 | m | Half wheel diameter |
| Axle Length (L) | 0.2 | m | Distance between wheel centers |
| Encoder PPR | 160 | pulses | Pulses per motor rotation |
| Max Velocity | 0.1665 | m/s | Maximum wheel linear velocity |
| Sampling Rate | 100 | ms | Control loop update frequency |

---

## 📈 Performance Characteristics

### Sampling & Control
- **Control Loop Rate:** 100ms (10 Hz) - adjustable in `config.h`
- **Encoder Precision:** ±1 pulse per update interval
- **Motor Update Rate:** 100ms minimum (synchronous with encoder)
- **Baud Rate:** 9600 bps for serial debugging

### PID Response
**Current Configuration:** `Kp=2.0, Ki=0.0, Kd=0.0`
- ✅ Fast proportional response
- ⚠️ May oscillate without damping


---

## 🎮 Control Input

### Velocity Potentiometer (A0)
- **Range:** 0-1023 ADC units
- **Maps to:** 0 → 0.1665 m/s
- **Use:** Speed control (0 = stop, max = full speed)

### Direction Potentiometer (A1)
- **Range:** 0-1023 ADC units
- **Maps to:** ±1.57 rad/s (±90°)
- **Use:** Turn control (center = straight, left/right = turn)



---

## 🐛 Serial Debugging

### Output Format
Each 100ms, both Arduinos output:
```
Target: 50.25, Actual: 49.80
Output: 200
```

**Interpretation:**
- **Target:** Desired RPM calculated from potentiometers
- **Actual:** Measured RPM from encoder
- **Output:** PWM value (0-255) sent to motor driver

### Monitor in Arduino IDE
```
Tools → Serial Monitor → 9600 baud
```

### Troubleshooting via Serial
- **All zeros?** Check potentiometer connections
- **Actual always lower than target?** Motor not strong enough or load too high
- **Oscillating rapidly?** Kp too high, add Kd damping
- **No response?** Check motor direction (IN3/IN4 polarity)

---


## 🎯 Key Features

✅ **Modular Architecture**
- Clean separation of concerns with dedicated classes
- Reusable DCEnc and PID implementations
- Easy to modify and extend

✅ **Independent Motor Control**
- Two separate Arduino boards for parallel processing
- No inter-Arduino communication overhead
- Scalable to more motors with additional Arduinos

✅ **Closed-Loop Velocity Control**
- Encoder feedback for accurate velocity tracking
- PID controller for responsive motor control
- Serial debugging output for monitoring

✅ **Comprehensive Documentation**
- Hardware setup guides with circuit diagrams
- PID tuning methodology and examples
- Troubleshooting guide for common issues

✅ **Professional Code Organization**
- Clear file structure
- Centralized configuration management
- Object-oriented C++ implementation
- No global state or hardcoded values

---

## 📋 Use Cases

🤖 **Robotics Projects**
- Line-following robots
- Obstacle avoidance platforms
- Autonomous navigation systems
- Educational robotics kits

🎓 **Learning & Development**
- Embedded systems education
- Control systems theory practical implementation
- Arduino advanced programming techniques
- Differential drive kinematics study

🔧 **Prototyping**
- Mobile robot platform
- Wheeled robot base
- Autonomous vehicle prototype
- Motor control research

---





## 📝 Hardware Compatibility

### Tested On
- Arduino Uno (ATmega328P)
- DC motors with mechanical encoders
- L298N motor driver module
- Generic potentiometers (10kΩ)

### Compatibility Notes
- **Arduino Mega:** Requires pin definition changes in `config.h`
- **Arduino Nano:** Requires pin definition changes in `config.h`
- **Other Motor Drivers:** Update motor control pins in `config.h`
- **Different Encoders:** Adjust `PULSES_PER_ROTATION` in `config.h`

---

## 🔐 License

This project is licensed under the **MIT License** - see [LICENSE](LICENSE) file for details.

---

## 👤 Author

**Created:** December 7, 2025
**Institution:** MIT Manipal - Bachelor of Technology in Computer Science & Engineering
**Specialization:** Networks and Internet of Things

---


## 📖 Related Resources

- [Arduino Official Documentation](https://www.arduino.cc/reference/)
- [Paul Stoffregen's Encoder Library](https://github.com/PaulStoffregen/Encoder)
- [PID Control Theory](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller)
- [Differential Drive Kinematics](https://en.wikipedia.org/wiki/Differential_wheeled_robot)

---

## ⭐ Acknowledgments

This project demonstrates:
- Clean object-oriented design patterns
- Professional embedded systems architecture
- Educational robotics implementation
- Open-source best practices

---

**Made with ❤️ for robotics enthusiasts and embedded systems learners**

---


