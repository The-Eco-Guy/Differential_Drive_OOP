// Differential Drive Robot - RIGHT MOTOR CONTROLLER
// Arduino-based PID controller for right motor with encoder feedback
// Date: 2025-12-07

#include "config.h"
#include "DCEnc.h"
#include "InputRight.h"
#include "PID.h"

PIDRight rightMotorPID;

void setup() {
    Serial.begin(9600);
    
    // Initialize motor control pins
    pinMode(EN, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    // Initialize motor state (stopped)
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(EN, 0);
}

void loop() {
    rightMotorPID.calculate_output();
}
