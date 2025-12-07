#include "PID.h"

// ==================== PIDLeft Implementation ====================

PIDLeft::PIDLeft() 
    : iL(), dcEL(), Kp(KP), Ki(KI), Kd(KD), 
      error(0), error_diff(0), error_prev(0), error_sum(0), 
      u(0), target_rpm(0), current_rpm(0) {}

void PIDLeft::calc_error() {
    target_rpm = iL.calc_desired_rpm();
    current_rpm = dcEL.calc_actual_rpm();
    error = target_rpm - current_rpm;
}

void PIDLeft::calc_error_sum() {
    error_sum += error;
    if (error_sum >= 255) error_sum = 255;
    if (error_sum <= -255) error_sum = -255;
}

void PIDLeft::calc_error_diff() {
    error_diff = error - error_prev;
    error_prev = error;
}

void PIDLeft::debug() {
    Serial.print("Target:");
    Serial.print(target_rpm);
    Serial.print(",");
    Serial.print("Actual:");
    Serial.println(current_rpm);
    Serial.print("Output:");
    Serial.println(u);
}

void PIDLeft::calculate_output() {
    if (!dcEL.isUpdated()) return;
    
    calc_error();
    calc_error_sum();
    calc_error_diff();
    u = Kp * error + Ki * error_sum + Kd * error_diff;
    
    if (u >= 0) {
        digitalWrite(IN4, HIGH);
        digitalWrite(IN3, LOW);
    } else {
        digitalWrite(IN4, LOW);
        digitalWrite(IN3, HIGH);
        u = u * -1;
    }
    
    if (u > 255) u = 255;
    analogWrite(EN, u);
    debug();
}

// ==================== PIDRight Implementation ====================

PIDRight::PIDRight() 
    : iR(), dcER(), Kp(KP), Ki(KI), Kd(KD), 
      error(0), error_diff(0), error_prev(0), error_sum(0), 
      u(0), target_rpm(0), current_rpm(0) {}

void PIDRight::calc_error() {
    target_rpm = iR.calc_desired_rpm();
    current_rpm = dcER.calc_actual_rpm();
    error = target_rpm - current_rpm;
}

void PIDRight::calc_error_sum() {
    error_sum += error;
    if (error_sum >= 255) error_sum = 255;
    if (error_sum <= -255) error_sum = -255;
}

void PIDRight::calc_error_diff() {
    error_diff = error - error_prev;
    error_prev = error;
}

void PIDRight::debug() {
    Serial.print("Target:");
    Serial.print(target_rpm);
    Serial.print(",");
    Serial.print("Actual:");
    Serial.println(current_rpm);
    Serial.print("Output:");
    Serial.println(u);
}

void PIDRight::calculate_output() {
    if (!dcER.isUpdated()) return;
    
    calc_error();
    calc_error_sum();
    calc_error_diff();
    u = Kp * error + Ki * error_sum + Kd * error_diff;
    
    if (u >= 0) {
        digitalWrite(IN4, HIGH);
        digitalWrite(IN3, LOW);
    } else {
        digitalWrite(IN4, LOW);
        digitalWrite(IN3, HIGH);
        u = u * -1;
    }
    
    if (u > 255) u = 255;
    analogWrite(EN, u);
    debug();
}
