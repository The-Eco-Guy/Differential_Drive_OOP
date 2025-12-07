#include "PID.h"

// Constructor implementation
// Initializes Input with the specific side (-1 or 1)
PID::PID(int side, float kp, float ki, float kd) 
  : input(side), dcEnc(), Kp(kp), Ki(ki), Kd(kd) {
}

void PID::calc_error() {
  target_rpm = input.calc_desired_rpm();
  current_rpm = dcEnc.calc_actual_rpm();
  
  error = target_rpm - current_rpm;
}

void PID::calc_error_sum() {
  error_sum += error;
  if (error_sum >= 255) error_sum = 255;
  if (error_sum <= -255) error_sum = -255;
}

void PID::calc_error_diff() {
  error_diff = error - error_prev;
  error_prev = error;
}

void PID::debug() {
  Serial.print("Target:");
  Serial.print(target_rpm);
  Serial.print(",");
  Serial.print("Actual:");
  Serial.println(current_rpm);
  Serial.print("Output");
  Serial.println(u);
}

void PID::calculate_output() {
  if (!dcEnc.isUpdated()) return; // Only runs every 100ms
  
  calc_error();
  calc_error_sum();
  calc_error_diff();
  
  u = Kp * error + Ki * error_sum + Kd * error_diff;
  
  if (u >= 0) {
    digitalWrite(IN4_PIN, HIGH);
    digitalWrite(IN3_PIN, LOW);
  } else {
    digitalWrite(IN4_PIN, LOW);
    digitalWrite(IN3_PIN, HIGH);
    u = u * -1;
  }
  
  if (u > 255) u = 255;
  
  analogWrite(EN_PIN, u);

  debug(); 
}
