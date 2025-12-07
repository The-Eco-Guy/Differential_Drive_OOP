#include "Constants.h"
#include "PID.h"

// --- CONFIGURATION ---
// Set this to true for Left Motor code, false for Right Motor code
bool isLeftMotor = true; 
// ---------------------

PID* motorPID;

void setup() {
  Serial.begin(9600); 
  
  pinMode(EN_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  
  // Initializing motor state
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(EN_PIN, 0);

  if (isLeftMotor) {
    // Left Motor Configuration: 
    // Side = -1
    // PID = 2, 0.001, 0.001 (Values from your left file)
    motorPID = new PID(-1, 2.0, 0.001, 0.001);
  } else {
    // Right Motor Configuration:
    // Side = 1
    // PID = 2, 0, 0 (Values from your right file)
    motorPID = new PID(1, 2.0, 0.0, 0.0);
  }
}

void loop() {
  motorPID->calculate_output();
}
