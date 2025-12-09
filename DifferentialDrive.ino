#include "Constants.h"
#include "PID.h"

// --- CONFIGURATION ---
// Set this to true for Left Motor code, false for Right Motor code
const bool isLeftMotor = true; 
// ---------------------

// STATIC ALLOCATION
// We use the ternary operator (?) to set the direction based on the config above.
PID motorPID(isLeftMotor ? -1 : 1, 2.0, 0.0, 0.0);

void setup() {
  Serial.begin(9600); 
  
  pinMode(EN_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  
  // Initializing motor state
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(EN_PIN, 0);

}

void loop() {
  motorPID.calculate_output();
}
