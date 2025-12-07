#include "DCEnc.h"

// Initialize Encoder with pins 2 and 3
DCEnc::DCEnc() : myEnc(2, 3) {}

bool DCEnc::isUpdated() {
  if (millis() - lastTime >= samplingInterval)
    return true;
  else
    return false;
}

float DCEnc::calc_actual_rpm() {
  if (millis() - lastTime >= samplingInterval) {
    long newPos = myEnc.read();
    long pulseDiff = newPos - oldPos;
    lastTime = millis();
    oldPos = newPos;
    actual_rpm = ((pulseDiff * 60000.0) / (PPR * samplingInterval));
    return actual_rpm;
  }
  return actual_rpm;
}
