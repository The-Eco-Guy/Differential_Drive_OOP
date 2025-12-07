#include "DCEnc.h"

DCEnc::DCEnc() : myEnc(ENC_PIN_A, ENC_PIN_B), oldPos(0), lastTime(0), actual_rpm(0) {}

bool DCEnc::isUpdated() {
    if (millis() - lastTime >= SAMPLING_INTERVAL)
        return true;
    else
        return false;
}

float DCEnc::calc_actual_rpm() {
    if (millis() - lastTime >= SAMPLING_INTERVAL) {
        long newPos = myEnc.read();
        long pulseDiff = newPos - oldPos;
        lastTime = millis();
        oldPos = newPos;
        actual_rpm = ((pulseDiff * 60000.0) / (PULSES_PER_ROTATION * SAMPLING_INTERVAL));
        return actual_rpm;
    }
    return actual_rpm;
}
