#ifndef DCENC_H
#define DCENC_H

#include <Arduino.h>
#include <Encoder.h>
#include "Constants.h"

class DCEnc {
  private:
    Encoder myEnc;
    long oldPos = 0;
    unsigned long lastTime = 0;
    float actual_rpm = 0;

  public:
    static const int samplingInterval = 100;

    // Pins 2 and 3 are hardcoded in the initializer list as per original files
    DCEnc();
    
    bool isUpdated();
    float calc_actual_rpm();
};

#endif
