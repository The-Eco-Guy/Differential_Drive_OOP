#ifndef DCENC_H
#define DCENC_H

#include <Encoder.h>
#include "config.h"

class DCEnc {
private:
    Encoder myEnc;
    long oldPos;
    unsigned long lastTime;
    float actual_rpm;

public:
    DCEnc();
    bool isUpdated();
    float calc_actual_rpm();
};

#endif
