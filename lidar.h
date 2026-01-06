#ifndef LIDAR_H
#define LIDAR_H

#include <Wire.h>
#include <Arduino.h>
#include <TFMPI2C.h>
//#include "printf.h"

class LiDAR {
public:
    bool begin();
    bool getData( float &dist, int16_t &flux, int16_t &temp);
    
private:
    TFMPI2C tfmP;

    static const uint8_t PIN_WIRE1_SDA = 21;
    static const uint8_t PIN_WIRE1_SCL = 22;
};

#endif