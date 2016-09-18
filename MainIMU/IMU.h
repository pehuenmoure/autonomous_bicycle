#ifndef IMU_h
#define IMU_h

#include "Arduino.h"

//declare call function to convert from big to little endian
void endianSwap();

//declare call functions to retrieve data
float get_gyro(int chipSelectPinIMU);
float get_euler(int chipSelectPinIMU);

#endif /* IMU_h */
