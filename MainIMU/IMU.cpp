#include "IMU.h"
#include "SPI.h"

/* Union for each of the 9 sensors */
// Needed to convert the bytes from SPI to float
union u_gyro {
    byte b[4];
    float fval;
} gyro[3];  // Create 3 unions, one for each gyro rate from the IMU
// Gyroscope: x-axis (pitch rate), y-axis (yaw rate), z-axis (roll rate)

union u_euler {
    byte b[4];
    float fval;
} euler[3]; // Create 3 unions, one for each gyro rate from the IMU
// Gyroscope: x-axis (pitch), y-axis (yaw), z-axis (roll)

/* Endian swap - big to little */
inline void endianSwap(byte temp[4]) {
    byte myTemp = temp[0];
    temp[0] = temp[3];
    temp[3] = myTemp;
    myTemp = temp[1];
    temp[1] = temp[2];
    temp[2] = myTemp;
}

/* Gyro rates*/
/////////////////////////////////////////////////////////////////
float get_gyro(int chipSelectPinIMU){
    // Clear the internal data buffer on the IMU
    SPI.transfer(0x01); //any command sent (besides 0xF6 & 0xFF) when
    //IMU is not processing a command will reset buffer
    delay(10);
    
    // Send command:
    SPI.transfer(0xF6); //must be sent before each command packet
    delay(1);
    
    SPI.transfer(0x26); //gets corrected gyro rate
    delay(1);
    
    // Wait for IMU to begin sending data
    byte result = SPI.transfer(0xFF); //0xFF reads from sensor
    while (result != 0x01) {  // Repeat until device is ready to send data (returns a 1)
        delay(1);
        result = SPI.transfer(0xFF);
    }
    
    // Get the 12 bytes of return data from the device:
    for (int ii=0; ii<3; ii++) {
        for (int jj=0; jj<4; jj++) {
            gyro[ii].b[jj] = SPI.transfer(0xFF);
            delay(1);
        }
    }
    
    // Convert Big Endian (IMU) to Little Endian (Arduino)
    for( int mm=0; mm<3; mm++) {
        endianSwap(gyro[mm].b); //calls function endianSwap
    }
    
    //create new array to store fval from gyro union
    float gyro_val[3];
    
    //store fval from gyro union to array that's returned
    for(int iii=0; iii<3; iii++){
        gyro_val[iii] = gyro[iii].fval;
    }
    
    return gyro_val[3];
}



/*Euler Angles*/
//////////////////////////////////////////////////////////////////
float get_euler(int chipSelectPinIMU){
    // Clear the internal data buffer on the IMU
    SPI.transfer(0x01); //any command sent (besides 0xF6 & 0xFF) when
    //IMU is not processing a command will reset buffer
    delay(10);
    
    // Send command:
    SPI.transfer(0xF6); //must be sent before each command packet
    delay(1);
    
    SPI.transfer(0x01); //gets tared euler angles
    delay(1);
    
    // Wait for IMU to begin sending data
    byte result = SPI.transfer(0xFF); //0xFF reads from sensor
    while (result != 0x01) {  // Repeat until device is ready to send data (returns a 1)
        delay(1);
        result = SPI.transfer(0xFF);
    }
    
    // Get the 12 bytes of return data from the device:
    for (int ii=0; ii<3; ii++) {
        for (int jj=0; jj<4; jj++) {
            euler[ii].b[jj] = SPI.transfer(0xFF);
            delay(1);
        }
    }
    
    // Take the chip select high to de-select:
    digitalWrite(chipSelectPinIMU, HIGH);
    
    // Convert Big Endian (IMU) to Little Endian (Arduino)
    for( int mm=0; mm<3; mm++) {
        endianSwap(euler[mm].b); //calls function endianSwap
    }
    
    //create new array to store fval from euler union
    float euler_val[3];
    
    //store fval from euler union to array that's returned
    for(int jjj=0; jjj<3; jjj++){
        euler_val[jjj] = euler[jjj].fval;
    }
    
    return euler_val[3];
}




