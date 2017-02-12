//MainIMU
#include "SPI.h"
#include "IMU.h"

/* Pin used for SPI connection */
const int chipSelectPinIMU = 65;
// the rest are controlled by the SPI library

//initialize arrays
float gyro_data[3];
float euler_data[3];


void setup() {
  Serial.begin(9600); //initializes Arduino to transmit data 
                      // at 9600 bits/sec, default for Arduino's
  
  // Start the SPI library:
  digitalWrite(chipSelectPinIMU, LOW); //IMU ignores Arduino
  SPI.begin();
  // Set the chip select pin for IMU:
  pinMode(chipSelectPinIMU, OUTPUT);  //set pin 7 as an OUTPUT pin
  
  // Give the sensor time to set up:
  delay(1000);
}

void loop() {
  //get gyro rate data
  gyro_data[3] = get_gyro(chipSelectPinIMU);

  //get euler angle data
  euler_data[3] = get_euler(chipSelectPinIMU);


/* Print data received from IMU */
  for (int kk=0;kk<3;kk++){
    if (kk==0){
    Serial.print("Pitch rate: ");
    Serial.print(gyro_data[kk]);
    Serial.print("\t\t");
    Serial.print("Pitch: ");
    Serial.print(euler_data[kk]);
    }
    else if (kk==1){
    Serial.println("Yaw rate: ");
    Serial.print(gyro_data[kk]);
    Serial.print("\t\t");
    Serial.print("Yaw: ");
    Serial.print(euler_data[kk]);
    }
    else if (kk==2){
    Serial.println("Roll rate: ");
    Serial.print(gyro_data[kk]);
    Serial.print("\t\t");
    Serial.print("Roll: ");
    Serial.print(euler_data[kk]);
    }
  }
  Serial.println("---------------------------------");

  
  delay(100);  //Wait x/1000 seconds before next loop
  
}

