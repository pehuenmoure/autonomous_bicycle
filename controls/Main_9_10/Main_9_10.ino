#include "IMU.h"
#include <math.h>

/*Define definite variables*/
//Front Motor
#define PWM_front 9
#define DIR 46
int steer_dir = 0;  

//Rear Motor
#define PWM_rear 8

//Control constants
#define k1 37.84
#define k2 9.85
#define k3 -11.5

//Encoder
const int quad_A = 2;
const int quad_B = 13;
const int idx = 60;
const unsigned int mask_quad_A = digitalPinToBitMask(quad_A);
const unsigned int mask_quad_B = digitalPinToBitMask(quad_B); 
const unsigned int mask_idx = digitalPinToBitMask(idx); 
int REnot = 3;
int DE = 4;
signed int oldPosition  = 0;
signed int oldIndex = 0;
unsigned long previousMicros = 0;
signed int x_offset = 0;
float desired_pos = 0;
float current_pos = 0;
float current_vel = 0;
float desired_vel = 0;
float vel_error = 0;
float pos_error = 0;
float total_error = 0;
float sp_error = 0;
float sv_error = 0;

float K_p = 100/(M_PI/2);
float K_d = 0.05;
float K_i = 0;

//Watchdog
#define WDI 42
#define EN 41

//Landing Gear
#define relay1 48
#define relay2 47
#define relay3 50
#define relay4 49

//RC
#define RC_CH1 51     //Steer Angle (Brown)
#define RC_CH2 28     //(Red)
#define RC_CH3 25     //Velocity (Orange)
#define RC_CH4 33     //(Yellow)
#define RC_CH5 27     //Kill Switch (Green)
#define RC_CH6 32     //Landing Gear (Blue)
//timers for each channel
int duration_CH1, duration_CH2, duration_CH3, duration_CH4, duration_CH5, duration_CH6;
int start_CH1, start_CH2, start_CH3, start_CH4, start_CH5, start_CH6;
int end_CH1, end_CH2, end_CH3, end_CH4, end_CH5, end_CH6;
//current cycle's logic
boolean CH1, CH2, CH3, CH4, CH5, CH6;
//RC variables
float desired_angle;  //CH1
int PWM_rear_output;  //CH3


void setup() {
  Serial.begin(9600);
  initIMU();

  //setup Encoder
  pinMode(REnot, OUTPUT);
  pinMode(DE, OUTPUT);
  // activate peripheral functions for quad pins
  REG_PIOB_PDR = mask_quad_A;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_quad_A;   // choose peripheral option B    
  REG_PIOB_PDR = mask_quad_B;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_quad_B;   // choose peripheral option B 
  REG_PIOB_PDR = mask_idx;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_idx;   // choose peripheral option B 
  REG_PMC_PCER0 = (1<<27)|(1<<28)|(1<<29);
  REG_TC0_CMR0 = 5; 
  REG_TC0_BMR = (1<<9)|(1<<8)|(1<<12);
  REG_TC0_QIER = 1;
  REG_TC0_CCR0 = 5;
  REG_TC0_CCR1 = 5;

  //setup Motor Outputs
  pinMode(DIR, OUTPUT);
  pinMode (PWM_front, OUTPUT);
  pinMode (PWM_rear, OUTPUT);    

  //setup Watchdog
  pinMode(WDI, OUTPUT);
  pinMode(EN, OUTPUT);
  digitalWrite(EN, LOW);

  //setup Landing Gear
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  pinMode(relay4, OUTPUT);

  //setup RC
  pinMode(RC_CH1, INPUT);
  pinMode(RC_CH2, INPUT);
  pinMode(RC_CH3, INPUT);
  pinMode(RC_CH4, INPUT);
  pinMode(RC_CH5, INPUT);
  pinMode(RC_CH6, INPUT);

  attachInterrupt(digitalPinToInterrupt(RC_CH1), ISR_CH1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH2), ISR_CH2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH3), ISR_CH3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH4), ISR_CH4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH5), ISR_CH5, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CH6), ISR_CH6, CHANGE);
//  analogWrite(PWM_rear, 100);

  //request desired angle value, do not proceed until an angle value has been provided
//  int  message_delivered = 0;
//   while (!(Serial.available())) {
//
//    if (message_delivered == 0) {
//      Serial.println("enter an angular float value between -45.0 and 45.0 degrees, then rotate wheel past the center line twice (in different directions) to calibrate ");
//      message_delivered = 1;
//    }
//   }
   x_offset = REG_TC0_CV0;
}

void loop() {
  /*RESET WATCHDOG*/
  digitalWrite(EN, HIGH);
  //delayMicroseconds(1);
  digitalWrite(EN, LOW);

//analogWrite(PWM_front, 10);
   delay(20);
   //analogWrite(PWM_front, 10);
   digitalWrite(DIR, HIGH);
   delay(50);
   digitalWrite(DIR, LOW);
   //analogWrite(PWM_rear, 10);
  
//   Serial.print("current velocity:     "); Serial.println(current_vel);
   int pwm = 0;
   while (pwm < 255){
    analogWrite(PWM_front, pwm);
    pwm += 10;
    delay(1000);
    signed int x = REG_TC0_CV0;
    signed int y = REG_TC0_CV1;
    oldPosition = x-x_offset;
    current_pos = (((x - x_offset) * 0.02197 * M_PI)/180); //Angle (rad)
    unsigned long currentMicros = micros();
    current_vel = (((((x-x_offset)-oldPosition)*0.02197*1000000*M_PI/180.0)/(currentMicros-previousMicros)));   //Angular Speed(rad/s)
    Serial.println(String(pwm) + "  " + String(current_vel));
    previousMicros = currentMicros;
    oldPosition = x-x_offset;
   }

   

// 
//  /*RC INPUTS*/
//  /*
//  //Velocity (Channel 3)
//  PWM_rear_output = 0.11041009463 * duration_CH3;
//  PWM_rear_output = PWM_rear_output - 49.9369085174;
//  //Safety checks
//  if (duration_CH3 < 1041 || PWM_rear_output < 65) {
//    PWM_rear_output = 65;
//  }
//  if (duration_CH3 > 1992 || PWM_rear_output > 170) {
//    PWM_rear_output = 170;
//  }
//  //Serial.println(PWM_rear_output);
//  
//
//  //Steering (Channel 4)
//  desired_angle = -0.140625 * duration_CH4;
//  desired_angle = desired_angle + 210.65625;
//  //convert to radians
//  desired_angle = desired_angle * M_PI;
//  desired_angle = desired_angle/180;
//  //Safety checks
//  if (duration_CH4 >= 1495 && duration_CH4 <= 1515) {
//    desired_angle = 0;
//  }
//  if (duration_CH4 > 1818 || desired_angle < -M_PI/4) {
//    desired_angle = -M_PI/4;
//  }
//  if (duration_CH4 < 1178 || desired_angle > M_PI/4) {
//    desired_angle = M_PI/4;
//  }
  //Serial.println(desired_angle);
//
// 
//  //Kill Switch (Channel 5)
//  if (duration_CH5 >= 2000) {
//     //Serial.println("Bike is ON");
//  }
//  else {
//     //Serial.println("Bike is OFF");
//    while(1);      //sets off watchdog
//  }
//
//
//  //Landing Gear (Channel 6)
//  if (duration_CH6 <= 1504) {
//    digitalWrite(relay1, HIGH);
//    digitalWrite(relay2, HIGH);
//    //Serial.println("Landing gear NOT DEPLOYED");
//  }
//  else {
//    digitalWrite(relay1, LOW);
//    digitalWrite(relay2, LOW);
//    //Serial.println("Landing gear DEPLOYED");
//  }
//  */
//
//  /*DATA COLLECTION*/
//  //get data from IMU (4 milliseconds)
//  float roll_angle = getIMU(0x01);   //get roll angle
//  /*
//  Serial.print("\nRoll Angle: ");
//  Serial.print(roll_angle*180/3.14159,4);
//  */
//  float roll_rate = getIMU(0x26);    //get roll rate    
//  /*
//  Serial.print("\t\tRoll Rate: ");
//  Serial.print(roll_rate*180/3.14159,4);
//  */
//
//  /*
//  if((roll_angle*180)/M_PI > 45 || (roll_angle*180)/M_PI < -45){
//    while(1);
//  }
//  */
//  
//  //get data from Encoder 
//  digitalWrite(REnot, LOW);
//  digitalWrite(DE, LOW);
//  signed int x = REG_TC0_CV0;
//  signed int y = REG_TC0_CV1;
//  unsigned long currentMicros = micros();
//  float encoder_rate = ((((x-oldPosition)*0.02197*1000000*M_PI/180.0)/(currentMicros-previousMicros))); //Angular Speed(rad/s)
//  encoder_rate = encoder_rate * -1;
//  previousMicros = currentMicros;
//  float encoder_angle = ((x * 0.02197 * M_PI)/180); //Angle (rad)
//  encoder_angle = encoder_angle * -1;
//  oldPosition = x; 
//
//  /*
//  Serial.println();
//  Serial.print("Encoder Angle: ");
//  Serial.print((encoder_angle*180)/M_PI);       //prints in degrees
//  Serial.print("\t\t");
//  Serial.print("Encoder Rate: ");
//  Serial.print((encoder_rate*180)/M_PI);
//  */
//
//  /*
//  if((encoder_angle*180)/M_PI > 60 || (encoder_angle*180)/M_PI < -60){
//    while(1);
//  }
//  */
//
//
//  /*DETERMINE OUTPUTS*/
//  //input sensor data into control algorithm
//  float desired_steer = (k1*roll_angle) + (k2*roll_rate) + (k3*(encoder_angle - desired_angle));
//  /*
//  Serial.println();
//  Serial.print("Desired Steer: ");
//  Serial.print(desired_steer);
//  */
//
//  //set direction of steer motor
//  if(desired_steer>=0){
//    steer_dir = 0;
//    digitalWrite(DIR, HIGH);        //clockwise
//    /*
//    Serial.println();
//    Serial.print("HIGH");
//    */
//  }
//  else{
//    steer_dir = 1;
//    digitalWrite(DIR, LOW);       //counter-clockwise
//    /*
//    Serial.println();
//    Serial.print("LOW");
//    */
//  }
//
//
//  /*OUTPUT TO MOTORS*/
//  //output PWM to front motor
//  int desired_PWM;
//  
//  //CLOCKWISE
//  if(steer_dir == 0){
//    desired_steer = desired_steer*-1;
//    if(desired_steer > -6.65667){     //first line (intercept at 0,0)
//      desired_PWM = desired_steer*-16.548;
//    }
//    else{     //second line
//      desired_PWM = (desired_steer*-46.616)-193.43;
//    }
// 
//    if(desired_PWM > 255){
//      desired_PWM = 255;
//    }
//    
//    if(desired_PWM >= 0 && desired_PWM <= 255){   
//      analogWrite(PWM_front, desired_PWM);    //output to steer motor
//      /*
//      Serial.println();
//      Serial.print("PWM_front: ");
//      Serial.print(desired_PWM);
//      */
//    }
//  }
//  
//  //COUNTER-CLOCKWISE
//  else{   //counter-clockwise
//    desired_steer = desired_steer*-1;
//    if(desired_steer < 6.18){     //first line (intercept at 0,0)
//      desired_PWM = desired_steer*17.275;
//    }
//    else{     //second line
//      desired_PWM = (desired_steer*43.902)-168.02;
//    }
//    
//    if(desired_PWM > 255){
//      desired_PWM = 255;
//    }
//    
//    if(desired_PWM >= 0 && desired_PWM <= 255){   
//      analogWrite(PWM_front, desired_PWM);    //output to steer motor
//      /*
//      Serial.println();
//      Serial.print("PWM_front: ");
//      Serial.print(desired_PWM);
//      */
//    }
//  }
//
//  //output PWM to rear motor
//  //analogWrite(PWM_rear, PWM_rear_output);
//  /*
//  Serial.println();
//  Serial.print("PWM_rear: ");
//  Serial.print(PWM_rear_output);
//  
//  Serial.println();
//  Serial.println();
//  */ 
}
//END OF LOOP




//Interrupt Service Routines
void ISR_CH1() {
  noInterrupts();
  CH1 = digitalRead(RC_CH1);
  if (CH1 == HIGH) {
    start_CH1 = micros();
  }
  else {
    end_CH1 = micros();
    duration_CH1 = end_CH1 - start_CH1;
  }
  interrupts();
}

void ISR_CH2() {
  noInterrupts();
  CH2 = digitalRead(RC_CH2);
  if (CH2 == HIGH) {
    start_CH2 = micros();
  }
  else {
    end_CH2 = micros();
    duration_CH2 = end_CH2 - start_CH2;
  }
  interrupts();
}

void ISR_CH3() {
  noInterrupts();
  CH3 = digitalRead(RC_CH3);
  if (CH3 == HIGH) {
    start_CH3 = micros();
  }
  else {
    end_CH3 = micros();
    duration_CH3 = end_CH3 - start_CH3;
  }
  interrupts();
}

void ISR_CH4() {
  noInterrupts();
  CH4 = digitalRead(RC_CH4);
  if (CH4 == HIGH) {
    start_CH4 = micros();
  }
  else {
    end_CH4 = micros();
    duration_CH4 = end_CH4 - start_CH4;
  }
  interrupts();
}

void ISR_CH5() {
  noInterrupts();
  CH5 = digitalRead(RC_CH5);
  if (CH5 == HIGH) {
    start_CH5 = micros();
  }
  else {
    end_CH5 = micros();
    duration_CH5 = end_CH5 - start_CH5;
  }
  interrupts();
}

void ISR_CH6() {
  noInterrupts();
  CH6 = digitalRead(RC_CH6);
  if (CH6 == HIGH) {
    start_CH6 = micros();
  }
  else {
    end_CH6 = micros();
    duration_CH6 = end_CH6 - start_CH6;
  }
  interrupts();
}


