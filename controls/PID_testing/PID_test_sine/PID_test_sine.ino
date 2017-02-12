#include "IMU.h"
#include <math.h>

/*Define definite variables*/
//Front Motor
#define PWM_front 9
#define DIR 46
int steer_dir = 0;  

//Rear Motor
#define PWM_rear 8

//Balance Control constants
const int k1 = 71;
const int k2 = 21;
const int k3 = -20;

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
float PID_output = 0;
float sp_error = 0;
float sv_error = 0;
int pwm = 0;

//count the number of times the time step has been calculated to calculate a running average time step
int numTimeSteps = 0;
float averageTimeStep = 0;

float desired_steer = 0;

//front motor PID contants
float K_p = 60;
float K_d = 2.5;
float K_i = 0;

//timers for each channel
int duration_CH1, duration_CH2, duration_CH3, duration_CH4, duration_CH5, duration_CH6;
int start_CH1, start_CH2, start_CH3, start_CH4, start_CH5, start_CH6;
int end_CH1, end_CH2, end_CH3, end_CH4, end_CH5, end_CH6;
//current cycle's logic
boolean CH1, CH2, CH3, CH4, CH5, CH6;
//RC variables
float desired_angle;  //CH1
int PWM_rear_output;  //CH3

//voltage constants and variables
const int VOLTAGE_PIN = 63; //A9
float VOLTAGE_CONST = 14.2;
float battery_voltage = 0;
float VELOCITY_VOLTAGE_K = 1.7936;
float VELOCITY_VOLTAGE_C = -1.2002;

//define maximum front wheel pwm
int maxfront_PWM = 110;

//Read the relative position of the encoder
signed int relativePos = REG_TC0_CV0;
//Read the index value (Z channel) of the encoder
signed int indexValue = REG_TC0_CV1;

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

  
  // activate clock for TC0 and TC1
  REG_PMC_PCER0 = (1<<27)|(1<<28)|(1<<29);

      // select XC0 as clock source and set capture mode
  REG_TC0_CMR0 = 5; 

  
    // activate quadrature encoder and position measure mode, no filters
  REG_TC0_BMR = (1<<9)|(1<<8)|(1<<12);

  
    // activate the interrupt enable register for index counts (stored in REG_TC0_CV1)
  REG_TC0_QIER = 1;

  
    // enable the clock (CLKEN=1) and reset the counter (SWTRG=1) 
    // SWTRG = 1 necessary to start the clock!!
  REG_TC0_CCR0 = 5;
  REG_TC0_CCR1 = 5;

  //setup Motor Outputs
  pinMode(DIR, OUTPUT);
  pinMode (PWM_front, OUTPUT);
  pinMode (PWM_rear, OUTPUT);    

//  //setup Watchdog
//  pinMode(WDI, OUTPUT);
//  pinMode(EN, OUTPUT);
//  digitalWrite(EN, LOW);
//
//  //setup Landing Gear
//  pinMode(relay1, OUTPUT);
//  pinMode(relay2, OUTPUT);
//  pinMode(relay3, OUTPUT);
//  pinMode(relay4, OUTPUT);
//
//  //setup RC
//  pinMode(RC_CH1, INPUT);
//  pinMode(RC_CH2, INPUT);
//  pinMode(RC_CH3, INPUT);
//  pinMode(RC_CH4, INPUT);
//  pinMode(RC_CH5, INPUT);
//  pinMode(RC_CH6, INPUT);
//
//  attachInterrupt(digitalPinToInterrupt(RC_CH1), ISR_CH1, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(RC_CH2), ISR_CH2, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(RC_CH3), ISR_CH3, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(RC_CH4), ISR_CH4, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(RC_CH5), ISR_CH5, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(RC_CH6), ISR_CH6, CHANGE);
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


  //the follwing loop will not terminate until wheel passes front tick on encoder twice. The second time should be passed very slowly- 
  //this will allow for the most accurate location to be found for the center alignment of the front wheel with the bike.
  //WHEN MANUALLY CONFIGURING THE WHEEL, MOVE SLOWLY TO FIND INDEX TICK VALUE IN ORDER TO HAVE THE LEAST ERROR
  
  while(indexValue==oldIndex){
    indexValue = REG_TC0_CV1;
  }  
  //redefine oldIndex to now be current y value
   oldIndex = indexValue;
   x_offset = REG_TC0_CV0;
}

/*
 * takes in desired angular velocity returns pwm
 */
int velocityToPWM (float desiredVelocity) { 
  battery_voltage = analogRead(VOLTAGE_PIN);
  Serial.println("pin 63 output " + String(battery_voltage));
  battery_voltage = battery_voltage/VOLTAGE_CONST;

  Serial.println("voltage is " + String(battery_voltage));
  pwm = 256*(desiredVelocity - VELOCITY_VOLTAGE_C)/(battery_voltage * VELOCITY_VOLTAGE_K);
  Serial.println("pwm is  " + String(pwm));

  if (desiredVelocity > 18 ){  //***TO DO*** THIS LIMITATION MUST GO ON ALL OF THE PWM GOING TO THE FRONT MOTOR, NOT JUST THE FEED FORWARD LOOP
    //put in the warning
    return maxfront_PWM;
  }else{
    return pwm;
  }
}


/*
 * intakes commanded velocity from balance controller
 * converts commanded velocity into commanded position
 */
int eulerIntegrate(float desiredVelocity){
  float desiredPosition = desiredVelocity*averageTimeStep ;
  return desiredPosition;
}

// updates global variables representing encoder position
float updateEncoderPosition(){
  //Read the relative position of the encoder
  relativePos = REG_TC0_CV0;
  //Read the index value (Z channel) of the encoder
  indexValue = REG_TC0_CV1;
  current_pos = (((relativePos - x_offset) * 0.02197 * M_PI)/180); //Angle (rad)
  return current_pos;
}

/*
 * takes in desired position and applies a PID controller to minimize error between current position and desired position
 */
void frontWheelControl(float desired_pos){
  //float desired_pos = eulerIntegrate(desiredVelocity);
  //P term
  //calculate position error (rad)
  pos_error = desired_pos - current_pos ;

  //position scaling factor should be a maximum of K_p = 100/(M_PI/2) found by taking 100 (100 being max pwm value I want to reach), and dividing 
  //by theoretical max absolute value of angle (Pi/2). This means with angles in that range, 100 will be the max PWM value 
  //outputted to the motor
  sp_error =  (K_p*pos_error);
  unsigned long currentMicros = micros();

  //obtain a running average for the value of the time step to use in the Euler integration. This time step 
  //is not constant, but a running average will work as an approximation to calculate desired position from desired velocity
  //numTimeSteps++;
  //averageTimeStep = ((averageTimeStep*(numTimeSteps-1)) + (currentMicros - previousMicros))/numTimeSteps ;
  
  //D term
  //calculates velocity error with (desired velocity - current velocity), desired velocity will always be zero
  //
  current_vel = (((((relativePos-x_offset)-oldPosition)*0.02197*1000000*M_PI/180.0)/(currentMicros-previousMicros)));   //Angular Speed(rad/s)
  previousMicros = currentMicros;

  sv_error =  (-K_d*current_vel);
  Serial.print("Vel error:");   Serial.println(sv_error);  
  
  PID_output =  sp_error + sv_error; //+ velocityToPWM(desiredVelocity);

  if (PID_output > 0) {
    digitalWrite(DIR, LOW); 
  } else {
    digitalWrite(DIR, HIGH);
  }
  analogWrite(PWM_front, abs((int)(PID_output)));

  oldPosition = relativePos-x_offset;
}

/*
 * TODO: FUNCTION THAT RETURNS DESIRED ANGULAR VELOCITY OF FRONT WHEEL
 */
float balanceController(float roll_angle, float roll_rate, float encoder_angle){
  float desiredSteerRate = (k1*roll_angle) + (k2*roll_rate) + (k3*encoder_angle);
  return desiredSteerRate;
}






void frontWheelControl(){
  //P term
  //calculate position error (rad)
  pos_error = desired_pos - current_pos ;

  //position scaling factor should be a maximum of K_p = 100/(M_PI/2) found by taking 100 (100 being max pwm value I want to reach), and dividing 
  //by theoretical max absolute value of angle (Pi/2). This means with angles in that range, 100 will be the max PWM value 
  //outputted to the motor
  sp_error =  (K_p*pos_error);
  unsigned long currentMicros = micros();
  //obtain a running average for the value of the time step to use in the Euler integration. This time step 
  //is not constant, but a running average will work as an approximation to calculate desired position from desired velocity
//  numTimeSteps++;
//  averageTimeStep = ((averageTimeStep*(numTimeSteps-1)) + (currentMicros - previousMicros))/numTimeSteps ;
  
  //D term
  //calculates velocity error with (desired velocity - current velocity), desired velocity will always be zero
  //
  current_vel = (((((relativePos-x_offset)-oldPosition)*0.02197*1000000*M_PI/180.0)/(currentMicros-previousMicros)));   //Angular Speed(rad/s)
  previousMicros = currentMicros;

  sv_error =  (-K_d*current_vel);
  Serial.print("Vel error:");   Serial.println(sv_error);  
  
  PID_output =  sp_error + sv_error; //+ velocityToPWM(desiredVelocity)

  if (PID_output > 0) {
    digitalWrite(DIR, LOW); 
  } else {
    digitalWrite(DIR, HIGH);
  }
  analogWrite(PWM_front, abs((int)(PID_output)));

  oldPosition = relativePos-x_offset;
}


unsigned long currentMicros = micros();

//modular test of IMU

void stateUpdate() {
  desired_pos = .75*sin(micros()*1000000); //sine function oscillating between -.75 and .75 such that the overall position does not exceed -45 to 45 degrees
  currentMicros = micros();
}



void loop() {
  // put your main code here, to run repeatedly:
  stateUpdate();
  frontWheelControl(desired_pos);
  

}
