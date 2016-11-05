#include "IMU.h"
#include "PID.h"
#include <math.h>

/*Define definite variables*/
//Front Motor
#define PWM_front 9
#define DIR 46
int steer_dir = 0;  

//Rear Motor
#define PWM_rear 8

//Timed Loop Variables
const long interval = 10000;
long l_start;
long l_diff;

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
unsigned long previous_t = 0;
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
//float K_p = 60;
//float K_d = 2.5;
//float K_i = 0;

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
  Serial.begin(115200);
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

  analogWrite(PWM_rear, 100);
  //request desired angle value, do not proceed until an angle value has been provided
  int  message_delivered = 0;
   while (!(Serial.available())) {
    if (message_delivered == 0) {
      Serial.println("Calibrate the front wheel") ;
      message_delivered = 1;
    }
   }

  //the follwing loop will not terminate until wheel passes front tick on encoder twice. The second time should be passed very slowly- 
  //this will allow for the most accurate location to be found for the center alignment of the front wheel with the bike.
  signed int y = REG_TC0_CV1; 
  oldIndex = y;
  digitalWrite(DIR, HIGH); 
  while(y==oldIndex){
    analogWrite(PWM_front,20);
    y = REG_TC0_CV1;
  }  
  
  delay(500) ;
  oldIndex = y;
  digitalWrite(DIR, LOW); 
  
  while(y==oldIndex){
    analogWrite(PWM_front,20);
    y = REG_TC0_CV1;
  }  
  
  //redefine oldIndex to now be current y value
   oldIndex = y;

  //set x offset to define where the front tick is with respect to the absolute position of the encoder A and B channels
   x_offset = REG_TC0_CV0;

}

/* takes in desired angular velocity returns pwm */
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


/* intakes commanded velocity from balance controller
 * converts commanded velocity into commanded position */
int eulerIntegrate(float desiredVelocity, float current_pos){
  float desiredPosition = current_pos + desiredVelocity*averageTimeStep ;
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

/* takes in desired position and applies a PID controller to minimize error between current position and desired position */
void frontWheelControl(float desiredVelocity, float current_pos){
  float desired_pos = eulerIntegrate(desiredVelocity, current_pos);
  
  unsigned long current_t = micros();
  PID_Controller(desired_pos, relativePos, x_offset, current_t, previous_t, oldPosition);
  
  previous_t = current_t;
  oldPosition = relativePos-x_offset;
}

/* FUNCTION THAT RETURNS DESIRED ANGULAR VELOCITY OF FRONT WHEEL */
float balanceController(float roll_angle, float roll_rate, float encoder_angle){
  float desiredSteerRate = (k1*roll_angle) + (k2*roll_rate) + (k3*encoder_angle);
  return desiredSteerRate;
}

struct roll_t{
  float rate;
  float angle;
};

// Retrieve data from IMU about roll angle and rate and return it
struct roll_t updateIMUData(){
  roll_t roll_data;
  //get data from IMU
  float roll_angle = getIMU(0x01);   //get roll angle
//  Serial.print("\nRoll Angle: ");
//  Serial.print(roll_angle,4);
  float roll_rate = getIMU(0x26);    //get roll rate
//  Serial.print("\t\tRoll Rate: ");
//  Serial.print(roll_rate,4);
//  Serial.print("\n--------------------------------------------------\n");  
  roll_data.angle = roll_angle;
  roll_data.rate = roll_rate;
  return roll_data;
}

//Loop variables
//int l_count = 0;
int num_loops = 10;
void loop() {
//  if (l_count < num_loops){
    l_start = micros();
    
    float encoder_position = updateEncoderPosition();
//    roll_t imu_data = updateIMUData();
//    float desiredVelocity = balanceController(imu_data.angle, imu_data.rate, encoder_position);//NEED TO UPDATE ROLL ANGLE AND RATE
    
    frontWheelControl(desiredVelocity, encoder_position);  //DESIRED VELOCITY FROM BALANCE CONTROLLER - NEED TO UPDATE
    
//    l_count += 1;
    l_diff = l_start - micros();
    if (l_diff < interval){
      delayMicroseconds(interval - l_diff);
    }
    Serial.println(micros() - l_start);
//  }else{
//    //Pring values here
//    Serial.println(l_diff);
//    l_count = 0;
//  }
}
