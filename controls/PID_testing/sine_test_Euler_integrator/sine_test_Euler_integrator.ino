
#include <math.h>
#define PWM_front 9   //changed from 8 to 9
#define DIR 46     //direction of rotation is a HIGH/LOW value assigned to pin 46

const int quad_A = 2;
const int quad_B = 13;
const int idx = 60;
//const int quad_A = 4;
//const int quad_B = 5;
//const int idx = 10;
const unsigned int mask_quad_A = digitalPinToBitMask(quad_A);
const unsigned int mask_quad_B = digitalPinToBitMask(quad_B); 
const unsigned int mask_idx = digitalPinToBitMask(idx); 

int REnot = 3;
int DE = 4;

signed int oldPosition  = 0;
signed int oldIndex = 0;
float currentMicros = 0;
float previousMicros = 0;
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
float t = 0;
float delta_t = 0;
int numTimeSteps = 0;
float averageTimeStep;

float StepValues[1100];
int k = 0;

float K_p = 100;
float K_d = 16;
float K_i = 0;

//for (i = 1:100)
//
//
//end




void setup() {
    Serial.begin(9600);  

//set up pin to power front motor
    pinMode (PWM_front, OUTPUT);
// define direction of front motor
    pinMode (DIR, OUTPUT);

//set up pins for encoder function
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
    //REG_TC2_CMR0 = 6; 
    
    // activate quadrature encoder and position measure mode, no filters
    REG_TC0_BMR = (1<<9)|(1<<8)|(1<<12);
    //REG_TC2_BMR = (1<<9)|(1<<8)|(1<<12);
    
    // activate the interrupt enable register for index counts (stored in REG_TC0_CV1)
    REG_TC0_QIER = 1;
    //REG_TC2_QIER = 1;
    
    // enable the clock (CLKEN=1) and reset the counter (SWTRG=1) 
    // SWTRG = 1 necessary to start the clock!!
    REG_TC0_CCR0 = 5;
    REG_TC0_CCR1 = 5;
    //REG_TC2_CCR0 = 5;
    //REG_TC2_CCR1 = 5;

  digitalWrite(REnot, LOW);
  digitalWrite(DE, LOW);

//request desired angle value, do not proceed until an angle value has been provided
  int  message_delivered = 0;
   while (!(Serial.available())) {

    if (message_delivered == 0) {
//      Serial.println("enter an angular float value between -45.0 and 45.0 degrees, then rotate wheel past the center line twice (in different directions) to calibrate ");
    Serial.println("press any key to calibrate front wheel") ;
      message_delivered = 1;
    }
   }

  //the follwing loop will not terminate until wheel passes front tick on encoder twice. The second time should be passed very slowly- 
  //this will allow for the most accurate location to be found for the center alignment of the front wheel with the bike.
  //WHEN MANUALLY CONFIGURING THE WHEEL, MOVE SLOWLY TO FIND INDEX TICK VALUE IN ORDER TO HAVE THE LEAST ERROR
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
////redefine oldIndex to now be current y value
   oldIndex = y;

//set x offset to define where the front tick is with respect to the absolute position of the encoder A and B channels
   x_offset = REG_TC0_CV0;
}
//create a varaible to convert error to pwm output value
int scaled_error=0;

float eulerIntegrate(float desiredVelocity, float current_pos){
  float desiredPosition = current_pos + desiredVelocity*delta_t ;
  return desiredPosition;
}

void loop() {

  digitalWrite(REnot, LOW);
  digitalWrite(DE, LOW);
  signed int x = REG_TC0_CV0;
  current_pos = (((x - x_offset) * 0.02197 * M_PI)/180); //Angle (rad)
  
  signed int y = REG_TC0_CV1;
  //signed int x = REG_TC2_CV0;
  //signed int y = REG_TC2_CV1;
  
  currentMicros = micros();
  //Serial.println(String(currentMicros) + "\t" + String(previousMicros) + "\t" + String(currentMicros/1000000) + "\t" + String((currentMicros - previousMicros)/(1000000)));
  //Serial.println((((x-oldPosition)*0.02197*1000000*M_PI/180.0)/(currentMicros-previousMicros)));   //Angular Speed(rad/s)
//Define desired position to be a sine wave. This will be used when comparing the output plot of position of the wheel in order to show the responsiveness of the motor
 t = millis()/(1000);
 
//Serial.print("t:");   Serial.println(t);
//desired_pos = 1*sin((t));
desired_vel = 0.0 * cos(t);
desired_pos = 0.0 ;
  float sint = sin(t);
//  Serial.println(String(sint) + "\t" + String(desired_pos) + "\t" + String(desired_vel));

  //obtain a running average for the value of the time step to use in the Euler integration. This time step 
  //is not constant, but a running average will work as an approximation to calculate desired position from desired velocity
  numTimeSteps++;
  averageTimeStep = ((averageTimeStep*(numTimeSteps-1)) + (currentMicros - previousMicros)/(1000000))/numTimeSteps ;
  //P term
  //calculate position error (rad)
  pos_error = desired_pos - current_pos ;
  //scaled positional error
  //position scaling factor K_p = 100/(M_PI/2) found by taking 100 (100 being max pwm value I want to reach), and dividing by theoretical max absolute value of angle (Pi/2). This means with angles in that range, 100 will be the max PWM value outputted to the motor
  sp_error =  (K_p*pos_error);  

 //D term //calculate velocity error
  current_vel = (((((x-x_offset)-oldPosition)*0.02197*1000000*M_PI/180.0)/(currentMicros-previousMicros)));   //Angular Speed(rad/s)

  //calculate the value of the current time step in microseconds
  delta_t = (currentMicros-previousMicros)/(1000000) ;
  previousMicros = currentMicros;
  // the value of the velocity error will be negative of the current velocity (in order to resist current direction of motion). Calculated as target_velocity - current_velocity where target velocity is always 0 
  //scaled velocity error
  sv_error =  (-K_d*current_vel)  ;  
  total_error =  sp_error + sv_error ;

  //print total error to get a sense of how high the values are for a normal sine wave.
  //  Serial.println(total_error) ;
  if (total_error > 0) {
  digitalWrite(DIR, LOW); 
  } else {
  digitalWrite(DIR, HIGH);
  }  
  Serial.println(String(current_pos) + "\t" + String(desired_pos) + "\t" + String(current_vel) + "\t" + String(desired_vel) + "\t" + String(delta_t) + "\t" + String(averageTimeStep));

  //clip the maximum output to the motor by essentially saying "if the value is greater than this threshold, make the output to the motor this exact threshold value"
   if (total_error > 25 || total_error < -25) {
      analogWrite(PWM_front, 25);
   }  else { 
      analogWrite(PWM_front, abs((int)(total_error))); 
   }
  oldPosition = current_pos;
}
  

