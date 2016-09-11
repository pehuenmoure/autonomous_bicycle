
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
  signed int y = REG_TC0_CV1;
  oldIndex = y;

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
  while(y==oldIndex){
    y = REG_TC0_CV1;
}  
//redefine oldIndex to now be current y value
   oldIndex = y;

//set x offset to define where the front tick is with respect to the absolute position of the encoder A and B channels
   x_offset = REG_TC0_CV0;

}  

//create a varaible to convert error to pwm output value
int scaled_error=0;
int pwm = 0;

void loop() {
    //read latest input from serial window
//    if (Serial.available()) {  //checks if there has been a value inputted to the serial window
//    desired_pos = (Serial.parseFloat()*M_PI/180);   //reads in float value, converts it to radians
//  }
  pwm += 5;
  
  if (pwm > 250) {
    pwm = 0;
  }
 
  analogWrite(PWM_front, pwm);
  delay(2000);
  
//  Serial.print("desired angle:      ");
//  Serial.println(desired_pos);

  
  digitalWrite(REnot, LOW);
  digitalWrite(DE, LOW);
  signed int x = REG_TC0_CV0;
  signed int y = REG_TC0_CV1;
  //signed int x = REG_TC2_CV0;
  //signed int y = REG_TC2_CV1;
  //unsigned long currentMicros = micros();
  //Serial.println((((x-oldPosition)*0.02197*1000000*M_PI/180.0)/(currentMicros-previousMicros)));   //Angular Speed(rad/s)
  //previousMicros = currentMicros;

//when y value changes (when wheel passes index tick) print absolute position of the wheel now to see if encoder absolute position
//is drifting

      /*
    if (y!= oldIndex) { 
    Serial.println("Number of Ticks in that Revolution:  ") ;
    //Serial.println(oldPosition); //extract old x value, number of ticks before the index tick changed y value
    //x_offset = x; //absolute x reading when wheel facing forward (z index tripped)
  } */

  //Serial.println(x_offset);
//  Serial.print(x - x_offset); 
//  Serial.print("   ");
//
//  Serial.print(y); 
//  Serial.print("   ");

 current_pos = (((x - x_offset) * 0.02197 * M_PI)/180); //Angle (rad)


//write PID controller based off of error signal received from encoder

  //P term
  //calculate position error (rad)
  pos_error = desired_pos - current_pos ;

//  Serial.print("current position:      ");  Serial.println(current_pos); 
//  Serial.print("error:");   Serial.println(pos_error);


  //scaled positional error
//position scaling factor K_p = 100/(M_PI/2) found by taking 100 (100 being max pwm value I want to reach), and dividing by theoretical max absolute value of angle (Pi/2). This means with angles in that range, 100 will be the max PWM value outputted to the motor
  sp_error = (int) (K_p*pos_error);
  //Serial.print("SP error:");   Serial.println(sp_error);
  
//
//    if (sp_error > 0) {
//  digitalWrite(DIR, LOW);    //direction DIR LOW will turn the motor counterclockwise
//  }
//  else {
//  digitalWrite(DIR, HIGH);  //direction DIR HIGH will turn the motor clockwise
//  }
//
//  analogWrite(PWM_front, abs(sp_error)); 


 //D term
 //calculate velocity error
 
 unsigned long currentMicros = micros();
  current_vel = (((((x-x_offset)-oldPosition)*0.02197*1000000*M_PI/180.0)/(currentMicros-previousMicros)));   //Angular Speed(rad/s)

//  Serial.print("current velocity:     "); Serial.println(current_vel);

//try writing a PWM value to the motor that will resist the manual rotation of the wheel using the velocity.

////  
//  if (current_vel > 0) {
//    
//  digitalWrite(DIR, HIGH);    //if wheel is rotating in CCW direction, motor should spin CW to resist this motion
//  }
//  else {
//  digitalWrite(DIR, LOW);     //Conversely, if wheel is rotating in CW direction, motor should spin CCW to resist this motion
//  }
//
//    int resistance_velocity = (int) .5*current_vel;
//    analogWrite(PWM_front, resistance_velocity);
////  






  
  previousMicros = currentMicros;
  

  //the value of the velocity error will be negative of the current velocity (in order to resist current direction of motion). Calculated as target_velocity - current_velocity where target velocity is always 0
  
  //scaled velocity error
//  sv_error = (int) abs(K_d*current_vel)  ;
//
//  total_error = sp_error ;
//
//  if (total_error > 0) {
//  digitalWrite(DIR, LOW); 
//  }
//  else {
//  digitalWrite(DIR, HIGH);
//  }

//  analogWrite(PWM_front, abs(total_error));








  

oldPosition = x-x_offset;
//int pwm = 0;
//
// while (pwm < 180){
//    analogWrite(PWM_front, pwm);
//    pwm += 10;
//    delay(1000);
//    signed int x = REG_TC0_CV0;
//    signed int y = REG_TC0_CV1;
//    oldPosition = x-x_offset;
//    current_pos = (((x - x_offset) * 0.02197 * M_PI)/180); //Angle (rad)
//    unsigned long currentMicros = micros();
//    current_vel = (((((x-x_offset)-oldPosition)*0.02197*1000000*M_PI/180.0)/(currentMicros-previousMicros)));   //Angular Speed(rad/s)
//    Serial.println(String(pwm) + "  " + String(current_vel));
//    previousMicros = currentMicros;
//    oldPosition = x-x_offset;
//   }
//  analogWrite(PWM_front, pwm);
//  pwm += 10;
//  delay(1000);
//  
  Serial.print("(" + String(pwm)); 
  Serial.print(",");
  Serial.println(String(current_vel) + ")");
  

}
  //oldIndex = y;
  //delay(1000);
  

