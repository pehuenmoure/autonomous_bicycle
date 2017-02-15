#include "RearMotor.h"

/*
   Method that sets value "speed" to current speed in rev/s
*/
void getPeriod() {
  tOld = tNew;
  tNew = micros();
  T = (tNew - tOld);
  speed = (1.2446)*(1E6) / (28 * T) ;
  if (speed < 100) {
    Serial.println(speed, 3);
  }
}


/*
   Method for setting rear motor at a certain PWM

   @param newPWM- the new pwm value to ramp up to
*/
void rampToPWM(float newPWM) {

  while (rear_pwm < newPWM) { //Ramps up speed- Workaround for rear motor safety features
    if (newPWM - rear_pwm < 10)
      pwm += newPWM - pwm;
    else
      pwm += 10;
    analogWrite(pwm_rear, rear_pwm);
  }
  while (rear_pwm > newPWM) {
    if (rear_pwm - newPWM < 10)
      rear_pwm -= rear_pwm - newPWM;
    else
      pwm -= 10;
    analogWrite(pwm_rear, rear_pwm);
  }
}


/*
   Switches direction of motor
*/
void switchDirection() {
 // Serial.println("entered method");
  if (forward) {
    digitalWrite(reverse_pin, LOW); //when low the motor goes reverse
    forward = false;
    //Serial.println("set to reverse");
  } else {
    digitalWrite(reverse_pin, HIGH); //when high the motor goes forward
    forward = true;
   // Serial.println("set to forward");
  }
}

