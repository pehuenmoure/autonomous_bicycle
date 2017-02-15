#include "RearMotor.h"
/*
   Method that sets value "speed" to current speed in rev/s
*/
void getPeriod() {
  tOld = tNew;
  tNew = micros();
  T = (tNew - tOld);
  speed = (1E6) / (28 * T) ;
  if (speed < 100) {
    Serial.println(speed, 3);
  }
}

/*
   Method for setting rear motor at a certain PWM

   @param newPWM- the new pwm value to ramp up to
*/
void rampToPWM(float newPWM) {

  while (pwm < newPWM) { //Ramps up speed- Workaround for rear motor safety features
    if (newPWM - pwm < 10)
      pwm += newPWM - pwm;
    else
      pwm += 10;
    analogWrite(PWM_rear, pwm);
  }
  while (pwm > newPWM) {
    if (pwm - newPWM < 10)
      pwm -= pwm - newPWM;
    else
      pwm -= 10;
    analogWrite(PWM_rear, pwm);
  }
}

/*
   Switches direction of motor
*/
void switchDirection() {
  if (forward) {
    digitalWrite(reverse_pin, LOW); //when low the motor goes reverse
    forward = false;
  } else {
    digitalWrite(reverse_pin, HIGH); //when high the motor goes forward
    forward = true;
  }
}
