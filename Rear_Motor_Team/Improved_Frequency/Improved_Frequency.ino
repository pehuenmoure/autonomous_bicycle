#define in_pin 11 //hall sensor pulse 
#define pwm_rear 8 //rear motor PWM pin
#define reverse_pin 50 //to change direction of rear motor
#define v_pin 63 //voltage reading

float pwm = 0; //current pwm value

double speed = 0; //speed in rev/s

boolean forward = true; //if False, is running in reverse

//Variables for calculating speed
float tOld = 0; //first time reading
float tNew = 0; //second time reading
double T = 0;




void setup() {

  Serial.begin(9600);
  pinMode(in_pin, INPUT);
  pinMode(pwm_rear, OUTPUT);
  pinMode(reverse_pin, OUTPUT);
  digitalWrite(reverse_pin, HIGH); //when high the motor goes forward


  float voltage = analogRead(63) / 14.2 * pwm / 180;
  analogWrite(pwm_rear, pwm);
  attachInterrupt(digitalPinToInterrupt(in_pin), getPeriod, RISING); //Interrupt
}

void loop() {
  rampToPWM(170);
  delay(5000);
  switchDirection();
}

/*
   Method that sets value "speed" to current speed in rev/s
*/
void getPeriod() {
  tOld = tNew;
  tNew = micros();
  T = (tNew - tOld);
  speed = (1E6) / (28 * T) ;
  if (speed < 100) {
    //Serial.println(speed, 3);
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
    analogWrite(pwm_rear, pwm);
  }
  while (pwm > newPWM) {
    if (pwm - newPWM < 10)
      pwm -= pwm - newPWM;
    else
      pwm -= 10;
    analogWrite(pwm_rear, pwm);
  }
}


/*
   Switches direction of motor
*/
void switchDirection() {
  Serial.println("entered method");
  if (forward) {
    digitalWrite(reverse_pin, LOW); //when low the motor goes reverse
    forward = false;
    Serial.println("set to reverse");
  } else {
    digitalWrite(reverse_pin, HIGH); //when high the motor goes forward
    forward = true;
    Serial.println("set to forward");
  }
}

