 #define in_pin 11 //hall sensor pulse 
#define pwm_rear 8 //rear motor PWM pin
#define v_pin 63
float pwm = 90;
float tOld = 0;
float tNew = 0;
float T = 0;
float w = 0; //Measured Angular velocity in RPS
float desW = 2.5; //Desired Angular Velocity in RPS
int sampleLength = 25;
int count = 0;
int x = 30; // Used in ramp up
float gain = 2; 

void setup() {

  Serial.begin(9600);
  pinMode(in_pin, INPUT);
  pinMode(pwm_rear, OUTPUT);

  while (x < pwm) { //Ramps up speed- Workaround for rear motor safety features
    analogWrite(pwm_rear, x);
    delay(100);
    x = x + 1;
    Serial.print(x);
    Serial.print("\n");
  }
  float voltage = analogRead(63) / 14.2 * pwm / 180;
  Serial.print("Effective Voltage: ");
  Serial.print(voltage);
  Serial.print("\n");
  analogWrite(pwm_rear, pwm);
  attachInterrupt(digitalPinToInterrupt(in_pin), getT, RISING); //Interrupt
}

void loop() {
  pwm = (int)(gain * (desW - w) + pwm);
  Serial.print(w);
  Serial.print(" vs. Desired: ");
  Serial.print(desW);
  Serial.print(" plus change in pwm: ");
  Serial.print((int)(gain * (desW - w)));
  Serial.print('\n');
  analogWrite(pwm_rear, pwm);
}


void getT() {
  //Getting Average
  if (count != sampleLength) {
    count++;
  }
  else { // Count == sampleLength
    count = 0;
    tOld = tNew;
    tNew = micros();
    T = (tNew - tOld);
    w = (1E6) * sampleLength / (28 * T);
  }
}

int getPWM(float desiredVelocity) {//velocity in RPS
  //Maximum voltage of battery pack, in Volts- to be read from PIN 63: 14.2 is conversion factor
  float voltage = analogRead(63) / 14.2;

  int pwm = (int)(desiredVelocity + 2.1965) / 0.2224 * 180 / voltage;
  return pwm;
}

void setW(float desired) {
  desW = desired;
}
