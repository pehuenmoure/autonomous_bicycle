#define in_pin 11 //hall sensor pulse 
#define pwm_rear 8 //rear motor PWM pin
#define v_pin 63
float pwm = 165;
float tOld= 0;
float tNew= 0; 
float T=0;
float w= 0; 
int sampleLength = 100;
int count = 0; 

void setup() {
  Serial.begin(9600);
  pinMode(in_pin, INPUT);
  pinMode(pwm_rear, OUTPUT);
  analogWrite(pwm_rear, pwm);
  attachInterrupt(digitalPinToInterrupt(in_pin), getT, RISING); //Interrupt
}

void loop() {
  T= tNew - tOld;
  w= 1/(28*T)*1E6;
  Serial.println(w);
  }


void getT() {
  if (count != sampleLength){
    count++;
  }
  else{
   count = 0; 
    tOld = tNew;
    tNew = micros();
  }
 
}
