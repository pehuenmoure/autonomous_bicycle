#define in_pin 11 //hall sensor pulse 
#define pwm_rear 8 //rear motor PWM pin
#define v_pin 63
float pwm = 60;
float tOld= 0;
float tNew= 0; 
float T=0;
float w= 0; 
int sampleLength = 100;
int count = 0; 
int minT = 11000;

//test count variable- Kenneth additions
int tc = 0;
int datasize = 300;


void setup() {
  Serial.begin(9600);
  pinMode(in_pin, INPUT);
  pinMode(63,INPUT);
  pinMode(pwm_rear, OUTPUT);
  analogWrite(pwm_rear, pwm);
  Serial.print(
  attachInterrupt(digitalPinToInterrupt(in_pin), getT, RISING); //Interrupt
}

void loop() {
  if(tc == datasize){
    tc = 0;
    pwm++;
    analogWrite(pwm_rear,pwm);
    Serial.print("Switching to pwm ");
    Serial.print(pwm);
    Serial.print('\n');
  }
  }


void getT() {
  tc++;
   if (count != sampleLength){
    count++;
    }
   else{
   count = 0; 
   tOld = tNew;
   tNew = micros();
   T= (tNew - tOld);
   w= 1E6/(28*T);
   Serial.print(w);
   Serial.print('\n');
  }
}
