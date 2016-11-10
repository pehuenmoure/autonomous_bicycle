#define in_pin 11 //hall sensor pulse 
#define pwm_rear 8 //rear motor PWM pin
#define v_pin 63
float pwm = 130;
float tOld= 0;
float tNew= 0; 
float T=0;
float w= 0; 
int sampleLength = 100;
int count = 0; 
int minT = 11000;
void setup() {
  Serial.begin(9600);
  pinMode(in_pin, INPUT);
  pinMode(pwm_rear, OUTPUT);
  analogWrite(pwm_rear, pwm);
  attachInterrupt(digitalPinToInterrupt(in_pin), getT, RISING); //Interrupt
}

void loop() {
  }


void getT() {
    int tOldtemp= 0;
    int tNewtemp= 0;
    int Ttemp = 0;
   tOldtemp = tNew;
   tNewtemp = micros();
   Ttemp= (tNewtemp - tOldtemp);
   if(Ttemp<minT){
    
   }
  else if (count != sampleLength){
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
 
}[
