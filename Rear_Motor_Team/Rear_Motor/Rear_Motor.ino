#define in_pin 11 //hall sensor pulse 
#define pwm_rear 8 //rear motor PWM pin
#define v_pin 63
float pwm = 95;
float tOld= 0;
float tNew= 0; 
float T=0;
float w= 0; 
int sampleLength = 10;
int count = 0; 
int x=30;
int minT = 0;
void setup() {
  
  Serial.begin(9600);
  pinMode(in_pin, INPUT);
  pinMode(pwm_rear, OUTPUT);
  while (x <pwm){ //Ramps up speed- Workaround for rear motor safety features
    analogWrite(pwm_rear, x);  
    delay(100);
    x=x+1;
  }
  float voltage = analogRead(63)/14.2*pwm/180;
  Serial.print("Effective Voltage: ");
  Serial.print(voltage);
  Serial.print("\n");
  //analogWrite(pwm_rear, pwm);
  attachInterrupt(digitalPinToInterrupt(in_pin), getT, RISING); //Interrupt
}

void loop() {
  }


void getT() {
   //Getting Average
  if (count != sampleLength){
    count++;
    }
  else{ // Count == sampleLength
   count = 0; 
   tOld = tNew;
   tNew = micros();
   T= (tNew - tOld);
   w= (1E6)*sampleLength/(28*T);
   Serial.print(w);
   Serial.print('\n');
  }
 
}
