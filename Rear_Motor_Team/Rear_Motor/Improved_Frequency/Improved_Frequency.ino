#define in_pin 11 //hall sensor pulse 
#define pwm_rear 8 //rear motor PWM pin
#define v_pin 63 //voltage reading

float pwm = 70;
float tOld = 0;
float tNew = 0;
double T = 0;
double w = 0;
int sampleLength = 1;
int count = 0;
int x = 30;
int minT = 0;
float vals[28];
int tc = 0; //Counts number of data points collected on this PWM
int ds = 20; //Number of data points to collect for each PWM

int newPWM; //Storage place for new pwms


int i = 0;
float avgsum = 0;
int avg = 0;

void setup() {

  Serial.begin(9600);
  pinMode(in_pin, INPUT);
  pinMode(pwm_rear, OUTPUT);

  while (x < pwm) { //Ramps up speed- Workaround for rear motor safety features
    analogWrite(pwm_rear, x);
    delay(100);
    x = x + 10;
  }
  float voltage = analogRead(63) / 14.2 * pwm / 180;
  
  analogWrite(pwm_rear, pwm);
  attachInterrupt(digitalPinToInterrupt(in_pin), getT, RISING); //Interrupt
}

void loop() {
  if (Serial.available()>0) {
    newPWM = Serial.read();
    pwm = newPWM;
    Serial.write((int)pwm);
  }

  if (tc == ds && 1 != 1) {
    tc = 0;

    float voltage = analogRead(63) / 14.2 * pwm / 180;

    pwm += 10;
    analogWrite(pwm_rear, pwm);
  }
}


void getT() {

  tOld = tNew;
  tNew = micros();
  T = (tNew - tOld);
  w = (1E6) / (28 * T) ;

}

