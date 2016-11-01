#define in_pin 11 //hall sensor pulse
#define pwm_rear 8 //rear motor PWM pin 

float freq =0;
float T=0;
int state=0;
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(in_pin, INPUT);
pinMode(pwm_rear, OUTPUT);
analogWrite(pwm_rear, 180); //set PWM
attachInterrupt(digitalPinToInterrupt(in_pin), changeState, CHANGE); //Interrupt
}

void loop() {
T=pulseInLong(in_pin, HIGH)+pulseInLong(in_pin, LOW);
digitalWrite(in_pin, state);
Serial.print(state);
Serial.print(' ');
Serial.print(T);
Serial.print(' ');
freq=(1e6)/(T*28);
Serial.print(freq);
Serial.print('\n');
}

void changeState() {
  state = !state;
}
