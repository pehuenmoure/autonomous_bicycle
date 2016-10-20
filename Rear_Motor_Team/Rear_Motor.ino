#define in_pin 11 //hall sensor pulse
#define Sample_length 10
#define pwm_rear 9

float freq =0;
float T=0;
int state=0;
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(in_pin, INPUT);
pinMode(pwm_rear, OUTPUT);
//analogWrite(pwm_rear, 170);
attachInterrupt(digitalPinToInterrupt(in_pin), changeState, CHANGE);
}

void loop() {
T=pulseInLong(in_pin, HIGH);
//state=digitalRead(in_pin);
digitalWrite(in_pin, state);
Serial.print(state);
Serial.print(' ');


Serial.print(T);
Serial.print(' ');
freq=10e6/(T*56);
Serial.print(freq);
Serial.print('\n');
}

void changeState() {
  state = !state;
}
