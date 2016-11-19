#define in_pin 11 //hall sensor pulse
#define Sample_length 10 //what is this? 
#define pwm_rear 9

int num1;
int num2;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(in_pin, INPUT);
pinMode(pwm_rear, OUTPUT);
//analogWrite(pwm_rear, 170);
attachInterrupt(digitalPinToInterrupt(in_pin), changeState, CHANGE);
}

void loop() {
digitalWrite(in_pin, state);
Serial.print(state);
}

void changeState() {
  state = !state;
}
