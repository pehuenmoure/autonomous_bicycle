#define relay1 48
#define relay2 47

#define RC_CH5 27     //Landing Gear (CH6 is for rear motor control)

volatile unsigned long timer_start5;  //micros when the pin goes HIGH
volatile int last_interrupt_time5; //calcSignal is the interrupt handler
volatile float pulse_time5 ; 

void calcSignal5() //landing gear control
{
  //record the interrupt time so that we can tell if the receiver has a signal from the transmitter
  last_interrupt_time5 = micros();
  //if the pin has gone HIGH, record the microseconds since the Arduino started up
  if (digitalRead(RC_CH5) == HIGH)
  {
    timer_start5 = micros();
  }
  //otherwise, the pin has gone LOW
  else
  {
    //only worry about this if the timer has actually started
    if (timer_start5 != 0 && ((volatile int)micros() - timer_start5 > 1000) && ((volatile int)micros() - timer_start5 < 2000) ) // filter out noise
    {
      //record the pulse time
      pulse_time5 = ((volatile int)micros() - timer_start5); //pulse time is the output from the rc value that we need to transform into a pwm value
      //restart the timer
      timer_start5 = 0;

        if (pulse_time5 > 1000 && pulse_time5 < 1200){
          landingGearDown();
        }
        else if (pulse_time5 > 1800 && pulse_time5 < 2000){
        landingGearUp();
        }
    }
  }
}


void landingGearDown() {
  digitalWrite(47, LOW); //sets relay pin 1 to High (turns light on) 
  digitalWrite(48, LOW); //sets relay pin 2 to High  (turns light on)
 // digitalWrite(orangeLED, LOW);
}
void landingGearUp() {
 // digitalWrite(orangeLED, HIGH);
  digitalWrite(47, HIGH); //Sets relay pin 1 to low (turns light off)
  digitalWrite(48, HIGH); //Sets relay pin 2 to low (turns light off)
}

//////////THIS BLOCK LOOKS IMPORTANT BUT IS PROBABLY ALREADY IN THE CODE//////////////////
      //timers for each channel
      int duration_CH1, duration_CH2, duration_CH3, duration_CH4, duration_CH5, duration_CH6;
      int start_CH1, start_CH2, start_CH3, start_CH4, start_CH5, start_CH6;
      int end_CH1, end_CH2, end_CH3, end_CH4, end_CH5, end_CH6;
      //current cycle's logic
      boolean CH1, CH2, CH3, CH4, CH5, CH6;
///////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
    timer_start5 = 0;
    attachInterrupt(RC_CH5, calcSignal5, CHANGE);

    pinMode(relay1, OUTPUT);
    pinMode(relay2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

}
