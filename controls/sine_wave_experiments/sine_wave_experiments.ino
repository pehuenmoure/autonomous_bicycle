
float currentMicros ;
float previousMicros;



void setup() {
  // put your setup code here, to run once:







}

void loop() {


   unsigned long currentMicros = micros();
   float t = micros()*1000000;

   Serial.print("t:");   Serial.println(t);


   float desired_pos = 1*sin(0.01*t) ;

   float delta_t = currentMicros-previousMicros ;
   
   Serial.print("delta_t:");   Serial.println(delta_t);
   
    previousMicros = currentMicros;
   
  // put your main code here, to run repeatedly:

}
