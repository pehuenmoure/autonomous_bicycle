void landingGearDown(){
  digitalWrite(48,LOW); //sets relay pin 1 to High (turns light on)
  digitalWrite(47,LOW); //sets relay pin 2 to High  (turns light on)
}

void landingGearUp(){
  digitalWrite(48,HIGH); //Sets relay pin 1 to low (turns light off)
  digitalWrite(47,HIGH); //Sets relay pin 2 to low (turns light off)
}

void setup() {
  // initializes pins for the landing gear relay switch
  pinMode(48,OUTPUT);
  pinMode(47,OUTPUT);

//landing gear should stay down given no input,
//however it should be switched off when uploading code because
     //uploading code causes it to go up which is weird




void loop() {

}
