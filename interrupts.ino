void Hall1Handler(){ //hall sensor 1 interrupt handler
  unsigned long currentMicros1=micros(); //get current microseconds 
  duration1 = int(currentMicros1 - previousMicros1); //see time since last measurement
  previousMicros1=currentMicros1;
  //Freq1[pointer1]=3*duration1; //change amount of magnets
  //int test=Freq1[pointer1];
 // pointer1++;
  //Serial.println("mamma malin");
}
void Hall2Handler(){ // hall sensor 2 interrupt handler
  unsigned long currentMicros2=micros();
  duration2 = currentMicros2-previousMicros2;
  previousMicros2=currentMicros2;
  Freq2[pointer2] =1/float(duration2); //change amount of magnets
  pointer2++;
}

void interruptSetup(){
  pinMode(6,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(6),Hall1Handler,FALLING); //add pin number
  //pinMode(6,INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(6),Hall2Handler,FALLING); //add pin number
}

