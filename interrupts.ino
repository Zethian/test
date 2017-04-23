volatile int line1=0;
volatile int line2=0;
volatile int line3=0;
volatile int line4=0;
volatile int line5=0;
volatile int line6=0;
volatile int line7=0;

void HallHandler(){ //hall sensor 1 interrupt handler
  unsigned long currentMicros=micros(); //get current microseconds 
  duration = int(currentMicros - previousMicros); //see time since last measurement
  previousMicros=currentMicros;
}


void interruptSetup(){
  /* Hall sensor interrupts */
  pinMode(9,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(9),HallHandler,FALLING); /*Interrupt triggers when the hall sensor passes a magnet*/

  /* Line sensor array interrupts */
  pinMode(1,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(1),Line1Int,CHANGE); /* Interrupts triggers when moving from high/low to the opposite */
  pinMode(3,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3),Line2Int,CHANGE);
  pinMode(4,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(4),Line3Int,CHANGE);
  pinMode(5,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(5),Line4Int,CHANGE);
  pinMode(6,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(6),Line5Int,CHANGE);
  pinMode(7,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(7),Line6Int,CHANGE);
  pinMode(8,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(8),Line7Int,CHANGE);
}

void Line1Int(){
  int temp=digitalRead(1); /*reads value from sensor */
  if(temp==HIGH){ /* Gives us workable value from status of sensor */
    line1=0;
  }
  else if(temp==LOW){
    line1=1;
  }
}

void Line2Int(){
  int temp=digitalRead(3);
  if(temp==HIGH){
    line2=0;
  }
  else if(temp==LOW){
    line2=1;
  }
}

void Line3Int(){
  int temp=digitalRead(4);
  if(temp==HIGH){
    line3=0;
  }
  else if(temp==LOW){
    line3=1;
  }
}

void Line4Int(){
  int temp=digitalRead(5);
  if(temp==HIGH){
    line4=0;
  }
  else if(temp==LOW){
    line4=1;
  }
}

void Line5Int(){
  int temp=digitalRead(6);
  if(temp==HIGH){
    line5=0;
  }
  else if(temp==LOW){
    line5=1;
  }
}

void Line6Int(){
  int temp=digitalRead(7);
  if(temp==HIGH){
    line6=0;
  }
  else if(temp==LOW){
    line6=1;
  }
}

void Line7Int(){
  int temp=digitalRead(8);
  if(temp==HIGH){
    line7=0;
  }
  else if(temp==LOW){
    line7=1;
  }
}



