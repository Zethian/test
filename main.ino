#include "functions.h"
#include "interrupts.h"
#include <PID_v1.h>
 double inVelo=0;//getSpeed();
 double refVelo=4.5;
 double outVelo=0;
 #define kp 1
 #define ki 200
 #define kd 0
 PID myPID(&inVelo, &outVelo, &refVelo,kp,ki,kd,DIRECT);
void setup() {
  // put your setup code here, to run once:
  functionSetup();
  interruptSetup();

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0,20);
  myPID.SetSampleTime(10);
}

void loop() {
  // put your main code here, to run repeatedly:
  /*SetSpeed(8);
  delay(2000);
  Serial.println(getSpeed());
  SetSpeed(10);
  delay(2000);*/
  Serial.println(getSpeed());
  inVelo=getSpeed();
  myPID.Compute();
  //Serial.println(outVelo);
  SetSpeed(outVelo);
  delay(500);
  
}
