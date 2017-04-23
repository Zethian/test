#include "functions.h"
#include "interrupts.h"
#include <PID_v1.h>
 double inVelo=0;//getSpeed();
 double refVelo=4.5;
 double outVelo=0;
 double inAngle=0;
 double outAngle=0;
 double refAngle=0;
 #define kp2 1
 #define ki2 200
 #define kd2 0
 #define kp1 0.1
 #define ki1 4
 #define kd1 0
 
 PID myPID(&inVelo, &outVelo, &refVelo,kp2,ki2,kd2,DIRECT);
 PID steering(&inAngle,&outAngle,&refAngle,kp1,ki1,kd1,DIRECT);
void setup() {
  // put your setup code here, to run once:
  functionSetup();
  interruptSetup();

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0,20);
  myPID.SetSampleTime(10);
  steering.SetMode(AUTOMATIC);
  steering.SetOutputLimits(-50,50);
  steering.SetSampleTime(1);
}

void loop() {
  /*SetSpeed(8);
  delay(2000);
  Serial.println(getSpeed());
  SetSpeed(10);
  delay(2000);*/
  Serial.println(getSpeed());
  /*Set Steering angle */
  inAngle=getFault();
  refVelo=5*(1.2-(abs(inAngle)/7.8));
  steering.Compute();
  SetSteering(outAngle);
  /*Set Speed from PID */
  inVelo=getSpeed();
  myPID.Compute();
  //Serial.println(outVelo);
  SetSpeed(outVelo);
  delay(500);
  
}
