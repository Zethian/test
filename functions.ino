#include <Servo.h>
#include <Math.h>
#include <PID_v1.h>
#include "interrupts.h"

#define radius 0.032 
#define pi 3.1415

Servo motor;
Servo wheels;

volatile unsigned int duration=0;
volatile unsigned long previousMicros=0;

volatile long Freq=0;
volatile float fault=0;

void functionSetup(){
  Serial.begin(19200);
  if(motor.attached()==false){
    motor.attach(9,1350,2000); // motor is attached to pin 9
  }
  if(wheels.attached()==false){
    wheels.attach(8,700,2000); // servo steering is attached to pin 8
  }
  motor.writeMicroseconds(1350); //set calibration, 0 position, wait 2.5 seconds to allow the calibration to pass
  wheels.write(90);
  delay(2500);

}

void SetSteering(float angle){ //60 to 120 degrees
  int setangle=round(angle+90);
  if(setangle<40){ //to avoid too high angle on the steering, set measured max and min values
    setangle=40;
  }
  else if(setangle>150){
    setangle=150;
  }
  wheels.write(setangle); //set angle to wheels
  
}

void SetSpeed(float mps){ //must be above 8% if it has had speed earlier, otherwise 8
  int speed;
  float mpsmax=50;
  speed=round(((mps/mpsmax)*650)+1350); //get speed as a microsecond value
  if(speed<1350){ //set max and min values to prevent burnout and underneath valid value
    speed=1350;
  }
  else if(speed>2000){
    speed=2000;
  }
  motor.writeMicroseconds(speed); //set speed to motor
}

float fetchFreq(){
      Freq = 1e6/(3*(duration));
      if((micros()-previousMicros)>=1000000){
        Freq=1e6/(3*(micros()-previousMicros));
      }
      return Freq;
}

float getSpeed(){
  float velocity;
  velocity = fetchFreq()*radius*2*pi;
  return velocity;
}

float getFault(){
  if(line1+line2+line3+line4+line5+line6+line7 == 0){
    return fault;
  }
  else{
   fault=((-7.8*line1)+(-5.2*line2)+(-2.6*line3)+(0*line4)+(2.6*line5)+(5.2*line6)+(7.8*line7))/(line1+line2+line3+line4+line5+line6+line7);
   return fault;
  }
}

