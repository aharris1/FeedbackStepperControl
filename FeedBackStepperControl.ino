#include "VescUart.h"
#include "datatypes.h"
#include "local_datatypes.h"


#include <I2C_Anything.h>

#include <Wire.h>
#include <DigitalIO.h>

#define pin_STEP 11
#define pin_DIR 12
#define pinA 2
#define pinB 3

PinIO readPinA(pinA);
PinIO readPinB(pinB);

int counter;
int stepper = 0;

bool runOnce = true;

unsigned long lastPulseMicros = 0;
unsigned long pulseDelayMicros = 2000;

long distanceToTarget = 0;
volatile int targetInDegrees = 360;
int targetEncoderPosition = 0;
volatile int rpm = 0;
int rollingAvg = 0;
int rollingAvgInterval = 5; //number of measurements for rolling average to span
float VESCrpm = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //Wire.begin(8);                // join i2c bus with address #8
  //Wire.onReceive(receiveEvent); // register event
  
  pinMode(pinA, INPUT);
  //pinMode(pinB, INPUT);
  attachInterrupt(0, pinA_ISR, RISING);
//  attachInterrupt(1, pinB_ISR, RISING);
  counter = 0;
  //Sets stepper control pins to outputs
  pinMode(pin_STEP, OUTPUT);
  pinMode(pin_DIR, OUTPUT);
  digitalWrite(A4, LOW);
  digitalWrite(A5, LOW);
  distanceToTarget = targetEncoderPosition - counter;
//  Serial.println(distanceToTarget);
}

void pinA_ISR(){
  //If the other pin is low, pin A is leading
  if(readPinB.read() == 0){
    counter--; //increments the times that pin A leads
  }
  else
  {
    counter++;
  }
}


/*
void pinB_ISR(){
  if(readPinA.read() == 0){
    counter++; //Increments the times that pin B leads
  }
  
}
*/

/*
void receiveEvent(int howMany) {
  sei();
  //I2C_readAnything(targetInDegrees);
  I2C_readAnything(rpm);
  //targetEncoderPosition = 400;
}
*/

void loop() {
  VESCrpm = (rpm - 127) / 18.1428; 
//  targetEncoderPosition = (int)(targetInDegrees * 1.111111);
  targetEncoderPosition = 400;
  distanceToTarget = targetEncoderPosition - counter;
  rollingAvg = rollingAvg * .99 + counter * .01;
  Serial.println(counter);
  /*
  if(stepper%10 == 0)
  {
  Serial.print("Current Position: ");
  Serial.println(counter);
  Serial.print("Destination: ");
  Serial.println(targetEncoderPosition);
  }
  */
//    VescUartSetCurrent(VESCrpm);

/*
if (Wire.available())
{
  I2C_readAnything(targetEncoderPosition);
}
*/
 
  if(counter >= 398 && counter <= 402){
    digitalWrite(13, HIGH);
  }
  else{
    digitalWrite(13, LOW);
  }
    
//    if(abs(distanceToTarget) > 7)
//    {
//      digitalWrite(13, HIGH);
//    }

      if(distanceToTarget > 0){
        digitalWrite(pin_DIR, HIGH);
      }
      else{
        digitalWrite(pin_DIR, LOW);
      }
    if(abs(distanceToTarget) > 1){

      digitalWrite(pin_STEP, HIGH);
      delayMicroseconds(1);
      digitalWrite(pin_STEP, LOW);
      delayMicroseconds(5000);
      stepper++;
      //digitalWrite(13, LOW);
    }
  
  
}
