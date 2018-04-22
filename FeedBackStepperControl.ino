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
#define STEPS_PER_REV 400
#define STEP_TOLERANCE 6
#define STEP_DELAY 500
PinIO readPinA(pinA);
PinIO readPinB(pinB);

int counter;

bool turnRight = true; //Initialized with arbitrary value

unsigned long lastPulseMicros = 0;
unsigned long pulseDelayMicros = 2000;

long distanceToTarget = 0;
volatile int targetEncoderPosition = 199;
volatile int rpm = 0;
float VESCrpm = 0;
void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
//  Serial.begin(9600);

  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  attachInterrupt(0, pinA_ISR, RISING);
  attachInterrupt(1, pinB_ISR, RISING);
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
}

void pinB_ISR(){
  if(readPinA.read() == 0){
    counter++; //Increments the times that pin B leads
  }
  
}



void receiveEvent(int howMany) {
  I2C_readAnything(targetEncoderPosition);
  I2C_readAnything(rpm);
  VESCrpm = (rpm - 127) / 18.1428; 
  targetEncoderPosition = ((int)(targetEncoderPosition * STEPS_PER_REV / 360.0))%STEPS_PER_REV;
}


void loop() {
  distanceToTarget = targetEncoderPosition - (counter%STEPS_PER_REV);
  //Sets turnRight based on the raw difference  
  if(distanceToTarget > STEP_TOLERANCE){
    turnRight = true;
  }
  else if(distanceToTarget < -1* STEP_TOLERANCE){
    turnRight = false;
  }
  
  //Inverts turn right if it would be easier to turn the other way
  if(abs(distanceToTarget) > 180){
    turnRight = !turnRight;
    //TODO: VESCrpm = -VESCrpm;
  }
  //Sets the direction pin according to the direction required
  if(turnRight){
    digitalWrite(pin_DIR, HIGH);
  }
  else{
    digitalWrite(pin_DIR, LOW);
  }
//  Serial.println(targetEncoderPosition);
  // put your main code here, to run repeatedly:
  if(micros()-lastPulseMicros > STEP_DELAY)
  {
    //Steps when it has been at least as long as the step delay
    if(abs(distanceToTarget) > STEP_TOLERANCE){
      digitalWrite(pin_STEP, HIGH);
      delayMicroseconds(1);
      digitalWrite(pin_DIR, LOW);
      lastPulseMicros=micros();
    }
  }
  
}
