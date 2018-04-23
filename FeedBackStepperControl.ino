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

int counter;

unsigned long lastPulseMicros = 0;
unsigned long pulseDelayMicros = 2000;

long distanceToTarget = 0;
volatile int targetEncoderPosition = 199;
volatile int rpm = 0;
float VESCrpm = 0;
void setup() {
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  attachInterrupt(0, pinA_ISR, RISING);
  attachInterrupt(1, pinB_ISR, RISING);
  counter = 0;
  Serial.begin(9600);
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
  if(!DigitalPin.fastDigitalRead(pinB)){
    counter--; //increments the times that pin A leads
  }
}

void pinB_ISR(){
  if(!DigitalPin.fastDigitalRead(pinA)){
    counter++; //Increments the times that pin B leads
  }
  
}


void loop() {
  Serial.println(counter);
}
