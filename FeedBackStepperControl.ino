#include <I2C_Anything.h>

#include <Wire.h>
#include <DigitalPin.h>

#define pin_STEP 11
#define pin_DIR 12
#define pinA 2
#define pinB 3

#define SEEK_TOLERANCE 0

int counter = 0;

int counterA = 0;
int counterB = 0;


unsigned long lastPulseMicros = 0;
unsigned long pulseDelayMicros = 2000;

long distanceToTarget = 0;
volatile int targetEncoderPosition = 400;
volatile int rpm = 0;
float VESCrpm = 0;
void setup() {
  fastPinMode(pinA, INPUT);
  fastPinMode(pinB, INPUT);
  attachInterrupt(0, pinA_ISR, RISING);
  attachInterrupt(1, pinB_ISR, RISING);
  counter = 0;
  //Serial.begin(9600);
  //Sets stepper control pins to outputs
  pinMode(pin_STEP, OUTPUT);
  pinMode(pin_DIR, OUTPUT);
  digitalWrite(A4, LOW);
  digitalWrite(A5, LOW);
  digitalWrite(13, HIGH);
  digitalWrite(13, LOW);
  distanceToTarget = targetEncoderPosition - counter;
//  Serial.println(distanceToTarget);
}

void pinA_ISR(){
if(!fastDigitalRead(pinB)){
  counterA--;  
}
else{
  counterA++;
}
}

void pinB_ISR(){
if(!fastDigitalRead(pinA)){
counterB++;
  }
  else{
    counterB--;
  }
}

int stepCounter = 0;

void loop() {
//  if(stepCounter < 200){
//    digitalWrite(pin_STEP, HIGH);
//    delayMicroseconds(250);
//    digitalWrite(pin_STEP, LOW);
//    stepCounter++;
//    delay(10);
//  }
//  Serial.println(counter);
  
//  DEBUG: prints distance to target
//  if(counterA % 10 == 0){
//  Serial.print(counterA);
//  Serial.print(',');
//  Serial.println(counterB);
//  }
//  Sets the direction each loop
  if(distanceToTarget > 0){
    digitalWrite(pin_DIR, HIGH);
    digitalWrite(13, HIGH);
  }
  else{
    digitalWrite(pin_DIR, LOW);
    digitalWrite(13, LOW);
  }

  if(abs(distanceToTarget > SEEK_TOLERANCE)){
      digitalWrite(pin_STEP, HIGH);
      delayMicroseconds(250);
      digitalWrite(pin_STEP, LOW);
  }

  //Recalculates distance to target
  distanceToTarget = targetEncoderPosition - max(counterA, counterB);
  //delayMicroseconds(1000);
  delay(1);
}
