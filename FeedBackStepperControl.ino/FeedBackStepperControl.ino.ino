#include <I2C_Anything.h>

#include <Wire.h>
#include <DigitalIO.h>

#define pin_STEP 11
#define pin_DIR 12
#define pinA 2
#define pinB 3

volatile int counter;
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

  
volatile int counterA = 0;
volatile int counterB = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event

  fastPinMode(pinA, INPUT);
  fastPinMode(pinB, INPUT);

  fastPinMode(pin_STEP, OUTPUT);
  fastPinMode(pin_DIR, OUTPUT);
  
  attachInterrupt(0, pinA_ISR, RISING);
  attachInterrupt(1, pinB_ISR, RISING);
  counter = 0;
  //Sets stepper control pins to outputs
  digitalWrite(A4, LOW);
  digitalWrite(A5, LOW);
  distanceToTarget = targetEncoderPosition - counter;
  //  Serial.println(distanceToTarget);

  counterA = 0;
  counterB = 0;
}

void pinA_ISR(){
  //If the other pin is low, pin A is leading
  if (!fastDigitalRead(pinB)) {
    counter--; //increments the times that pin A leads
  }
  counterA++;
}



  void pinB_ISR(){
  if(!fastDigitalRead(pinA)){
    counter++; //Increments the times that pin B leads
  }
  counterB++;
  }



  void receiveEvent(int howMany) {
  I2C_readAnything(targetInDegrees);
  I2C_readAnything(rpm);
  //targetEncoderPosition = 400;
  }
  bool lastDirHigh = true;
void loop() {
  
  if(abs(counterA - counterB) > 2){
    digitalWrite(13, HIGH);
    delayMicroseconds(500);
    digitalWrite(13, LOW);
    counterA = counterB;
  }
  VESCrpm = (rpm - 127) / 18.1428;
  targetEncoderPosition = (int)(targetInDegrees * 1.111111);
  distanceToTarget = targetEncoderPosition - counter;
  
  if (distanceToTarget > 0) {
    fastDigitalWrite(pin_DIR, HIGH);
    lastDirHigh = true;
    if(!lastDirHigh){
    delayMicroseconds(50);
    }
  }
  else {
   fastDigitalWrite(pin_DIR, LOW);
   lastDirHigh = false;
   if(lastDirHigh){
   delayMicroseconds(50);
   }
  }
  
    if (abs(distanceToTarget) > 1) {

      fastDigitalWrite(pin_STEP, HIGH);
      delayMicroseconds(1500);
      fastDigitalWrite(pin_STEP, LOW);
    }
}
