#define pin_STEP 6
#define pin_DIR 7

void setup() {
  // put your setup code here, to run once:

  //Sets stepper control pins to outputs
  pinMode(pin_STEP, OUTPUT);
  pinMode(pin_DIR, OUTPUT);
}

void loop() {
  stepMotor(100, true);
  // put your main code here, to run repeatedly:

}

void stepMotor(int steps, boolean dir){
  if(dir){ //Sets direction  
  digitalWrite(pin_DIR, HIGH); 
  }
  else{
    digitalWrite(pin_DIR, LOW);
  }
  for(int i = 0 ; i < steps; i++){ //Pulses for prescribed number of steps
    digitalWrite(pin_STEP, HIGH);
    digitalWrite(pin_STEP, LOW);
    delay(1);
  }
}

