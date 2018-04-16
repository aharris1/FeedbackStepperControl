#include <Wire.h>
#include <I2C_Anything.h>


 int posDeg = 0;

void setup() {
  Wire.begin();
  Serial.begin(9600);// join i2c bus (address optional for master)
}

void loop() {
  Serial.println("BEFORE");
  Wire.beginTransmission(8); // transmit to device #8
  I2C_writeAnything(posDeg);   // sends one byte
  Wire.endTransmission();    // stop transmitting
  Serial.println("AFTER"); 
  delay(4000);
  posDeg += 90;
  posDeg = posDeg % 360;
}
