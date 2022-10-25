#include <Arduino.h>
/*
Optical switch:
The IR-diode is using a 200 Ohm resistor
The phototransistor is pulled up by 10k, 
note that the collector is parallell to the cathode
The Voltage is measured at collector/resistor.
*/
//Threshold value 200 = 0.7 V / (3.3 V/1024)
#define THRESHOLD 200

#define RED 22     

const int analogPin = A3; 
int val = 0;  // variable to store the value read


void setup() {
    pinMode(RED, OUTPUT);
}

void loop() {
  
  val = analogRead(analogPin);  // read the input pin
  if(val>THRESHOLD){
      digitalWrite(RED, HIGH); // turn the LED on by making the voltage HIGH
  }
  else{
    digitalWrite(RED, LOW); // turn the LED off by making the voltage LOW
  }
}

