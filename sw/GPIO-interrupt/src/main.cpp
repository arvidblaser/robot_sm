#include <Arduino.h>

void blink();
//const byte ledPin = 13;
#define RED 22     

const byte interruptPin = A3;
volatile byte state = LOW;

void setup() {
  pinMode(RED, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE); //LOW, HIGH, CHANGE, RISING, FALLING
}

void loop() {
  digitalWrite(RED, state);
}

void blink() {
  state = !state;
}