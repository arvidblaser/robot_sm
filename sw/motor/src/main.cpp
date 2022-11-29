#include <Arduino.h>
// Basic sketch for trying out the Adafruit DRV8871 Breakout

#define MOTOR_IN1 9
#define MOTOR_IN2 10

void setup() {
  Serial.begin(115200);

  Serial.println("DRV8871 test");
  
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
}

void loop() {

  // ramp up forward
  //i=0 0%, i= 255=100% pwm
  analogWrite(MOTOR_IN1, 0); //need to set PWM to 0 to stop it, digital write is not enough
  digitalWrite(MOTOR_IN1, LOW);
  for (int i=100; i<175; i++) {
    analogWrite(MOTOR_IN2, i);
    delay(100);
  }

  // forward full speed for one second
  //delay(1000);
  
  // ramp down forward
  for (int i=175; i>=100; i--) {
    analogWrite(MOTOR_IN2, i);
    delay(100);
  }
  analogWrite(MOTOR_IN2, 0);
  // ramp up backward
  digitalWrite(MOTOR_IN2, LOW);
  for (int i=100; i<175; i++) {
    analogWrite(MOTOR_IN1, i);
    delay(100);
  }

  // backward full speed for one second
  //delay(1000);

  // ramp down backward
  for (int i=175; i>=100; i--) {
    analogWrite(MOTOR_IN1, i);
    delay(100);
  }
}