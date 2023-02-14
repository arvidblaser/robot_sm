#include <Arduino.h>
#include <Servo.h>
#include <HardwareBLESerial.h>


/********* Pin definitions **********/
/* Board & BLE */
#define RED_LED 22   
#define GREEN_LED 23  
#define BLUE_LED 24     
#define PWR_LED 25  

HardwareBLESerial &bleSerial = HardwareBLESerial::getInstance();


/* Motor */
#define MOTOR_IN1 A2
#define MOTOR_IN2 A1

// Servo
Servo servo;
const byte servoPin = PIN_SERIAL_TX;

// Tof

// Rotation
const byte interruptPin = A0;

/********** Globals *************/
volatile byte red_led_state = LOW;
volatile byte green_led_state = LOW;

int pos = 0;    // variable to store the servo position
int counter = 0;

/****** Function declarations *****/
void count();

/******** Functions *********/
void setup() {
  /* debug, led, ble */
  Serial.begin(115200);
  Serial.println("Wobot test");

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  if (!bleSerial.beginAndSetupBLE("Woboten")) {
    while (true) {
      Serial.println("failed to initialize HardwareBLESerial!");
      delay(1000);
    }
  }

  /* rotaion encoder */
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), count, CHANGE); //LOW, HIGH, CHANGE, RISING, FALLING

  /* Motor */
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  /* Servo */
  servo.attach(servoPin);


  /* Testing Code Run once */
  // servo test
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      servo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      servo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }


  // motor test
  analogWrite(MOTOR_IN1, 0); //need to set PWM to 0 to stop it, digital write is not enough
  analogWrite(MOTOR_IN2, 200);
  delay(2000);

  analogWrite(MOTOR_IN2, 0);

}

void loop() {
  // put your main code here, to run repeatedly:
  red_led_state = !red_led_state;
  digitalWrite(RED_LED, red_led_state);

  bleSerial.poll();

  while (bleSerial.availableLines() > 0) {
    bleSerial.print((double)counter);
    bleSerial.print(" You said: ");
    char line[128]; 
    bleSerial.readLine(line, 128);
    bleSerial.println(line);
  }
  delay(500);

}

void count(){
  green_led_state = !green_led_state;
  digitalWrite(GREEN_LED, green_led_state);
  counter += 1;

}