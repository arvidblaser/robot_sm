#include <Arduino.h>

#include <HardwareBLESerial.h>

 #define RED 22     
 #define BLUE 24     
 #define GREEN 23
 //#define LED_PWR 25

HardwareBLESerial &bleSerial = HardwareBLESerial::getInstance();


void setup() {
  // put your setup code here, to run once:
   // initialize the digital Pin as an output
  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(LED_PWR, OUTPUT);

    Serial.begin(9600);

    //Initialize BLE
  if (!bleSerial.beginAndSetupBLE("Roboten")) {
    while (true) {
      Serial.println("failed to initialize HardwareBLESerial!");
      delay(1000);
    }
  }
}

void loop() {
  bleSerial.poll();

  
  while (bleSerial.availableLines() > 0) {
    bleSerial.print("You said: ");
    char line[128]; bleSerial.readLine(line, 128);
    bleSerial.println(line);
  }
  delay(500);
  // put your main code here, to run repeatedly:

  /*
    bool running = true;
  String go = "go";
  String stop = "stop";
  char ble_input[128];
  
  bleSerial.poll();

  while (bleSerial.availableLines() > 0) {
    bleSerial.readLine(ble_input, 128);
    Serial.println("line available");

  }

  if (ble_input == go) {
    running = true;
          Serial.println("go");

  } else if (ble_input == stop) {
    running = false;
          Serial.println("stop");

  }

  if (running) {
        delay(200);

      digitalWrite(RED, LOW); // turn the LED off by making the voltage LOW
        digitalWrite(GREEN, HIGH);
              Serial.println("green");

  }
  else {
      digitalWrite(RED, HIGH); // turn the LED off by making the voltage LOW
      digitalWrite(GREEN, LOW);
            Serial.println("fred");

  }*/
}