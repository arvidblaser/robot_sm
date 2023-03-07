#include <Arduino.h>
#include <Servo.h>
#include <HardwareBLESerial.h>
#include <VL53L1X.h>



/********* Pin definitions **********/
/* Board & BLE */
#define RED_LED 22   
#define GREEN_LED 23  
#define BLUE_LED 24     
#define PWR_LED 25  

HardwareBLESerial &bleSerial = HardwareBLESerial::getInstance();

char line[128]; //rxed ble data



/* Motor */
#define MOTOR_IN1 A2
#define MOTOR_IN2 A1

// Servo
Servo servo;
#define servoPin PIN_SERIAL_TX

// Tof
VL53L1X sensor_1;
VL53L1X sensor_3;
VL53L1X sensor_5;

#define reset_1  10
//#define reset_2  7 // not used yet
#define reset_3  4
//#define reset_2  2 // not used yet
#define reset_5  PIN_SERIAL_RX



// Rotation
#define interruptPin A0

/********** Globals *************/
volatile byte red_led_state = LOW;
volatile byte green_led_state = LOW;

int pos = 0;    // variable to store the servo position
int counter = 0;

// pid
int GLOBAL_E_N = 0;

int left = 0;
int right = 0;
int servo_val = 0;

/****** Function declarations *****/
void count();
void stop_wobot();
void crash_forward(int speed);
void crash_backward(int speed);

bool waiting_for_ble_cmd(const char* text);

void print_sensor_data();
int pid(int y_t);
/******** Functions *********/
void setup() {
  /* debug, led, ble */
  Serial.begin(115200);
  Serial.println("Wobot test");

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  if (!bleSerial.beginAndSetupBLE("Woboten reserv")) {
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

  /* ToF */
  pinMode(reset_1,OUTPUT);
  pinMode(reset_3, OUTPUT);
  pinMode(reset_5, OUTPUT);

  digitalWrite(reset_1, LOW);
  digitalWrite(reset_3, LOW);
  digitalWrite(reset_5, LOW);  

  delay(500);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  pinMode(reset_1, INPUT);
  delay(1500);
  sensor_1.setTimeout(500);
  if (!sensor_1.init())
  {
    Serial.println("Failed to detect and initialize sensor 1!");
    while (1);
  }
  delay(1000);
  sensor_1.setAddress((uint8_t)30);
  Serial.println("Sensor 1 initialized");

  Serial.println("addresses set");
  Serial.print("Address 1: ");
  Serial.println(sensor_1.getAddress());

  sensor_1.setDistanceMode(VL53L1X::Long);
  sensor_1.setMeasurementTimingBudget(50000);

  sensor_1.startContinuous(500);

  pinMode(reset_3, INPUT);
  delay(1500);
  sensor_3.setTimeout(500);
  if (!sensor_3.init())
  {
    Serial.println("Failed to detect and initialize sensor 2!");
    while (1);
  }
  delay(1000);
  sensor_3.setAddress((uint8_t)32);
  Serial.println("Sensor 2 initialized");

  pinMode(reset_5, INPUT);
  delay(1500);
  sensor_5.setTimeout(500);
    if (!sensor_5.init())
  {
    Serial.println("Failed to detect and initialize sensor 3!");
    while (1);
  }
  delay(1000);
  sensor_5.setAddress((uint8_t)34);
  Serial.println("Sensor 3 initialized");
  
  Serial.println("addresses set");
  Serial.print("Address 1: ");
  Serial.println(sensor_1.getAddress());
  Serial.print("Address 3: ");
  Serial.println(sensor_3.getAddress());
  Serial.print("Address 5: ");
  Serial.println(sensor_5.getAddress());
  
  sensor_1.setDistanceMode(VL53L1X::Long);
  sensor_1.setMeasurementTimingBudget(50000);
  
	sensor_3.setDistanceMode(VL53L1X::Long);
  sensor_3.setMeasurementTimingBudget(50000);
  
  sensor_5.setDistanceMode(VL53L1X::Long);
  sensor_5.setMeasurementTimingBudget(50000);

  sensor_1.startContinuous(500);
  sensor_3.startContinuous(500);
  sensor_5.startContinuous(500);


  /* Other stuff*/
  while(waiting_for_ble_cmd("test")){
    print_sensor_data();
    left = sensor_1.read();
    right = sensor_5.read();
    bleSerial.println((uint64_t)(left));
    bleSerial.println((uint64_t)(sensor_3.read()));
    bleSerial.println((uint64_t)(right));

    Serial.print("Difference left - right = ");
    Serial.println(left-right);

    servo_val = pid(left-right);
    Serial.print("servo val: ");
    Serial.println(servo_val);
    delay(500);
  }



}

void loop() {
  // put your main code here, to run repeatedly:
  red_led_state = !red_led_state;
  digitalWrite(RED_LED, red_led_state);


  /* Waiting for start */
  while(waiting_for_ble_cmd("go"));

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


  /* Run motor until stop */ 
  
  crash_forward(200);
  while (waiting_for_ble_cmd("end")){
    print_sensor_data();
    left = sensor_1.read();
    right = sensor_5.read();
    bleSerial.println((uint64_t)(left));
    bleSerial.println((uint64_t)(sensor_3.read()));
    bleSerial.println((uint64_t)(right));


    Serial.print("Difference left - right = ");
    Serial.println(left-right);
    pid(left-right);
    delay(500);
  }
  
  stop_wobot();
}

void count(){
  green_led_state = !green_led_state;
  digitalWrite(GREEN_LED, green_led_state);
  counter += 1;
}

void stop_wobot(){
  analogWrite(MOTOR_IN1, 0);
  analogWrite(MOTOR_IN2, 0);
}

void crash_forward(int speed){
  analogWrite(MOTOR_IN1, speed);
  analogWrite(MOTOR_IN2, 0);
}

void crash_backward(int speed){
  analogWrite(MOTOR_IN1, 0);
  analogWrite(MOTOR_IN2, speed);
}

bool waiting_for_ble_cmd(const char* text){

  //bool waiting_for_cmd = true;
  //while(waiting_for_cmd){
  bleSerial.print("Send '");
  bleSerial.print(text); 
  bleSerial.print("' to continue \n");
  delay(1000);

  bleSerial.poll();
  while (bleSerial.availableLines() > 0){
    bleSerial.readLine(line, 128);
    if(strcmp(line, text) == 0){
      //waiting_for_cmd = false;
      return false;
    }  
  }
  return true;
  //}
}


void print_sensor_data(){
    Serial.print(sensor_1.read());
    Serial.println("  (first sensor)"); 
    if (sensor_1.timeoutOccurred()) { Serial.print(" 1. TIMEOUT"); }

    Serial.print(sensor_3.read());
    Serial.println(" (second sensor)");
    if (sensor_3.timeoutOccurred()) { Serial.print(" 2. TIMEOUT"); }

    Serial.print(sensor_5.read());
    Serial.println(" (third sensor)");
    if (sensor_5.timeoutOccurred()) { Serial.print("3.  TIMEOUT"); }

    Serial.println();
}

int pid(int y_t){
  int e_n_last = GLOBAL_E_N;
  int r_t = 0, u_t = 0, e_t =0;
  double kp=1, ki =0, kd = 1;
  int t = 0;

  e_t = r_t-y_t;
  GLOBAL_E_N = e_t;

  u_t = kp*e_t+kd*(e_t-e_n_last)/t;

  //typical values
  //y(t) = diff betwen tof1 & tof5
  // in range 

  u_t = u_t+90;
  if(u_t>180){
    u_t=180;
  }
  else if(u_t < 0){
    u_t=0;
  }
  return(u_t);
}