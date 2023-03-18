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
VL53L1X sensor[3];

#define reset_1  10
//#define reset_2  7 // not used yet
#define reset_3  4
//#define reset_2  2 // not used yet
#define reset_5  PIN_SERIAL_RX

uint8_t sensor_pins[] = {reset_1,reset_3,reset_5};
uint8_t sensor_address[] = {30, 32, 34};

// Rotation
#define interruptPin A0

/********** Globals *************/
volatile byte red_led_state = LOW;
volatile byte green_led_state = LOW;

int pos = 0;    // variable to store the servo position
int counter = 0;

// pid
double GLOBAL_E_N = 0;

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

  for(int i=0;i<3;i++){
    pinMode(sensor_pins[i],INPUT);
    delay(1500);
    sensor[i].setTimeout(500);
    if(!sensor[i].init()){
        Serial.print("Failed to detect and initialize sensor: ");
        Serial.println(i);
        while (1);
    }
    delay(1000);
    sensor[i].setAddress(sensor_address[i]);
    Serial.print("Sensor initialized: ");
    Serial.println(i);
    Serial.print("Address: ");
    Serial.println(sensor[i].getAddress());
    sensor[i].setDistanceMode(VL53L1X::Long);
    sensor[i].setMeasurementTimingBudget(50000);
  
  }

  for(int i=0;i<3;i++){
      sensor[i].startContinuous(500);
  }

  /* Other stuff*/

}

void loop() {
  // put your main code here, to run repeatedly:
  red_led_state = !red_led_state;
  digitalWrite(RED_LED, red_led_state);


  /* Waiting for start */
  while(waiting_for_ble_cmd("go"));

  /* Testing Code Run once */
  

  /* Run motor until stop */ 
  
  crash_forward(120);
  while (waiting_for_ble_cmd("end")){
    print_sensor_data();
    left = sensor[0].read();
    right = sensor[2].read();

    Serial.print("Difference left - right = ");
    Serial.println(right-left);

    servo_val = pid(right-left);
    Serial.print("servo val: ");
    Serial.println(servo_val);
    servo.write(servo_val); 

    if(sensor[1].read()<300){
      crash_backward(120);
    }else{
      crash_forward(120);
    }

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
  bleSerial.print("Send '");
  bleSerial.print(text); 
  bleSerial.print("' to continue \n");
  delay(1000);

  bleSerial.poll();
  while (bleSerial.availableLines() > 0){
    bleSerial.readLine(line, 128);
    if(strcmp(line, text) == 0){
      return false;
    }  
  }
  return true;
}


void print_sensor_data(){
    Serial.print(sensor[0].read());
    Serial.println("  (first sensor)"); 
    if (sensor[0].timeoutOccurred()) { Serial.print(" 1. TIMEOUT"); }

    Serial.print(sensor[1].read());
    Serial.println(" (second sensor)");
    if (sensor[1].timeoutOccurred()) { Serial.print(" 2. TIMEOUT"); }

    Serial.print(sensor[2].read());
    Serial.println(" (third sensor)");
    if (sensor[2].timeoutOccurred()) { Serial.print("3.  TIMEOUT"); }

    Serial.println();
}

int pid(int y_t){
  double e_n_last = GLOBAL_E_N;
  int r_t = 0, u_t = 0, e_t =0;
  double kp = 0.1, ki = 0, kd = 1;
  double u_calc =0;
  int t = 500;

  e_t = r_t-y_t;
  GLOBAL_E_N = (double)e_t;

  u_calc = kp*GLOBAL_E_N;//+kd*(e_t-e_n_last)/t;
  u_t = u_calc;

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