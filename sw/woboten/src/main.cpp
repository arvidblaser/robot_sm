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

char line[128]; // rxed ble data

/* Motor */
#define MOTOR_IN1 A2
#define MOTOR_IN2 A1

// Servo
Servo servo;
#define servoPin PIN_SERIAL_TX

// Tof
VL53L1X sensor[3];

#define reset_1 10
// #define reset_2  7 // not used yet
#define reset_3 4
// #define reset_2  2 // not used yet
#define reset_5 PIN_SERIAL_RX

uint8_t sensor_pins[] = {reset_1, reset_3, reset_5};
uint8_t sensor_address[] = {30, 32, 34};

// Rotation
#define interruptPin A0

/********** Globals *************/
volatile byte RED_LED_STATE = LOW;
volatile byte GREEN_LED_STATE = LOW;

// pid servo
double GLOBAL_E_N = 0;

// rotations encoder, pid speed
double E_SPEED_SUM = 0;
int ROTATIONS = 0;
unsigned long T_SPEED_PREV = 0;
double E_SPEED_PREV = 0;
int U_PWM_ACTUAL;

// Globals for rot_ISR
volatile int ROTATION_COUNTER = 0;
volatile double ROTATION_SPEED = 0;
volatile double AV_ROTATION_SPEED = 0;

unsigned long PREV_ROT_T = 0;

// Globals for set_speed
double R_speed = 40;
unsigned long int t_prev_set_speed = 0;
double P_speed = 35;
double I_speed = 0.0001;
double D_speed = 0.0001;

/****** Function declarations *****/
void rot_ISR();
void stop_wobot();
void crash_forward(int speed);
void crash_backward(int speed);

bool waiting_for_ble_cmd(const char *text);
double set_pid_param(const char *text);

void print_sensor_data();
int pid_turn(int right_t, int left_t);
int set_speed(double speed);
/******** Functions *********/
void setup()
{
  /* debug, led, ble */
  Serial.begin(115200);
  Serial.println("Wobot test");

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  if (!bleSerial.beginAndSetupBLE("Woboten"))
  {
    while (true)
    {
      Serial.println("failed to initialize HardwareBLESerial!");
      delay(1000);
    }
  }

  /* rotaion encoder */
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), rot_ISR, RISING); // LOW, HIGH, CHANGE, RISING, FALLING

  /* Motor */
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  /* Servo */
  servo.attach(servoPin);

  /* ToF */
  pinMode(reset_1, OUTPUT);
  pinMode(reset_3, OUTPUT);
  pinMode(reset_5, OUTPUT);

  digitalWrite(reset_1, LOW);
  digitalWrite(reset_3, LOW);
  digitalWrite(reset_5, LOW);

  delay(500);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  for (int i = 0; i < 3; i++)
  {
    pinMode(sensor_pins[i], INPUT);
    delay(1500);
    sensor[i].setTimeout(500);
    if (!sensor[i].init())
    {
      Serial.print("Failed to detect and initialize sensor: ");
      Serial.println(i);
      while (1)
        ;
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

  for (int i = 0; i < 3; i++)
  {
    sensor[i].startContinuous(500);
  }

  /* Other stuff*/
  // set pid values
  while (waiting_for_ble_cmd("hi"))
    ;
}

void loop()
{
  int left = 0;
  int right = 0;
  int servo_val = 0;
  // put your main code here, to run repeatedly:
  RED_LED_STATE = !RED_LED_STATE;
  digitalWrite(RED_LED, RED_LED_STATE);

  /* Waiting for start */
  bleSerial.println("default pid values: (currently new must be written)");
  bleSerial.println(P_speed);
  bleSerial.println(I_speed);
  bleSerial.println(D_speed);
  bleSerial.println(R_speed);

  bleSerial.println("test");

  P_speed = set_pid_param("p");
  I_speed = set_pid_param("i");
  D_speed = set_pid_param("d");
  R_speed = set_pid_param("speed");
  bleSerial.println(P_speed);
  bleSerial.println(I_speed);
  bleSerial.println(D_speed);
  bleSerial.println(R_speed);

  while (waiting_for_ble_cmd("go"))
    ;

  /* Testing Code Run once */

  /* Run motor until stop */

  while (waiting_for_ble_cmd("end"))
  {
    int pwm_result = 0;
    bleSerial.println("Some data:");
    bleSerial.print("pwm_result: ");
    bleSerial.println((double)pwm_result);
    bleSerial.print("U_PWM_ACTUAL: ");
    bleSerial.println((double)U_PWM_ACTUAL);
    bleSerial.print("millis(): ");
    bleSerial.println((double)millis());
    bleSerial.print("ROTATION_COUNTER: ");
    bleSerial.println((double)ROTATION_COUNTER);
    bleSerial.print("E_SPEED_PREV: ");
    bleSerial.println(E_SPEED_PREV);
    bleSerial.println("data done");

    // bleSerial.println(ROTATION_SPEED);
    // bleSerial.println(AV_ROTATION_SPEED);

    // ROTATION_COUNTER = 0;

    unsigned long currentMillis = 0, previousMillis = 0, interval = 10;
    currentMillis = millis();
    previousMillis = millis();
    bool run_forward = true;

    for (int i = 0; i < 1000; i++)
    { // do this for 10s before ble message again
      previousMillis = currentMillis;
      while (currentMillis - previousMillis <= interval)
      {

        // print_sensor_data();
        left = sensor[0].read(false);
        right = sensor[2].read(false);

        // Serial.print("Difference left - right = ");
        // Serial.println(right-left);

        servo_val = pid_turn(right, left);

        // Serial.print("servo val: ");
        // Serial.println(servo_val);

        if (sensor[1].read(false) < 300)
        {
          run_forward = false;
          servo.write(180-servo_val);
        }
        else
        {
          run_forward = true;
          servo.write(servo_val);
        }

        currentMillis = millis();
      }

      if (run_forward)
      {
        crash_forward(set_speed(R_speed));
      }
      else
      {
        pwm_result = set_speed(R_speed);
        crash_backward(pwm_result); // 120
      }
    }
  }

  stop_wobot();
}

void stop_wobot()
{
  analogWrite(MOTOR_IN1, 0);
  analogWrite(MOTOR_IN2, 0);
}

void crash_forward(int speed)
{
  analogWrite(MOTOR_IN1, speed);
  analogWrite(MOTOR_IN2, 0);
}

void crash_backward(int speed)
{
  analogWrite(MOTOR_IN1, 0);
  analogWrite(MOTOR_IN2, speed);
}

bool waiting_for_ble_cmd(const char *text)
{
  bleSerial.print("Send '");
  bleSerial.print(text);
  bleSerial.print("' to continue \n");
  delay(100); // TODO: consider removing

  bleSerial.poll();
  while (bleSerial.availableLines() > 0)
  {
    bleSerial.readLine(line, 128);
    if (strcmp(line, text) == 0)
    {
      return false;
    }
  }
  return true;
}

double set_pid_param(const char *text)
{
  bool waiting = true;
  int number = 0, whole_number = 0;
  int digit_number = 0, state = 0, part = 0;
  double sum_number = 0;
  while (waiting)
  {
    bleSerial.print("Send '");
    bleSerial.print(text);
    bleSerial.print("' param \n");
    delay(3000);
    bleSerial.poll();
    if (bleSerial.availableLines() > 0)
    {
      int buffer_s = bleSerial.readLine(line, 128);
      waiting = false;

      for (int i = 0; i < buffer_s; i++)
      {
        if (line[i] == '.')
        {
          part = i;
          i++;
          state = 1;
        }
        if (state == 0)
        {
          number = line[i] - '0';
          whole_number = 10 * whole_number + number;
        }
        else if (state == 1)
        {
          number = line[i] - '0';
          digit_number = 10 * digit_number + number;
        }
      }
      sum_number = whole_number + (double)digit_number / (pow(10, buffer_s - part - 1));
    }
  }
  return sum_number;
}

void print_sensor_data()
{
  Serial.print(sensor[0].read());
  Serial.println("  (first sensor)");
  if (sensor[0].timeoutOccurred())
  {
    Serial.print(" 1. TIMEOUT");
  }

  Serial.print(sensor[1].read());
  Serial.println(" (second sensor)");
  if (sensor[1].timeoutOccurred())
  {
    Serial.print(" 2. TIMEOUT");
  }

  Serial.print(sensor[2].read());
  Serial.println(" (third sensor)");
  if (sensor[2].timeoutOccurred())
  {
    Serial.print("3.  TIMEOUT");
  }

  Serial.println();
}

int pid_turn(int right_t, int left_t)
{
  // double e_n_last = GLOBAL_E_N;
  int threshold = 1000;
  if (right_t < threshold || left_t < threshold)
  {
    int y_t = right_t - left_t;
    int r_t = 0, u_t = 0, e_t = 0;
    double kp = 0.125; //, ki = 0, kd = 1;
    double u_calc = 0;

    e_t = r_t - y_t;
    GLOBAL_E_N = (double)e_t;

    u_calc = kp * GLOBAL_E_N; //+kd*(e_t-e_n_last)/t;
    u_t = u_calc;

    // typical values
    // y(t) = diff betwen tof1 & tof5
    //  in range

    u_t = u_t + 90;
    if (u_t > 180)
    {
      u_t = 180;
    }
    else if (u_t < 0)
    {
      u_t = 0;
    }
    return (u_t);
  }
  return (90);
}

int PID_speed(int r_speed, int y_speed)
{
  double e_t = r_speed - y_speed;

  // double kp = 1, kd = 5000.0, ki= 0.0001;
  int u_pwm;
  unsigned long t; // Proportional, derivative, integrate, time, previous time
  t = millis();
  E_SPEED_SUM = E_SPEED_SUM + e_t * (t - T_SPEED_PREV);
  u_pwm = P_speed * e_t + D_speed * (e_t - E_SPEED_PREV) / (t - T_SPEED_PREV) + I_speed * E_SPEED_SUM; //+ ki*E_SPEED_SUM;
  T_SPEED_PREV = t;
  E_SPEED_PREV = e_t;
  U_PWM_ACTUAL = u_pwm;
  if (u_pwm < 0)
  {
    u_pwm = 0;
  }
  if (u_pwm > 200)
  {
    u_pwm = 200; //kanske ballar ur men behövs högre än 150 för att backamed full sväng
  }
  return u_pwm;
}

int set_speed(double speed)
{ // cm/s
  // 7 varv = 25 cm-> 1 varv 3.57 cm
  double goal_rotations_speed = speed; // byt skalfaktor, double?
  int t = millis();
  double actual_rotations_speed = ROTATION_COUNTER * 3570 / (t - t_prev_set_speed); // ROTATION_SPEED*3570;
  t_prev_set_speed = t;
  ROTATION_COUNTER = 0;
  int pwm = PID_speed(goal_rotations_speed, actual_rotations_speed);

  return pwm;
}

void rot_ISR()
{
  // unsigned long t = millis();
  // ROTATION_SPEED = 1.0/(t-PREV_ROT_T);
  // AV_ROTATION_SPEED = (AV_ROTATION_SPEED+ROTATION_SPEED)/2;
  // PREV_ROT_T=t;

  // GREEN_LED_STATE = !GREEN_LED_STATE;
  // digitalWrite(GREEN_LED, GREEN_LED_STATE);
  ROTATION_COUNTER += 1;
  // TODO mät intervall för rotenkoder på oscillioskop
}
