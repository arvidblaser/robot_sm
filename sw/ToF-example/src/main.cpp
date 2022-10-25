#include <Arduino.h>

#include <Wire.h>
#include <VL53L1X.h>

void print_sensor_data(void);

VL53L1X sensor_1;
VL53L1X sensor_2;
VL53L1X sensor_3;

const int reset_1 = 7;
const int reset_2 = 8;
const int reset_3 = 9;

void setup()
{
	//Pin setup
  pinMode(reset_1,OUTPUT);
  pinMode(reset_2, OUTPUT);
  pinMode(reset_3, OUTPUT);
  digitalWrite(reset_1, LOW);
  digitalWrite(reset_2, LOW);  
  digitalWrite(reset_3, LOW);

  delay(500);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  Serial.begin(115200);

	//Initialize sensors
  //All sensors start with same address, therefore they need to be  hold in reset mode
  //and initalized after each other 
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

  pinMode(reset_2, INPUT);
  delay(1500);
  sensor_2.setTimeout(500);
  if (!sensor_2.init())
  {
    Serial.println("Failed to detect and initialize sensor 2!");
    while (1);
  }
  delay(1000);
  sensor_2.setAddress((uint8_t)32);
  Serial.println("Sensor 2 initialized");

  pinMode(reset_3, INPUT);
  delay(1500);
  sensor_3.setTimeout(500);
    if (!sensor_3.init())
  {
    Serial.println("Failed to detect and initialize sensor 3!");
    while (1);
  }
  delay(1000);
  sensor_3.setAddress((uint8_t)34);
  Serial.println("Sensor 3 initialized");
  
  Serial.println("addresses set");
  Serial.print("Address 1: ");
  Serial.println(sensor_1.getAddress());
  Serial.print("Address 2: ");
  Serial.println(sensor_2.getAddress());
  Serial.print("Address 3: ");
  Serial.println(sensor_3.getAddress());
  
  sensor_1.setDistanceMode(VL53L1X::Long);
  sensor_1.setMeasurementTimingBudget(50000);
  
	sensor_2.setDistanceMode(VL53L1X::Long);
  sensor_2.setMeasurementTimingBudget(50000);
  
  sensor_3.setDistanceMode(VL53L1X::Long);
  sensor_3.setMeasurementTimingBudget(50000);

  sensor_1.startContinuous(500);
  sensor_2.startContinuous(500);
  sensor_3.startContinuous(500);

}

void loop()
{
  print_sensor_data();
  delay(1000);
}

void print_sensor_data(){
    Serial.print(sensor_1.read());
    Serial.println("  (first sensor)"); 
    if (sensor_1.timeoutOccurred()) { Serial.print(" 1. TIMEOUT"); }

    Serial.print(sensor_2.read());
    Serial.println(" (second sensor)");
    if (sensor_2.timeoutOccurred()) { Serial.print(" 2. TIMEOUT"); }

    Serial.print(sensor_3.read());
    Serial.println(" (third sensor)");
    if (sensor_3.timeoutOccurred()) { Serial.print("3.  TIMEOUT"); }

    Serial.println();
}