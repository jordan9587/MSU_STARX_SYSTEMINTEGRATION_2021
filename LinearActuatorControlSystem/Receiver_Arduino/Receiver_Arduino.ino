#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
Adafruit_MPU6050 mpu;

byte dataR[4]; //Initialized variable to store recieved data
float parameter;
void setup() 
{
  // Begin the Serial at 9600 Baud
  
  Serial.begin(9600);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);
}

void loop() 
{
  Serial.readBytes(dataR,4); //Read the serial data and store in var
  //memcpy(&parameter,dataR,4);
  Serial.print(dataR[0]HEX);
  Serial.print(dataR[1],HEX);
  Serial.print(dataR[2],HEX);
  Serial.println(dataR[3],HEX);
  //Serial.println(parameter); //Print data on Serial Monitor
  delay(1000);
}
