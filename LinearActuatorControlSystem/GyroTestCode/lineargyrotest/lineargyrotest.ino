#include <PID_v1.h>     //PID library
#include <EnableInterrupt.h>  //library that enables to read pwm values
#include <Adafruit_MPU6050.h> //gyro library
#include <Adafruit_Sensor.h>  //library to run adafruit sensors
#include <Wire.h>   //library that enables serail communications
#include <GyroToVelocity.h>   //geometry functions for HIP and KNEE actuators

#define SERIAL_PORT_SPEED 9600 

#define PWM_NUM  2 //number of PWM signals
#define PWMS  0 //speed is stored in the 0 position of the array
#define PWMP  1 //position is stored in the 1 position of the array

#define PWMS_INPUT  A0 //analog pin for the speed reading
#define PWMP_INPUT  A1 //analog pin for the position reading
#define ANV 5 //analog voltage speed control pin
#define IN1 2 //direction of actuation
#define IN2 3 //direction of actuation
//final zero rate offset in radions/s:

//-0.0868,0.0044,0.0180
//-0.0511, -0.0128, 0.0009

const double X_OFFSET = -0.0868;
const double Y_OFFSET = -0.0044;
const double Z_OFFSET = 0.0180;

uint32_t pwm_start[PWM_NUM]; // stores the time when PWM square wave begins
uint16_t pwm_values[PWM_NUM]; //stores the width of the PWM pulse in microseconds
volatile uint16_t pwm_shared[PWM_NUM]; //array used in the interrupt routine to hold the pulse width values

Adafruit_MPU6050 mpu; //included - Ben
double corrected_X, corrected_Y, corrected_Z; //variables that store the angular speed taking into acount the offset 
//Excel data
const int NUMBER_OF_FIELDS = 1; // how many comma separated fields we expect
int fieldIndex = 0;            // the current field being received
double dummy[NUMBER_OF_FIELDS];   // array holding values for all the fields
double serial_values[NUMBER_OF_FIELDS];
int sign[NUMBER_OF_FIELDS];
bool singleLoop = false;

//PID
double currentSpeed, outputSpeed ,desiredSpeed;
double HP = 100, HI = 500, HD = 0;
PID loadCompensator(&currentSpeed, &outputSpeed ,&desiredSpeed, HP, HI, HD,P_ON_M, DIRECT);
int mode = 0; //keeps track if PID is on or off
double displacement; //position variable
void setup() 
{
  for(int i = 0; i < NUMBER_OF_FIELDS; i++)
  {
    sign[i] = 1;
  }

  Serial.begin(SERIAL_PORT_SPEED); //from 9600 to 115200 - Ben

  pinMode(PWMS_INPUT, INPUT);
  pinMode(PWMP_INPUT, INPUT);
  pinMode(ANV,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(22,OUTPUT);
  digitalWrite(22,HIGH);
  enableInterrupt(PWMS_INPUT, calc_speed, CHANGE);
  enableInterrupt(PWMP_INPUT, calc_position, CHANGE); 
  
  //PI Controller
  pwm_read_values();
  desiredSpeed = 0;
  loadCompensator.SetMode(AUTOMATIC);
  loadCompensator.SetOutputLimits(0,255);
  
  HorK(HIP);
  //Checks for gyroscope - Ben
  if (!mpu.begin()) 
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1) 
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  //range settings - Ben
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() 
{
  while(singleLoop)
  {
    delay(1000);
    desiredSpeed = abs(serial_values[0]);
    singleLoop = false;
  }
  //print sensor values - Ben
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //Serial.print("Acceleration X: ");
  //Serial.print(a.acceleration.x);
  //Serial.print(", Y: ");
  //Serial.print(a.acceleration.y);
  //Serial.print(", Z: ");
  //Serial.print(a.acceleration.z);
  //Serial.println(" m/s^2");

  //Serial.print("Rotation Y: ");
  //Serial.println(g.gyro.y - Y_OFFSET);
  //Serial.print(", Y: ");
  //Serial.print(g.gyro.y);
  //Serial.print(", Z: ");
  //Serial.print(g.gyro.z);
  //Serial.println(" rad/s");

  //Serial.print("Temperature: ");
  //Serial.print(temp.temperature);
  //Serial.println(" degC");
  
  if(abs(g.gyro.y - Y_OFFSET) <= 0.05)
  {
    Serial.println("below threshold");
    corrected_Y = 0;
  }
  else
  {
    Serial.println("above threshold");
    corrected_Y = g.gyro.y - Y_OFFSET;
  }
  pwm_read_values();
  desiredSpeed = abs(geometry(corrected_Y, displacement));
  Mdirection(corrected_Y);
  PIDtoggle(pwm_values[PWMS]);
  loadCompensator.Compute();
  analogWrite(ANV, outputSpeed);
  //Serial.print(corrected_Y);Serial.print(","); Serial.print(currentSpeed); Serial.print(",");Serial.print(desiredSpeed);Serial.print(","); Serial.println(displacement);

  

}
void serialEvent()
{
  if(Serial.available() > 0)
  {
    char ch = Serial.read();
    //Serial.println(ch);
    if(ch >= '0' && ch <= '9')
    {
      dummy[fieldIndex] = (dummy[fieldIndex] * 10) + (ch - '0');
      //Serial.println(dummy[fieldIndex]);
    }
    else if(ch == ',')
    {
      if(fieldIndex < (NUMBER_OF_FIELDS - 1))
        fieldIndex++;
    }
    else if(ch == '-')
      sign[fieldIndex] = -1;
    else
    {
      for(int i = 0; i < NUMBER_OF_FIELDS; i++)
      {
        serial_values[i] = dummy[i]*sign[i];
        dummy[i] = 0;
        sign[i] = 1;
      }
      fieldIndex = 0;   
      singleLoop = true;
    } 
    
  }
}

void Mdirection(float dir)
{
  if(dir > 0)
  {
    //extend
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else if(dir < 0)
  {
    //retract
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}
void pwm_read_values() 
{
  noInterrupts();
  memcpy(pwm_values, (const void *)pwm_shared, sizeof(pwm_shared));
  interrupts();
  currentSpeed = ((double)(pwm_values[PWMS] - 510)/26+1/13); //calculates the real value [inch/sec] for the actuator speed
  displacement = abs(7.5 / 990 * pwm_values[PWMP] - 7.5); //calculates the real value [inch] for the actuator position
}

void calc_input(uint8_t channel, uint8_t input_pin) 
{
  if (digitalRead(input_pin) == HIGH) 
  {
    pwm_start[channel] = micros();
  } 
  else 
  {
    uint16_t pwm_compare = (uint16_t)(micros() - pwm_start[channel]);
    if(pwm_compare > 1002)
    {
      pwm_compare = 0;
    }
    pwm_shared[channel] = pwm_compare;
  }
}
void calc_speed() { calc_input(PWMS, PWMS_INPUT); }
void calc_position() { calc_input(PWMP, PWMP_INPUT); }

//the code below is going through testing
void PIDtoggle(int hardstop)
{
  if(hardstop < 515 && hardstop > 500)
  {
    loadCompensator.SetMode(MANUAL);
    mode = MANUAL;
  }
  else
  {
    loadCompensator.SetMode(AUTOMATIC);
    mode = AUTOMATIC;
  }
}
