#include <PID_v1.h>
#include <EnableInterrupt.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <GyroToVelocity.h>

#define SERIAL_PORT_SPEED 9600 

#define PWM_NUM  2 //number of PWM signals
#define PWMS  0 //speed is stored in the 0 position of the array
#define PWMP  1 //position is stored in the 1 position of the array

#define PWMS_INPUT  A0 //analog pin for the speed reading
#define PWMP_INPUT  A1 //analog pin for the position reading

//final zero rate offset in radions/s:
//-0.0868,0.0044,0.0180
const double X_OFFSET = -0.0868;
const double Y_OFFSET = 0.0044;
const double Z_OFFSET = 0.0180;

uint32_t pwm_start[PWM_NUM]; // stores the time when PWM square wave begins
uint16_t pwm_values[PWM_NUM]; //stores the width of the PWM pulse in microseconds
volatile uint16_t pwm_shared[PWM_NUM]; 

const byte ANV = 5;
const byte IN1 = 2;
const byte IN2 = 3;

Adafruit_MPU6050 mpu; //included - Ben
double corrected_X, corrected_Y, corrected_Z;
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
int mode = 1;
double displacement;
void setup() 
{
  for(int i = 0; i < NUMBER_OF_FIELDS; i++)
  {
    sign[i] = 1;
  }

  Serial.begin(9600); //from 9600 to 115200 - Ben

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
  desiredSpeed = 50;
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
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }  
}

void loop() 
{
  while(singleLoop)
  {
    
    Serial.println("Setting...");
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

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x - X_OFFSET);
  //Serial.print(", Y: ");
  //Serial.print(g.gyro.y);
  //Serial.print(", Z: ");
  //Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  //Serial.print("Temperature: ");
  //Serial.print(temp.temperature);
  //Serial.println(" degC");
  corrected_X = g.gyro.x - X_OFFSET;
  desiredSpeed = geometry(corrected_X, displacement);
  Mdirection(corrected_X);
  pwm_read_values();
  PIDtoggle(pwm_values[PWMS]);
  loadCompensator.Compute();
  analogWrite(ANV, outputSpeed);
  Serial.print(currentSpeed); Serial.print(" , ");Serial.println(desiredSpeed);

  

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

void Mdirection(int dir)
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
  currentSpeed = -((double)(pwm_values[PWMS] - 510)/26+1/13);
  displacement = abs(7.5 / 990 * pwm_values[PWMP] - 7.5);
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
