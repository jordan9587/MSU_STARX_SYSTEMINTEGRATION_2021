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

//zero rate offset in radions/s
//recalibrate gyros periodically, probably once a month or before testing
const double gyro1[] = {-0.0213, -0.0192, -0.03};
const double gyro2[] = {-0.0919, 0.052, -0.019};
const double gyro3[] = {-0.0511, -0.0076, -0.0019};
const double gyro4[] = {-0.0588, -0.0185, -0.0298};
const double gyro5[] = {-0.0231, -0.0526, -0.0486};
const double gyro6[] = {-0.0254, -0.0077, -0.0045};
const double gyro7[] = {-0.1019, -0.041, -0.0028};
const double gyro8[] = {-0.0791, -0.027, -0.0346};
//variable that holds the specific gyro offset
double X_OFFSET = 0;
double Y_OFFSET = 0;
double Z_OFFSET = 0;

uint32_t pwm_start[PWM_NUM]; // stores the time when PWM square wave begins
uint16_t pwm_values[PWM_NUM]; //stores the width of the PWM pulse in microseconds
volatile uint16_t pwm_shared[PWM_NUM]; //array used in the interrupt routine to hold the pulse width values

Adafruit_MPU6050 mpu; //included
double corrected_X, corrected_Y, corrected_Z; //variables that store the angular speed taking into acount the offset 
//Excel data
bool singleLoop = false;
bool hardstop = true;

//PID_runtime
double currentSpeed, currentSpeed_abs, outputSpeed ,desiredSpeed;
double displacement; //position variable
double HP = 50, HI = 20, HD = 60;
PID loadCompensator(&currentSpeed_abs, &outputSpeed ,&desiredSpeed, HP, HI, HD,P_ON_M, DIRECT);
int mode = 0; //keeps track if PID is on or off

/*
PID_stop

This PID is needed as this PID goes by displacment inside of speed
*/
double setpoint = 3.75;   //the point that is considered to standing postion
double HP_stop = 50, HI_stop = 20, HD_stop = 60;  //these are the parameters to the PID

#define to_standing_pin 53  // pin that the button is connected too
const int seconds = 5;      // time delay after moving to standing before resuming

PID StoppingPID(&displacement, &currentSpeed ,&setpoint, HP, HI, HD,P_ON_M, DIRECT);
int Stop_PID_status = 0; //keeps track if PID is on or off



void setup() 
{
  //begin serial plotter
  Serial.begin(SERIAL_PORT_SPEED); //from 9600 to 115200


  //establish all pins
  pinMode(PWMS_INPUT, INPUT);
  pinMode(PWMP_INPUT, INPUT);
  pinMode(ANV,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(22,OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(22,HIGH);
  enableInterrupt(PWMS_INPUT, calc_speed, CHANGE);
  enableInterrupt(PWMP_INPUT, calc_position, CHANGE); 
  //PI Controller for sending to soft stop
  //placed here as I will toggle the High Pin when turning off this PID
  StoppingPID.SetOutputLimits(0,255);
  StopPID_status(true); // turn off this PID
  
  //PI Controller for load compensator
  pwm_read_values(); 
  desiredSpeed = 0;
  loadCompensator.SetMode(AUTOMATIC);
  loadCompensator.SetOutputLimits(0,255);

  HorK(HIP);
  //Checks for gyroscope
  if (!mpu.begin()) 
  {
    digitalWrite(LED_BUILTIN,HIGH); //built in led high means the arduino failed to recognize the gyro
    while (1) 
    {
      delay(10);
    }
  }
  else  digitalWrite(LED_BUILTIN,LOW); //led low means gyro is connected properly
  //range settings
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  offsetSwitch(5);
  Serial.println("end of setup");
}

void loop() 
{

 
  //print sensor values
  //gyrocheck();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  if(abs(g.gyro.y - Y_OFFSET) <= 0.05)//Sets the gyro value to 0 if it is below the threshold
  {
    Serial.println("below threshold");
    corrected_Y = 0;
    hardstop = true;
  }
  else
  {
    Serial.println("above threshold");
    corrected_Y = g.gyro.y - Y_OFFSET;
    hardstop = false;
  }
  
  //pwm_read_values();
  //desiredSpeed = abs(geometry(corrected_Y, displacement));
  Serial.println(corrected_Y);

//////////////////////// TO standing implementation
  if(!(digitalRead(to_standing_pin)))
  {
    Serial.println("entered if statment");
    pwm_read_values();
    StopPID_status(false); // turn the PID on


    //print for testing purposes
    Serial.println(".....Begin Move to Standing.....");
    String message = "Understood to be at distance " + String(displacement) +" Setpoint is : " + String(setpoint);
    Serial.println(message);


    //given a 20% range within the setpoint
    while(displacement >= setpoint*.1 || displacement <= setpoint*.9)
    {
      //collect the current actuator speed and displacement
      pwm_read_values();  // this will update current speed and the displacment

      //determine movement dirrection
      if(displacement > setpoint)
      {
        Serial.println("Moving negative");
        Mdirection(-1); 
      }
      else if (displacement < setpoint)
      {
         Serial.println("Moving postive");
         Mdirection(1);
      }
      else
      {
        Serial.println("withing tolerance");
        Mdirection(0);
        break;
      }

      //compute the displacement with the PID
      StoppingPID.Compute(); 

      
      //write the PID result to the actuator
      analogWrite(ANV, outputSpeed);
    }
     
    Serial.println("Pause for durration");
    delay(seconds*1000);
    Serial.println(".....resuming System.....");
    StopPID_status(true);
  }
  
  

////////////////////////
  desiredSpeed = geometry(corrected_Y, displacement); //determine the speed from angularVelocity and Current postion
  Mdirection(corrected_Y);  //set the dirrection for the actuator movemnt (postitive or negative)
  loadCompensator.Compute();  
  analogWrite(ANV, outputSpeed);
  Serial.println("after if statement");
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
  noInterrupts(); //Disables interrupts to make sure it doesnt mess up the memory copy below
  memcpy(pwm_values, (const void *)pwm_shared, sizeof(pwm_shared)); //memcpy handles memory to memory copy, copies from pwm_shared to pwm_values
  interrupts(); //Enables interrupts
  currentSpeed = ((double)(pwm_values[PWMS] - 510)/26+1/13); //calculates the real value [inch/sec] for the actuator speed
  currentSpeed_abs = abs(currentSpeed);
  displacement = abs(7.5 / 990 * pwm_values[PWMP] - 7.5); //calculates the real value [inch] for the actuator position
}

void calc_input(uint8_t channel, uint8_t input_pin) //Does the math for the PWM data
{
  if (digitalRead(input_pin) == HIGH) //Measures how long the PWM is that its peak
  {
    pwm_start[channel] = micros(); //Micros returns the number of microseconds since the board began running
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

/*
 * This function is used to turn the stopping PID on and off
 *  IMPORTANT NOTE: This will also toggle the other PID on/off
 */

void StopPID_status(bool PID_off)
{
  
  if(PID_off)
  {
    StoppingPID.SetMode(MANUAL); //Manual - PI is deactivated
    Stop_PID_status = MANUAL;
    digitalWrite(22,HIGH);

    //turn other PID on
    loadCompensator.SetMode(AUTOMATIC);//Automatic - PI is activated
    mode = AUTOMATIC;
  }
  else
  {
    StoppingPID.SetMode(AUTOMATIC);//Automatic - PI is activated
    Stop_PID_status = AUTOMATIC;
    digitalWrite(22,LOW);

    //turn other PID off
    loadCompensator.SetMode(MANUAL); //Manual - PI is deactivated
    mode = MANUAL;
    digitalWrite(22,HIGH);
  }
}


//the code below is going through testing
void PIDtoggle(bool hardstop) 
{
  if(hardstop == true || displacement >= 7.5 || displacement <= 0.05)
  {
    loadCompensator.SetMode(MANUAL); //Manual - PI is deactivated
    mode = MANUAL;
    digitalWrite(22,HIGH);
  }
  else
  {
    loadCompensator.SetMode(AUTOMATIC);//Automatic - PI is activated
    mode = AUTOMATIC;
    digitalWrite(22,LOW);
  }
}
void offsetSwitch(int choice){
    switch(choice){
        case 1:
            X_OFFSET = gyro1[1];
            Y_OFFSET = gyro1[2];
            Z_OFFSET = gyro1[3];
        break;

        case 2:
            X_OFFSET = gyro2[1];
            Y_OFFSET = gyro2[2];
            Z_OFFSET = gyro2[3];
        break;

        case 3:
            X_OFFSET = gyro3[1];
            Y_OFFSET = gyro3[2];
            Z_OFFSET = gyro3[3];
        break;
       
        case 4:
            X_OFFSET = gyro2[1];
            Y_OFFSET = gyro2[2];
            Z_OFFSET = gyro2[3];
        break;

        case 5:
            X_OFFSET = gyro2[1];
            Y_OFFSET = gyro2[2];
            Z_OFFSET = gyro2[3];
        break;

    }
}
