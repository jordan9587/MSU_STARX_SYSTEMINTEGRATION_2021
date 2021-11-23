#include <PID_v1.h>

//PWM reading variables
volatile unsigned long timer[4]; //timer that holds the rising and falling edge of PIN 24/DIO1->reads position waveform
volatile double PWMS;
volatile double PWMP; //holds Duty cycle value
volatile long DutyS, DutyP;
double error;
//pinout constants
const byte ANV = 2;
const byte DIOS = 24;
const byte DIOP = 25;
const byte IN1 = 26;
const byte IN2 = 27;
//PID
double currentSpeed, outputSpeed ,desiredSpeed, HP, HI, HD;
PID loadCompensator(&currentSpeed, &outputSpeed ,&desiredSpeed, HP, HI, HD, DIRECT);

void setup() 
{
  Serial.begin(9600);
  pinMode(DIOS, INPUT);
  pinMode(DIOP, INPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(ANV,OUTPUT);
  //attachInterrupt(digitalPinToInterrupt(24),ISR1, CHANGE);
  loadCompensator.SetOutputLimits(0,205);
  loadCompensator.SetMode(AUTOMATIC);
  loadCompensator.SetTunings(100,100,0);
  analogWrite(ANV,155);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,HIGH);
  ISR_Enable(true,true);
}
void loop() 
{
  //PIDCompute(10,50,60);
  Serial.print(PWMS); Serial.print("   "); Serial.println(PWMP);
  loadCompensator.Compute();
}
void ISR1() 
{
  switch(digitalRead(24))
  {
    case 0:
    timer[0] = micros();
    break;
    case 1:
    timer[1] = micros();
    DutyS = timer[1] - timer[0];
    PWMS = DutyS;
    break;
  }
}
void ISR2()  
{
  switch(digitalRead(25)) //Reads pin 25
  {
    case 0: // If the PWM is at the lowest
    timer[2] = micros(); //Measures how long PMW is at the lowest
    break;
    case 1: //If the PWM is at the highest
    timer[3] = micros();  //Measures how long PMW is at the highest
    DutyP = timer[3] - timer[2]; // Comapres the time between the highest and the lowest
    PWMP = DutyP;
    break;
  }
}
void ISR_Enable(bool Enable_speed, bool Enable_position)
{
  if(Enable_speed)
  {
  attachInterrupt(digitalPinToInterrupt(24),ISR1, CHANGE);
  }
  if(Enable_position)
  {
  attachInterrupt(digitalPinToInterrupt(25),ISR2, CHANGE);
  }
}
void Mdirection(double error)
{
  if(error < 0)
  {
    //extend
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    //M_dir = 0;
  }
  if(error > 0)
  {
    //contract
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    //M_dir = 1;
  }
}
