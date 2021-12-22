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
double currentSpeed, outputSpeed ,desiredSpeed;
double HP = 1, HI = 5, HD = 0;
PID loadCompensator(&currentSpeed, &outputSpeed ,&desiredSpeed, HP, HI, HD,P_ON_M, DIRECT);
//Excel data
const int NUMBER_OF_FIELDS = 3; // how many comma separated fields we expect
int fieldIndex = 0;            // the current field being received
double dummy[NUMBER_OF_FIELDS];   // array holding values for all the fields
double values[NUMBER_OF_FIELDS];
int sign[NUMBER_OF_FIELDS] = {1};
void setup() 
{
  for(int i = 0; i < NUMBER_OF_FIELDS; i++)
  {
    sign[i] = 1;
  }
  Serial.begin(9600);
  pinMode(DIOS, INPUT);
  pinMode(DIOP, INPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(ANV,OUTPUT);
  //PI Controller
  currentSpeed = PWMS;
  desiredSpeed = 50;
  loadCompensator.SetMode(MANUAL);
  loadCompensator.SetOutputLimits(0,205);
  //loadCompensator.SetTunings(100,100,0);
  //analogWrite(ANV,5);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  ISR_Enable(true,false);
}
void loop() 
{
  //Serial.print(PWMS); Serial.print(",");Serial.print(outputSpeed); Serial.println(",");
  //Serial.println(PWMS);
  currentSpeed = abs(PWMS);
  loadCompensator.Compute();
  analogWrite(ANV, outputSpeed);
}
void serialEvent()
{
  if(Serial.available() > 0)
  {
    char ch = Serial.read();
    if(ch >= '0' && ch <= '9')
    {
      dummy[fieldIndex] = (dummy[fieldIndex] * 10) + (ch - '0');
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
        values[i] = dummy[i]*sign[i];
        dummy[i] = 0;
        sign[i] = 1;
        Serial.println(values[i]);
      }
      fieldIndex = 0;
      //desiredSpeed = values[0];
      Mdirection(values[0]);
      loadCompensator.SetTunings(values[1],values[2],0);
      PIDtoggle();
    } 
  }
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
    PWMS = DutyS - 501;
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
    //retract
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    //M_dir = 1;
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}
void PIDtoggle()
{
  if((digitalRead(26) == 1 && digitalRead(27) == 1) || (digitalRead(26) == 0 && digitalRead(27) == 0))
      {
        loadCompensator.SetMode(MANUAL);
      }
      else
      {
        loadCompensator.SetMode(AUTOMATIC);
      }
}
