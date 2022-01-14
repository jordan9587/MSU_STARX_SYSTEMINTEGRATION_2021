//PWM reading variables
volatile unsigned long timer[4]; //timer that holds the rising and falling edge of PIN 24/DIO1->reads position waveform
volatile double PWMS;
volatile double PWMP; //holds Duty cycle value
volatile long DutyS, DutyP;


//pinout constants
const byte ANV = 2;
const byte DIOS = 24;
const byte DIOP = 25;
const byte IN1 = 26;
const byte IN2 = 27;

const int NUMBER_OF_FIELDS = 1;   // how many comma separated fields we expect
int fieldIndex = 0;               // the current field being received
double dummy[NUMBER_OF_FIELDS];   // array holding values for all the fields
double values[NUMBER_OF_FIELDS];  // array holding each final value of the serial monitor inputs
int sign[NUMBER_OF_FIELDS];

int singleLoop = true;
void setup() 
{
  // Serial and pin mode setup
  Serial.begin(9600);
  pinMode(DIOS, INPUT);
  pinMode(DIOP, INPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(ANV,OUTPUT);
  //
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  analogWrite(ANV,50);
  delay(500);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  analogWrite(ANV,50);
  delay(500);
  
  ISR_Enable(false,true);
  for(int i = 0; i < NUMBER_OF_FIELDS; i++)
  {
    sign[i] = 1;
  }
}

void loop() 
{
  // put your main code here, to run repeatedly:
  //Serial.println(PWMS);
  while(singleLoop)
  {
      // To begin testing make sure the serial monitor commands are working
      // 1. make sure the linear actuator is at min lenght
      // 2. Try the following serial monitor commands: 50 then -50
      Serial.print("Position Start: "); Serial.println(PWMP);
      Mdirection(values[0]);
      analogWrite(ANV, abs(values[0]));
      delay(500);
      digitalWrite(IN1,LOW);
      digitalWrite(IN2,LOW);
      analogWrite(ANV, 0);
      Serial.print("Position End: "); Serial.println(PWMP);
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
        values[i] = dummy[i]*sign[i];
        dummy[i] = 0;
        sign[i] = 1;
      }
      fieldIndex = 0;      
    } 
  }
}
void Mdirection(double dir)
{
  if(dir < 0)
  {
    //extend
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  if(dir > 0)
  {
    //retract
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}
