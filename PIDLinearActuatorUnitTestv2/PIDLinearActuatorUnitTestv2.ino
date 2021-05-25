#include <PID_v1.h>
#include "PWM.hpp"
#define IN1 1
#define IN2 2
#define IO_PIN 3
#define ANV_PIN 4

int directionTest = 0;
int actuatorSpeed;
double currentLengthIO;
double InputH, OutputH, SetpointH;
double HP = 1200;
double HI = 250000;
double HD = 10000;
PWM PWMInput(IO_PIN); 
PID Hpid(&InputH, &OutputH, &SetpointH, HP, HI, HD, DIRECT); 

void setup() 
{
  Serial.begin(115200);  // initialize serial communication at 115200 bits per second
  //pin setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IO_PIN, INPUT);
  pinMode(ANV_PIN, OUTPUT);
<<<<<<< Updated upstream:PIDLinearActuatorUnitTestv2/PIDLinearActuatorUnitTestv2.ino

  Hpid.SetMode(AUTOMATIC);
  
  initiate();
  delay(1000);
  currentLengthIO = pulseIn(IO_PIN, HIGH) / 1000 *1023 ;
  Serial.println(currentLengthIO);
=======
  PWMInput.begin(true); //PWM reads high duration 
>>>>>>> Stashed changes:attempt2/attempt2.ino

  //PID setup
  SetpointH = 500; //Setpoint has to be between 0 and 1023
  Hpid.SetOutputLimits(0,255);
  Hpid.SetMode(AUTOMATIC);
  
}
void loop() 
{
<<<<<<< Updated upstream:PIDLinearActuatorUnitTestv2/PIDLinearActuatorUnitTestv2.ino
    //lenght input test
//  digitalWrite(IN1, 0);
//  digitalWrite(IN2, 0);
//  analogWrite(ANV_PIN, 127);
//  delay(1000);
  
//  currentLengthIO = analogRead(IO_PIN);
//  currentLengthIO = pulseIn(IO_PIN, LOW);
//  Serial.print(currentLengthIO);
//  Serial.print("---");
//  currentLengthIO = pulseIn(IO_PIN, LOW);
//  Serial.println(currentLengthIO);
//Serial.println(PIND3);

//Serial.println(PINC);


  //extention test
//  currentLengthIO = map(pulseIn(IO_PIN, LOW), 0, 1000, 0, 1023);
//  Serial.println(currentLengthIO);
//  switch(directionTest)
//  {
//    case 0:
//      analogWrite(ANV_PIN,255);
////      extend();
//      Serial.println("testing testing");
//      if(currentLengthIO > 995)
//      {
//        Serial.println("Switching direction");
//        directionTest = 1;
//      }
//      break;
//    case 1:
//      analogWrite(ANV_PIN,10);
////      retract();
//      if(currentLengthIO < 100)
//      {
//        Serial.println("Switching direction");
//        directionTest = 0;
//      }
//      break;
//  }
  //PID Test
  Hpid.SetTunings(HP,HI,HD);
  InputH = map(pulseIn(IO_PIN, LOW), 0, 1000, 0, 1023);
  Hpid.Compute();
//  OutputH = map(OutputH, 0, 255, 0, 1023);
  
  Serial.println(OutputH);
//  if(InputH < 0)
=======
      //lenght input test
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  analogWrite(ANV_PIN,0);
  delay(5000);
  currentLengthIO = PWMInput.getValue();
  Serial.println(currentLengthIO);
  analogWrite(ANV_PIN, 255);
  delay(5000);
  currentLengthIO = PWMInput.getValue();
  Serial.println(currentLengthIO);
  //extention test
//  currentLengthIO = map(PWMInput.getValue(), 0, 1000, 0, 1023);
//  switch(directionTest)
//  {
//    case 0:
//    analogWrite(ANV_PIN,10);
//    extend();
//    if(currentLengthIO > 1000)
//    {
//      Serial.print("extention: ");
//      Serial.println(currentLengthIO);
//      delay(1000);
//      directionTest = 1;
//    }
//    break;
//    case 1:
//    analogWrite(ANV_PIN,10);
//    retract();
//    if(currentLengthIO < 100)
//    {
//      Serial.print("extention: ");
//      Serial.println(currentLengthIO);
//      delay(1000);
//      directionTest = 0;
//    }
//    break;
//  }
  //PID Test
//  InputH = map(PWMInput.getValue(), 0, 1000, 0, 1023);
//  Hpid.SetTunings(HP,HI,HD);
//  Hpid.Compute();
//  if(OutputH < 0)
>>>>>>> Stashed changes:attempt2/attempt2.ino
//  {
////    retract();
    analogWrite(ANV_PIN, abs(OutputH));
//  }
//  if(InputH >= 0)
//  {
////    extend();
//    analogWrite(ANV_PIN, abs(OutputH)); 
//  }
 
}

void extend() // this function enables the motor to run
{
  digitalWrite(IN2, LOW);
  digitalWrite(IN1, HIGH);
  
}
void retract() // this function reverses the direction of motor
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}
