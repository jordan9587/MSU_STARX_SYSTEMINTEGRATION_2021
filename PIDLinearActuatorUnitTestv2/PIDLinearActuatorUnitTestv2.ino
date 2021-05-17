#include <PID_v1.h>
#define IN1 1
#define IN2 2
#define IO_PIN 3
#define ANV_PIN 4


double currentLengthIO;
int directionTest = 0;
int actuatorSpeed;
// Set setpoint you want to go to. Currently its at 3 inches.
int ePOS = 255;
double InputH, OutputH, SetpointH;
double HP = 1200;
double HI = 250000;
double HD = 10000;
PID Hpid(&InputH, &OutputH, &SetpointH, HP, HI, HD, DIRECT); 
void setup() 
{
  Serial.begin(115200);  // initialize serial communication at 115200 bits per second
  //pin setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IO_PIN, INPUT);
  pinMode(ANV_PIN, OUTPUT);

  Hpid.SetMode(AUTOMATIC);
  
  initiate();
  delay(1000);
  currentLengthIO = pulseIn(IO_PIN, HIGH) / 1000 *1023 ;
  Serial.println(currentLengthIO);

  //PID setup
  SetpointH = 500; //Setpoint has to be between 0 and 1023
  Hpid.SetOutputLimits(0,255);
}
void loop() 
{
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
void initiate()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ANV_PIN, 0);
}
