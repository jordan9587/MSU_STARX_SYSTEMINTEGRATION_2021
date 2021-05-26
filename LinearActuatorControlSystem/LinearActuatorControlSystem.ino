#define ANVPIN 7
#define IN1 3
#define IN2 4
#define IOPIN 5

double difference;
double setpoint;

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(ANVPIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IOPIN, INPUT);
  
  /// User Input:
  setpoint = 100; // Changes setpoint length on values 0-255.
  analogWrite(ANVPIN, 10); // Changes speed on values 0-255.
}

void loop() {
  int inputCurrent = analogRead(map(IOPIN, 0, 1023, 0 , 255));     // reads the current input value between 0 - 255.
  Serial.println("Input_Current: " + inputCurrent);
  
  difference = setpoint - inputCurrent;

  if (difference > inputCurrent) // Setpoint is greater length than current length.
  {
    retract(); // retract the stroke
  }
  else if (difference < inputCurrent) // Setpoint is less than length of current length.
  {
    extend(); // extend the stroke
  }
  else // Setpoint is eqivalent length than current length.
  {
    
  }

  /*
  // put your main code here, to run repeatedly:
  extend();
  delay(3000);
  retract();
  delay(3000);
  */
}

void extend() {
  Serial.println("Extending...");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void retract() {
  Serial.println("Retracting...");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}
