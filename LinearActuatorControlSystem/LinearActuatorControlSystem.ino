#define ANV_PIN 7
#define IN1 3
#define IN2 4
#define IO_PIN 5

double difference;

// Length of linear actuator from ranges 0-255.
double Setpoint = 100;

void setup() {
  // put your setup code here, to run once:
  pinMode(ANV_PIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IO_PIN, INPUT);

  analogWrite(ANV_PIN, 10); // Changes speed on values 0-255
  Serial.begin(115200);
}

void loop() {
  int Input_Current = analogRead(map(IO_PIN, 0, 1023, 0 , 255));     // reads the current input value between 0 - 255.
  Serial.println("Input_Current: " + Input_Current);
  
  difference = Setpoint - Input_Current;

  if (difference > Input_Current) // Setpoint is greater length than current length.
  {
    retract(); // retract the stroke
  }
  else if (difference < Input_Current) // Setpoint is less than length of current length.
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
