#include <PID_v1.h>
#define ANVPIN 7
#define IN1 3
#define IN2 4
#define IOPIN 5

double difference;  // Stores the difference between outputH of PID and Setpoint.
double inputH, outputH, setpointH;  // Parameters for PID.
double HP = 1200;
double HI = 250000;
double HD = 10000;
// PID Object declares the paramteres that will represents tunings.
PID pidH(&inputH, &outputH, &setpointH, HP, HI, HD, DIRECT); 

// Rate of Change for Position to find velocity of linear actuator.
unsigned long lastTime = 0;   // Last time you connected to the server, in milliseconds.
const unsigned long updateDelay = 1000L;    // Delay between updates, in milliseconds.
float previousDistance, currentDistance;
float deltaSpeed = 0;   // Speed value calculated from change of position.

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(ANVPIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IOPIN, INPUT);

  // PID Setup
  pidH.SetOutputLimits(0,255);
  pidH.SetMode(AUTOMATIC);
  
  /// User Input:
  setpointH = 100; // Changes setpointH length on values 0-255.
  analogWrite(ANVPIN, 10); // Changes speed on values 0-255.
}

void loop() {
  // put your main code here, to run repeatedly:
  inputH = analogRead(map(IOPIN, 0, 1023, 0 , 255));     // reads the current input value between 0 - 255.

  // Compute PID and use outputH as the output of PID controller.
  pidH.Compute();
  Serial.print("Current Linear Actuator Length: ");
  Serial.print(outputH);

  // Calculate change of position for speed delta.
  previousDistance = outputH;
  // if 1000 milliseconds have passed since your last connection, then update delta for velocity.
  if (millis() - lastTime > updateDelay) {
    updateDelta(outputH, previousDistance);
  }
  
  difference = setpointH - outputH;

  if (difference > outputH) // Setpoint is greater length than current length.
  {
    retract(); // retract the stroke
  }
  else if (difference < outputH) // Setpoint is less than length of current length.
  {
    extend(); // extend the stroke
  }
  else // Setpoint is equivalent length than current length.
  {
    // Do nothing.
  }
}

void extend() {
  Serial.println("Extending...");
  analogWrite(ANVPIN, deltaSpeed); // Changes speed on values 0-255.
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void retract() {
  Serial.println("Retracting...");
  analogWrite(ANVPIN, deltaSpeed); // Changes speed on values 0-255.
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void updateDelta(float currentDistance, float previousDistance) {
  currentDistance = outputH;
  deltaSpeed = (currentDistance - previousDistance) / (updateDelay / 1000);   // Delta speed is in analog value (0-255) / updateDelay (1 sec)
  lastTime = millis();
}
