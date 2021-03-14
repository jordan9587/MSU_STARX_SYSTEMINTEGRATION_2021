#include "svmPrototype.h"

#define NUM_SAMPLES 90
#define NUM_AXES 3
#define INTERVAL 20 // Originally 90
#define THRESHOLD 250 // NEW
// To reduce spikes in reading we set a sensible value to truncate too large EMG voltage values.
#define TRUNCATE 1000 // NEW

float baseline[NUM_AXES];  // NEW
float features[NUM_SAMPLES * NUM_AXES];
int emg0 = A0;
int emg1 = A1;
int emg2 = A2;
const char* currentState;

Eloquent::ML::Port::SVM clf;

void readEMG(float &v0, float &v1,float &v2) {
  // Read each emg sensor and convert to voltage (0-5V)
  // Should be recorded as analogread * (5.0 / 1023.0) in final version.
  v0 = analogRead(emg0);
  v1 = analogRead(emg1);
  v2 = analogRead(emg2);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  calibrate();
}

void loop() {
  float v0, v1, v2;
  // put your main code here, to run repeatedly:
  if (motionDetected(v0, v1, v2) == true) {
    // Read EMG sensors
    readEMG(v0, v1, v2);
  
    // Condition/ make any normalisation to the emg sensor input.
    // Calibration procedure to remove this fixed offset from the readings. All EMG Voltage lines will oscillate from 0.
    v0 = constrain(v0 - baseline[0], -TRUNCATE, TRUNCATE);
    v1 = constrain(v1 - baseline[1], -TRUNCATE, TRUNCATE);
    v2 = constrain(v2 - baseline[2], -TRUNCATE, TRUNCATE);
  
    // Record emg data into svm format [v0, v1, v2, v0, v1, v2, ... , NUM_SAMPLESth v3]
    recordEMG();
    // Use SVM to classify emg dataset
    svmClassify();
  }
  else {
    // No movement detected, return 0 as voltage.
    v0 = 0;
    v1 = 0;
    v2 = 0;
  }

}

//NEW
// "Zero" the readings by setting baseline startup noise
void calibrate() {
    float v0, v1, v2;

    for (int i = 0; i < 10; i++) {
        readEMG(v0, v1, v2);
        delay(10);
    }

    baseline[0] = v0;
    baseline[1] = v1;
    baseline[2] = v2;
}

// NEW
bool motionDetected(float v0, float v1, float v2) 
{
  return (abs(v0) + abs(v1) + abs(v2)) > THRESHOLD;
}

void recordEMG() {
    // Record emg data into svm format [v0, v1, v2, v0, v1, v2, ... , NUM_SAMPLESth v3]
    float v0, v1, v2;
    for (int i = 0; i < NUM_SAMPLES; i++) 
    {
        readEMG(v0, v1, v2);
        // Calibration procedure to remove this fixed offset from the readings. All EMG Voltage lines will oscillate from 0.
        v0 = constrain(v0 - baseline[0], -TRUNCATE, TRUNCATE);
        v1 = constrain(v1 - baseline[1], -TRUNCATE, TRUNCATE);
        v2 = constrain(v2 - baseline[2], -TRUNCATE, TRUNCATE);
        
        features[i * NUM_AXES + 0] = v0;
        features[i * NUM_AXES + 1] = v1;
        features[i * NUM_AXES + 2] = v2;
        delay(INTERVAL);
    }
}
void svmClassify() {
    // Use SVM to predict label for given emg input array.
    Serial.print("Detected gesture: ");
    currentState = clf.predictLabel(features);
    Serial.println(currentState);
}
