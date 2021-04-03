#include "svmPrototype.h"
#include <PID_v1.h>
#define NUM_SAMPLES 90
#define NUM_AXES 3
#define INTERVAL 20 // Originally 90
#define THRESHOLD 250 // NEW
// To reduce spikes in reading we set a sensible value to truncate too large EMG voltage values.
#define TRUNCATE 1000 // NEW
//set the PID pins for Knee and Hip actuators
#define PIN_INH 0
#define PIN_OUTH 1
#define PIN_INK 2
#define PIN_OUTK 3
#define KP 1200
#define KI 25000
#define KD 10000
float baseline[NUM_AXES];  // NEW
float features[NUM_SAMPLES * NUM_AXES];
int emg0 = A0;
int emg1 = A1;
int emg2 = A2;
//PID object variables
int currentStateMovement = 0;
double SetpointH, InputH, OutputH, SetpointK, InputK, OutputK;
//Movement label values for Hip angle, Knee angle, Hip Setpoint, Knee Setpoint respectively
float Standing[] = { 0, 0, 18.25, 21.21 };      //1
float HeelStrike[] = { 20, 0, 16.22, 21.21 };   //2
float LdgResp[] = { 20, 20, 19.13, 20.160 };    //3
float MidStance[] = { 0, 5, 18.25, 20.98 };     //4
float TmlStance[] = { -20, 0, 17.21, 21.21 };   //5
float PreSwing[] = { -10, 40, 17.74, 18.85 };   //6
float InitSwing[] = { 15, 60, 18.93, 17.39 };   //7
float MidSwing[] = { 25, 25, 19.31, 19.86 };    //8
float TmlSwing[] = { 20, 0, 19.13, 21.21 };     //9
int arSize = sizeof(Standing) / sizeof(Standing[0]);
Eloquent::ML::Port::SVM clf;
PID Hpid(&InputH, &OutputH, &SetpointH, HipP, HipI, HipD, DIRECT);
PID Kpid(&InputK, &OutputK, &SetpointK, KneeP, KneeI, KneeD, DIRECT);
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
  SetpointH = Standing[2];
  Hpid.SetOutputLimits(0,255);
  Kpid.SetOutputLimits(0,255);
  Hpid.SetTunings(KP, KI, KD);
  Kpid.SetTunings(KD, KI, KD);
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
    FSM(currentStateMovement);
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
    currentStateMovement = clf.predictLabel(features);
    
    Serial.println(currentState);
}
void FSM(int currentStateMovement)
{
  InputH = analogRead(PIN_INH);
  InputK = analogRead(PIN_INK);
  switch (index)
  {
    case 0:           //Standing
      SetpointH = Standing[2];
      SetpointK = Standing[3];
      PIDCompute();
      //get EMG data and find movement label and change the value of Movement acordingly in order to go to next state
      break;
    case 1:
      SetpointH = HeelStrike[2];
      SetpointK = HeelStrike[3];
      PIDCompute();
      break;
    case 2:           //LdgResp
      SetpointH = LdgResp[2];
      SetpointK = LdgResp[3];
      PIDCompute();
      break;
    case 3:           //MidStance
      SetpointH = MidStance[2];
      SetpointK = MidStance[3];
      PIDCompute();
      break;
    case 4:           //TmlStance
      SetpointH = TmlStance[2];
      SetpointK = TmlStance[3];
      PIDCompute();
      break;
    case 5:           //PreSwing
      SetpointH = PreSwing[2];
      SetpointK = PreSwing[3];
      PIDCompute();
      break;
    case 6:           //InitSwing
      SetpointH = InitSwing[2];
      SetpointK = InitSwing[3];
      PIDCompute();
      break;
    case 7:           //MidSwing
      SetpointH = MidSwing[2];
      SetpointK = MidSwing[3];
      PIDCompute();
      break;
    case 8:           //TmlSwing
      SetpointH = TmlSwing[2];
      SetpointK = TmlSwing[3];
      PIDCompute();
      break;
     default:
      PIDCompute();
    }
}
void PIDCompute()
{
    Hpid.Compute();
    Kpid.Compute();
    analogWrite(PIN_OUTH, OutputH);
    analogWrite(PIN_OUTK, OutputK);
}
