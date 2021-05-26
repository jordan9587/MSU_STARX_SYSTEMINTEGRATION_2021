#include "svmPrototype.h"
#include <PID_v1.h>

/// Support Vector Machine Variables
#define NUM_SAMPLES 90
#define NUM_AXES 3
// Time interval for delaying EMG recordings.
#define INTERVAL 20
#define LOWERTHRESHOLD 250
// To reduce spikes in reading we set a sensible value to truncate too large EMG voltage values.
#define HIGHERTHRESHOLD 1000
float features[NUM_SAMPLES * NUM_AXES];
int emg0 = A0;
int emg1 = A1;
int emg2 = A2;
//PID object variables
int currentStateMovement = 0;
Eloquent::ML::Port::SVM clf;


/// PID Variables/ Linear Actuator
#define PIN_INH 0
#define PIN_OUTH 1
#define PIN_INK 2
#define PIN_OUTK 3
#define KP 1200
#define KI 25000
#define KD 10000
double SetpointH, InputH, OutputH, SetpointK, InputK, OutputK;
// Array of Movement label values for Hip angle, Knee angle, Hip Setpoint, Knee Setpoint respectively.
float Standing[] = { 0, 0, 18.25, 21.21 };      // State 1
float HeelStrike[] = { 20, 0, 16.22, 21.21 };   // State 2
float LdgResp[] = { 20, 20, 19.13, 20.160 };    // State 3
float MidStance[] = { 0, 5, 18.25, 20.98 };     // State 4
float TmlStance[] = { -20, 0, 17.21, 21.21 };   // State 5
float PreSwing[] = { -10, 40, 17.74, 18.85 };   // State 6
float InitSwing[] = { 15, 60, 18.93, 17.39 };   // State 7
float MidSwing[] = { 25, 25, 19.31, 19.86 };    // State 8
float TmlSwing[] = { 20, 0, 19.13, 21.21 };     // State 9
int arSize = sizeof(Standing) / sizeof(Standing[0]);  // Size of array
PID pidH(&InputH, &OutputH, &SetpointH, KP, KI, KD, DIRECT);

/*
* Setup code that runs once before loop execution function.
*/
void setup() {
  Serial.begin(115200);
  SetpointH = Standing[2];
  pidH.SetOutputLimits(0,255);
  pidH.SetTunings(KP, KI, KD);
}

/*
* Loop code that runs repeatedly on every tick of the microprocessor clock cycle.
*/
void loop() {
  float v0, v1, v2;

  // Read EMG sensors
  readEMG(v0, v1, v2);
  
  // Condition/ make any normalisation to the 3 emg sensor input.
  // Calibration procedure to remove this fixed offset from the readings. All EMG Voltage lines will oscillate from LOWER THRESHOLD (255) - HIGHER THRESHOLD (1000).
  v0 = constrain(v0, 0, HIGHERTHRESHOLD);
  v1 = constrain(v1, 0, HIGHERTHRESHOLD);
  v2 = constrain(v2, 0, HIGHERTHRESHOLD);
  while (motionDetected(v0, v1, v2) == false)
  {
    // Wait until motion has been detected. Motion should be between LOWER THRESHOLD (255) - HIGHER THRESHOLD (1000).
  }

  // Record emg data into svm format [v0, v1, v2, v0, v1, v2, ... , NUM_SAMPLES'th]
  recordEMG();

  // Use SVM to classify emg dataset
  svmClassify();

  FSM(currentStateMovement);
}

/*
* SVM Function - Analog reads the three EMG data per leg.
* @param v0 float of EMG sensor 1.
* @param v1 float of EMG sensor 2.
* @param v2 float of EMG sensor 3.
*/
void readEMG(float &v0, float &v1,float &v2) {
  // Read each emg sensor and convert to analog signal (0-1023).
  v0 = analogRead(emg0);
  v1 = analogRead(emg1);
  v2 = analogRead(emg2);
}

/*
* SVM Function - Check if the EMG value 
*/
bool motionDetected(float v0, float v1, float v2) 
{
  if (abs(v0) <= LOWERTHRESHOLD || abs(v1) <= LOWERTHRESHOLD || abs(v2) <= LOWERTHRESHOLD)
  {
    return false;
  }
  else
  {
    return true;
  }
}

/*
* SVM Function - Record EMG data of 3 sensors to features array. Constrains values between Lower and Higher Threshold.
*/
void recordEMG() {
    // Record emg data into svm format [v0, v1, v2, v0, v1, v2, ... , NUM_SAMPLESth v3]
    float v0, v1, v2;
    for (int i = 0; i < NUM_SAMPLES; i++) 
    {
        readEMG(v0, v1, v2);
        // Calibration procedure to remove this fixed offset from the readings. All EMG Voltage lines will oscillate from 0.
        v0 = constrain(v0, LOWERTHRESHOLD, HIGHERTHRESHOLD);
        v1 = constrain(v1, LOWERTHRESHOLD, HIGHERTHRESHOLD);
        v2 = constrain(v2, LOWERTHRESHOLD, HIGHERTHRESHOLD);
        
        features[i * NUM_AXES + 0] = v0;
        features[i * NUM_AXES + 1] = v1;
        features[i * NUM_AXES + 2] = v2;
        delay(INTERVAL);
    }
}

/*
* SVM Function - Classify current movement label.
*/
void svmClassify() {
    // Use SVM to predict label for given emg input array.
    Serial.print("Detected gesture: ");
    
    currentStateMovement = String(clf.predictLabel(features)).toInt();
    
    Serial.println(currentStateMovement);
}

/*
* Linear Actuator Function - Finite State Machine that changes position and speed of linear actuator depending on PID controller.
*/
void FSM(int currentStateMovement)
{
  InputH = analogRead(PIN_INH);
  switch (currentStateMovement)
  {
    case 0:           //Standing
      SetpointH = Standing[2];
      PIDCompute();
      //get EMG data and find movement label and change the value of Movement accordinly in order to go to next state
      break;
    case 1:
      SetpointH = HeelStrike[2];
      PIDCompute();
      break;
    case 2:           //LdgResp
      SetpointH = LdgResp[2];
      PIDCompute();
      break;
    case 3:           //MidStance
      SetpointH = MidStance[2];
      PIDCompute();
      break;
    case 4:           //TmlStance
      SetpointH = TmlStance[2];
      PIDCompute();
      break;
    case 5:           //PreSwing
      SetpointH = PreSwing[2];
      PIDCompute();
      break;
    case 6:           //InitSwing
      SetpointH = InitSwing[2];
      PIDCompute();
      break;
    case 7:           //MidSwing
      SetpointH = MidSwing[2];
      PIDCompute();
      break;
    case 8:           //TmlSwing
      SetpointH = TmlSwing[2];
      PIDCompute();
      break;
     default:
      PIDCompute();
    }
}

/*
* Linear Actuator Function - Compute PID values.
*/
void PIDCompute()
{
    pidH.Compute();
    analogWrite(PIN_OUTH, OutputH);
    analogWrite(PIN_OUTK, OutputK);
}
