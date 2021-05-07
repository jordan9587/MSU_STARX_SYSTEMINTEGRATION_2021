#include <PID_v1.h>
//set the PID pins for Knee and Hip actuators
#define PIN_INH A0
#define PIN_INK A1
#define PIN_OUTH A2
#define PIN_OUTK A3
#define KP 1200
#define KI 25000
#define KD 10000

// NEEDED: Get Length from Katherine!
float MAXLENGTHHIP = 7.75; // 4 inches.
float MAXLENGTHKNEE = 7.75; // 6 inches.
float StandingHipSetpoint = ((1)/MAXLENGTHHIP)*1023;
float HeelStrikeHipSetpoint = ((2)/MAXLENGTHHIP)*1023;
float LdgRespHipSetpoint = ((3)/MAXLENGTHHIP)*1023;
float MidStanceHipSetpoint = ((4)/MAXLENGTHHIP)*1023;
float TmlStanceHipSetpoint = ((5)/MAXLENGTHHIP)*1023;
float PreSwingHipSetpoint = ((6)/MAXLENGTHHIP)*1023;
float InitSwingHipSetpoint = ((7)/MAXLENGTHHIP)*1023;
float MidSwingHipSetpoint = ((1)/MAXLENGTHHIP)*1023;
float TmlSwingHipSetpoint = ((2)/MAXLENGTHHIP)*1023;

float StandingKneeSetpoint = ((1)/MAXLENGTHHIP)*1023;
float HeelStrikeKneeSetpoint = ((2)/MAXLENGTHHIP)*1023;
float LdgRespKneeSetpoint = ((3)/MAXLENGTHHIP)*1023;
float MidStanceKneeSetpoint = ((4)/MAXLENGTHHIP)*1023;
float TmlStanceKneeSetpoint = ((5)/MAXLENGTHHIP)*1023;
float PreSwingKneeSetpoint = ((6)/MAXLENGTHHIP)*1023;
float InitSwingKneeSetpoint = ((7)/MAXLENGTHHIP)*1023;
float MidSwingKneeSetpoint = ((1)/MAXLENGTHHIP)*1023;
float TmlSwingKneeSetpoint = ((2)/MAXLENGTHHIP)*1023;

//PID object variables
int currentStateMovement = 0;
double SetpointH, InputH, OutputH, SetpointK, InputK, OutputK;

//Movement label values for Hip angle, Knee angle, Hip Setpoint, Knee Setpoint respectively
float Standing[] = { 0, 0, StandingHipSetpoint, StandingKneeSetpoint };      //1
float HeelStrike[] = { 20, 0, HeelStrikeHipSetpoint, HeelStrikeKneeSetpoint };   //2
float LdgResp[] = { 20, 20, LdgRespHipSetpoint, LdgRespKneeSetpoint };    //3
float MidStance[] = { 0, 5, MidStanceHipSetpoint, MidStanceKneeSetpoint };     //4
float TmlStance[] = { -20, 0, TmlStanceHipSetpoint, TmlStanceKneeSetpoint };   //5
float PreSwing[] = { -10, 40, PreSwingHipSetpoint, PreSwingKneeSetpoint };   //6
float InitSwing[] = { 15, 60, InitSwingHipSetpoint, InitSwingKneeSetpoint };   //7
float MidSwing[] = { 25, 25, MidSwingHipSetpoint, MidSwingKneeSetpoint };    //8
float TmlSwing[] = { 20, 0, TmlSwingHipSetpoint, TmlSwingKneeSetpoint };     //9
int arSize = sizeof(Standing) / sizeof(Standing[0]);
int kp = 1200;
int ki = 250000;
int kd = 10000;
PID Hpid(&InputH, &OutputH, &SetpointH, kp, ki, kd, DIRECT);
PID Kpid(&InputK, &OutputK, &SetpointK, kp, ki, kd, DIRECT);
double ErrorH;
double ErrorK;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(PIN_INH, INPUT);
  pinMode(PIN_OUTH, OUTPUT);
  SetpointH = Standing[2];
  Hpid.SetOutputLimits(0,255);
  Kpid.SetOutputLimits(0,255);
  Hpid.SetTunings(KP, KI, KD);
  Kpid.SetTunings(KD, KI, KD);
}

void loop() {
  /* One Dimensional Traversal*/
  testCase1();
  /*
  testCase2();
  testCase3();
  testCase0();

  testCase4();
  testCase5();
  testCase6();
  testCase7();
  testCase8();
  testCase9();

  testCase10();
  */
}

/*
* State machine that iterates through states and set PID parameters accordingly.
* @param currentStateMovement integer represents starting state.
* @param stateBuffer array of integers represents following states after currentStateMovement.
* @param stateBufferSize integer is size of array stateBuffer.
*/
void FSM(int currentStateMovement, int stateBuffer[], int stateBufferSize)
{
  int index = currentStateMovement;
  int arrayIndex = 0;
  switch (index)
  {
    case 0:           //Standing
      InputH = analogRead(PIN_INH);
      InputK = analogRead(PIN_INH);
      //InputK = analogRead(PIN_INK);
      SetpointH = Standing[2];
      SetpointK = Standing[3];
      PIDCompute();
      //get EMG data and find movement label and change the value of Movement acordingly in order to go to next state
      if (arrayIndex == stateBufferSize)
      {
        break;
      }
      else
      {
        index = stateBuffer[arrayIndex];
        arrayIndex += 1;
      }
    case 1:           //HealStrike
      InputH = analogRead(PIN_INH);
      InputK = analogRead(PIN_INH);
      //InputK = analogRead(PIN_INK);
      SetpointH = HeelStrike[2];
      SetpointK = HeelStrike[3];
      PIDCompute();
      //get EMG data and find movement label and change the value of Movement acordingly in order to go to next state
      if (arrayIndex == stateBufferSize)
      {
        break;
      }
      else
      {
        index = stateBuffer[arrayIndex];
        arrayIndex += 1;
      }
    case 2:           //LdgResp
      InputH = analogRead(PIN_INH);
      InputK = analogRead(PIN_INH);
      //InputK = analogRead(PIN_INK);
      SetpointH = LdgResp[2];
      SetpointK = LdgResp[3];
      PIDCompute();
      //get EMG data and find movement label and change the value of Movement acordingly in order to go to next state
      if (arrayIndex == stateBufferSize)
      {
        break;
      }
      else
      {
        index = stateBuffer[arrayIndex];
        arrayIndex += 1;
      }
    case 3:           //MidStance
      InputH = analogRead(PIN_INH);
      InputK = analogRead(PIN_INH);
      //InputK = analogRead(PIN_INK);
      SetpointH = MidStance[2];
      SetpointK = MidStance[3];
      PIDCompute();
      //get EMG data and find movement label and change the value of Movement acordingly in order to go to next state
      if (arrayIndex == stateBufferSize)
      {
        break;
      }
      else
      {
        index = stateBuffer[arrayIndex];
        arrayIndex += 1;
      }
    case 4:           //TmlStance
      InputH = analogRead(PIN_INH);
      InputK = analogRead(PIN_INH);;
      //InputK = analogRead(PIN_INK);
      SetpointH = TmlStance[2];
      SetpointK = TmlStance[3];
      PIDCompute();
      //get EMG data and find movement label and change the value of Movement acordingly in order to go to next state
      if (arrayIndex == stateBufferSize)
      {
        break;
      }
      else
      {
        index = stateBuffer[arrayIndex];
        arrayIndex += 1;
      }
    case 5:           //PreSwing
      InputH = analogRead(PIN_INH);
      InputK = analogRead(PIN_INH);
      //InputK = analogRead(PIN_INK);
      SetpointH = PreSwing[2];
      SetpointK = PreSwing[3];
      PIDCompute();
      //get EMG data and find movement label and change the value of Movement acordingly in order to go to next state
      if (arrayIndex == stateBufferSize)
      {
        break;
      }
      else
      {
        index = stateBuffer[arrayIndex];
        arrayIndex += 1;
      }
    case 6:           //InitSwing
      InputH = analogRead(PIN_INH);
      InputK = analogRead(PIN_INH);
      //InputK = analogRead(PIN_INK);
      SetpointH = InitSwing[2];
      SetpointK = InitSwing[3];
      PIDCompute();
      //get EMG data and find movement label and change the value of Movement acordingly in order to go to next state
      if (arrayIndex == stateBufferSize)
      {
        break;
      }
      else
      {
        index = stateBuffer[arrayIndex];
        arrayIndex += 1;
      }
    case 7:           //MidSwing
      InputH = analogRead(PIN_INH);
      InputK = analogRead(PIN_INH);
      //InputK = analogRead(PIN_INK);
      SetpointH = MidSwing[2];
      SetpointK = MidSwing[3];
      PIDCompute();
      //get EMG data and find movement label and change the value of Movement acordingly in order to go to next state
      if (arrayIndex == stateBufferSize)
      {
        break;
      }
      else
      {
        index = stateBuffer[arrayIndex];
        arrayIndex += 1;
      }
    case 8:           //TmlSwing
      InputH = analogRead(PIN_INH);
      InputK = analogRead(PIN_INH);
      //InputK = analogRead(PIN_INK);
      SetpointH = TmlSwing[2];
      //SetpointK = TmlSwing[3];
      PIDCompute();
      //get EMG data and find movement label and change the value of Movement acordingly in order to go to next state
      if (arrayIndex == stateBufferSize)
      {
        break;
      }
      else
      {
        index = stateBuffer[arrayIndex];
        arrayIndex += 1;
      }
     default:
      printf("Error: Computing Previous parameters for PID.");
      PIDCompute();
    }
}

/*
* Computes the new PID controller output and writes it to output as analog value.
*/
void PIDCompute()
{
    Hpid.Compute();
    Kpid.Compute();
    ErrorH = SetpointH - InputH;
    ErrorK = SetpointK - InputK;
    analogWrite(PIN_OUTH, OutputH);
    analogWrite(PIN_OUTK, OutputK);
}


/*
Test Cases
*/


/* One Dimensional Traversal*/
void testCase0()
{
  printf("testCase0: Error State - State number -2 not found.");
  int stateBuffer[] = {9};
  int stateBufferSize = sizeof(stateBuffer) / sizeof(stateBuffer[0]);
  currentStateMovement = -2;
  FSM(currentStateMovement, stateBuffer, stateBufferSize);
  printf("testCase0 Complete!\n");
}

void testCase1()
{
  printf("testCase1: (1 -> 2 inches).");
  int stateBuffer[] = {1};
  int stateBufferSize = sizeof(stateBuffer) / sizeof(stateBuffer[0]);
  currentStateMovement = 0;
  FSM(currentStateMovement, stateBuffer, stateBufferSize);
  printf("testCase1 Complete!\n");
}

void testCase2()
{
  printf("testCase2: (3 -> 1 inches).");
  int stateBuffer[] = {0};
  int stateBufferSize = sizeof(stateBuffer) / sizeof(stateBuffer[0]);
  currentStateMovement = 2;
  FSM(currentStateMovement, stateBuffer, stateBufferSize);
  printf("testCase2 Complete!\n");
}

void testCase3()
{
  printf("testCase3: (2 -> 2 inches).");
  int stateBuffer[] = {3};
  int stateBufferSize = sizeof(stateBuffer) / sizeof(stateBuffer[0]);
  currentStateMovement = 3;
  FSM(currentStateMovement, stateBuffer, stateBufferSize);
  printf("testCase3 Complete!\n");
}

/* Two Dimensional Traversal*/
void testCase4()
{
  printf("testCase4: (1 -> 6 -> 7 inches).");
  int stateBuffer[] = {5,6};
  int stateBufferSize = sizeof(stateBuffer) / sizeof(stateBuffer[0]);
  currentStateMovement = 1;
  FSM(currentStateMovement, stateBuffer, stateBufferSize);
  printf("testCase4 Complete!\n");
}

void testCase5()
{
  printf("testCase5:  (2 -> 4 -> 2 inches).");
  int stateBuffer[] = {5,1};
  int stateBufferSize = sizeof(stateBuffer) / sizeof(stateBuffer[0]);
  currentStateMovement = 1;
  FSM(currentStateMovement, stateBuffer, stateBufferSize);
  printf("testCase5 Complete!\n");
}

void testCase6()
{
  printf("testCase6: (2 -> 4 -> 7 inches). ");
  int stateBuffer[] = {3,6};
  int stateBufferSize = sizeof(stateBuffer) / sizeof(stateBuffer[0]);
  currentStateMovement = 8;
  FSM(currentStateMovement, stateBuffer, stateBufferSize);
  printf("testCase6 Complete!\n");
}

void testCase7()
{
  printf("testCase7:  (7 -> 1-> 1 inches). ");
  int stateBuffer[] = {7,7};
  int stateBufferSize = sizeof(stateBuffer) / sizeof(stateBuffer[0]);
  currentStateMovement = 6;
  FSM(currentStateMovement, stateBuffer, stateBufferSize);
  printf("testCase7 Complete!\n");
}

void testCase8()
{
  printf("testCase8: (7 -> 1 -> 1 inches). ");
  int stateBuffer[] = {7,7};
  int stateBufferSize = sizeof(stateBuffer) / sizeof(stateBuffer[0]);
  currentStateMovement = 6;
  FSM(currentStateMovement, stateBuffer, stateBufferSize);
  printf("testCase8 Complete!\n");
}

void testCase9()
{
  printf("testCase9: (6 -> 6 -> 6 inches). ");
  int stateBuffer[] = {7,7};
  int stateBufferSize = sizeof(stateBuffer) / sizeof(stateBuffer[0]);
  currentStateMovement = 7;
  FSM(currentStateMovement, stateBuffer, stateBufferSize);
  printf("testCase9 Complete!\n");
}

/* Three+ Dimensional Traversal*/
void testCase10()
{
  printf("testCase10: ( 2 -> 3 -> 6 -> 7 -> 3 -> 3 -> 2 -> 1 -> 4 -> 1 -> 1 inches).");
  int stateBuffer[] = {3,5,7,2,2,8,0,3,7,7};
  int stateBufferSize = sizeof(stateBuffer) / sizeof(stateBuffer[0]);
  currentStateMovement = 1;
  FSM(currentStateMovement, stateBuffer, stateBufferSize);
  printf("testCase10 Complete!\n");
}
