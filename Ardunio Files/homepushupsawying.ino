#include <Arduino.h>
#include <SMS_STS.h>

// =====================================================
// ESP32 + FE-URT-1 + 12 x STS3215
// Initial HOME pose + CH5 Push-up + CH6 Hip Sway Demo
// =====================================================

// ---------------- UART SETTINGS ----------------
#define SERVO_RX 16
#define SERVO_TX 17
#define SERVO_BAUD 1000000

HardwareSerial ServoSerial(1);
SMS_STS sts;

// ---------------- RC RECEIVER ----------------
#define CH5_PIN 27
#define CH6_PIN 14

// ---------------- MOTOR SETTINGS ----------------
const int NUM_MOTORS = 12;
int motorID[NUM_MOTORS] = {1,2,3,4,5,6,7,8,9,10,11,12};

// ---------------- MOTOR MAPPING ----------------
// Front Right
#define FR_LOWER 1
#define FR_UPPER 2
#define FR_HIP   3
// Front Left
#define FL_HIP   10
#define FL_UPPER 11
#define FL_LOWER 12
// Back Left
#define BL_HIP   4
#define BL_UPPER 6
#define BL_LOWER 5
// Back Right
#define BR_HIP   9
#define BR_UPPER 7
#define BR_LOWER 8

// ---------------- HIP SWAY MOTORS ----------------
// CH6: sway/slender
#define SWAY_HIP_1 FR_HIP // M3
#define SWAY_HIP_2 BL_HIP // M4
#define SWAY_HIP_3 FL_HIP // M10
#define SWAY_HIP_4 BR_HIP // M9

// ---------------- FIXED HOME ----------------
int fixedHomePos[NUM_MOTORS] = {2048,2100,2045,2000,2048,2100,2048,2048,2100,2100,2048,2048};
int homePos[NUM_MOTORS];
int currentCmdPos[NUM_MOTORS];

// ---------------- SETTINGS ----------------
int moveSpeed = 550;
int moveAcc   = 45;
int moveSteps = 30;
int moveDelay = 20;
int pushOffset = 600;
int supportOffset = 220;
int swayOffset = 220;
int maxOffsetLimit = 420;

// Right/Left motor direction correction
int rightSign = 1;
int leftSign = -1;

// =====================================================
// HELPER FUNCTIONS
// =====================================================
int idToIndex(int id) { return id-1; }
int limitPosition(int pos) { return constrain(pos,0,4095); }
int limitOffset(int offset) { return constrain(offset,-maxOffsetLimit,maxOffsetLimit); }
void writeMotorPosition(int id, int targetPos){ targetPos=limitPosition(targetPos); sts.WritePosEx(id,targetPos,moveSpeed,moveAcc);}
int readChannel(int pin){int val=pulseIn(pin,HIGH,25000); return (val<900||val>2100)?1500:val;}

// =====================================================
// HOME FUNCTIONS
// =====================================================
void loadFixedHomePositions(){for(int i=0;i<NUM_MOTORS;i++){homePos[i]=fixedHomePos[i];currentCmdPos[i]=homePos[i];}}
void readCurrentPositions(){for(int i=0;i<NUM_MOTORS;i++){int pos=sts.ReadPos(motorID[i]); if(pos<0||pos>4095) pos=homePos[i]; currentCmdPos[i]=pos;}}
void moveToHome(){for(int step=1;step<=moveSteps;step++){float t=(float)step/moveSteps;for(int i=0;i<NUM_MOTORS;i++){int newPos=currentCmdPos[i]+(homePos[i]-currentCmdPos[i])*t; writeMotorPosition(i+1,newPos);} delay(moveDelay);} for(int i=0;i<NUM_MOTORS;i++) currentCmdPos[i]=homePos[i];}

// =====================================================
// UPPER PAIR FUNCTIONS
// =====================================================
int getRightUpperTarget(int rightMotor,int offset){int idx=idToIndex(rightMotor); return limitPosition(homePos[idx]+rightSign*limitOffset(offset));}
int getLeftUpperTarget(int leftMotor,int offset){int idx=idToIndex(leftMotor); return limitPosition(homePos[idx]+leftSign*limitOffset(offset));}
void moveUpperPairFromHome(int rightUpperMotor,int leftUpperMotor,int offset){
  offset=limitOffset(offset);
  int rightIndex=idToIndex(rightUpperMotor), leftIndex=idToIndex(leftUpperMotor);
  int rightTarget=getRightUpperTarget(rightUpperMotor,offset);
  int leftTarget=getLeftUpperTarget(leftUpperMotor,offset);
  int rightStart=currentCmdPos[rightIndex], leftStart=currentCmdPos[leftIndex];
  for(int step=1;step<=moveSteps;step++){
    float t=(float)step/moveSteps;
    int rightNew=rightStart+(rightTarget-rightStart)*t;
    int leftNew=leftStart+(leftTarget-leftStart)*t;
    writeMotorPosition(rightUpperMotor,rightNew);
    writeMotorPosition(leftUpperMotor,leftNew);
    currentCmdPos[rightIndex]=rightNew;
    currentCmdPos[leftIndex]=leftNew;
    delay(moveDelay);
  }
}

// =====================================================
// ACTIVE PUSH-UP PAIR + SUPPORT PAIR HOLD
// =====================================================
void moveActivePairWithSupport(int activeRight,int activeLeft,int activeOffset,int supportRight,int supportLeft,int supportFixedOffset){
  int aR=idToIndex(activeRight), aL=idToIndex(activeLeft);
  int sR=idToIndex(supportRight), sL=idToIndex(supportLeft);
  int aRTarget=getRightUpperTarget(activeRight,activeOffset);
  int aLTarget=getLeftUpperTarget(activeLeft,activeOffset);
  int sRTarget=getRightUpperTarget(supportRight,supportFixedOffset);
  int sLTarget=getLeftUpperTarget(supportLeft,supportFixedOffset);
  int aRStart=currentCmdPos[aR], aLStart=currentCmdPos[aL];
  for(int step=1;step<=moveSteps;step++){
    float t=(float)step/moveSteps;
    int aRNew=aRStart+(aRTarget-aRStart)*t;
    int aLNew=aLStart+(aLTarget-aLStart)*t;
    writeMotorPosition(activeRight,aRNew);
    writeMotorPosition(activeLeft,aLNew);
    writeMotorPosition(supportRight,sRTarget);
    writeMotorPosition(supportLeft,sLTarget);
    currentCmdPos[aR]=aRNew;
    currentCmdPos[aL]=aLNew;
    currentCmdPos[sR]=sRTarget;
    currentCmdPos[sL]=sLTarget;
    delay(moveDelay);
  }
}

// =====================================================
// FRONT PUSH-UP
// =====================================================
void frontPushUpCycle(){
  int backLowerOffset = supportOffset;
  moveUpperPairFromHome(BR_UPPER,BL_UPPER,backLowerOffset);
  while(readChannel(CH5_PIN)>1700){
    moveActivePairWithSupport(FR_UPPER,FL_UPPER,pushOffset,BR_UPPER,BL_UPPER,backLowerOffset);
    moveActivePairWithSupport(FR_UPPER,FL_UPPER,-pushOffset,BR_UPPER,BL_UPPER,backLowerOffset);
  }
  moveToHome();
}

// =====================================================
// BACK PUSH-UP
// =====================================================
void backPushUpCycle(){
  int frontLowerOffset=-supportOffset;
  moveUpperPairFromHome(FR_UPPER,FL_UPPER,frontLowerOffset);
  while(readChannel(CH5_PIN)<1300){
    moveActivePairWithSupport(BR_UPPER,BL_UPPER,pushOffset,FR_UPPER,FL_UPPER,frontLowerOffset);
    moveActivePairWithSupport(BR_UPPER,BL_UPPER,-pushOffset,FR_UPPER,FL_UPPER,frontLowerOffset);
  }
  moveToHome();
}

// =====================================================
// CH6 HIP SWAY / SLANDER (Pose-based loop)
// M3 & M10 same, M4 & M9 opposite
// =====================================================
void hipSwayCycle(){
  while(readChannel(CH6_PIN) > 1700){
    // --- SWAY RIGHT ---
    moveFourHipsFromHome(FR_HIP, swayOffset, BL_HIP, -swayOffset, FL_HIP, swayOffset, BR_HIP, -swayOffset);
    delay(300);
    moveToHome();
    delay(200);
    // --- SWAY LEFT ---
    moveFourHipsFromHome(FR_HIP, -swayOffset, BL_HIP, swayOffset, FL_HIP, -swayOffset, BR_HIP, swayOffset);
    delay(300);
    moveToHome();
    delay(200);
  }
}

// =====================================================
// FOUR HIP MOVE HELPER
// =====================================================
void moveFourHipsFromHome(int motorA, int offsetA, int motorB, int offsetB, int motorC, int offsetC, int motorD, int offsetD){
  offsetA=limitOffset(offsetA); offsetB=limitOffset(offsetB); offsetC=limitOffset(offsetC); offsetD=limitOffset(offsetD);
  int indexA=idToIndex(motorA), indexB=idToIndex(motorB), indexC=idToIndex(motorC), indexD=idToIndex(motorD);
  int startA=currentCmdPos[indexA], startB=currentCmdPos[indexB], startC=currentCmdPos[indexC], startD=currentCmdPos[indexD];
  int targetA=homePos[indexA]+offsetA, targetB=homePos[indexB]+offsetB, targetC=homePos[indexC]+offsetC, targetD=homePos[indexD]+offsetD;
  for(int step=1;step<=moveSteps;step++){
    float t=(float)step/moveSteps;
    int newA=startA+(targetA-startA)*t;
    int newB=startB+(targetB-startB)*t;
    int newC=startC+(targetC-startC)*t;
    int newD=startD+(targetD-startD)*t;
    writeMotorPosition(motorA,newA); writeMotorPosition(motorB,newB);
    writeMotorPosition(motorC,newC); writeMotorPosition(motorD,newD);
    currentCmdPos[indexA]=newA; currentCmdPos[indexB]=newB;
    currentCmdPos[indexC]=newC; currentCmdPos[indexD]=newD;
    delay(moveDelay);
  }
}

// =====================================================
// SETUP
// =====================================================
void setup(){
  Serial.begin(115200);
  delay(1000);
  pinMode(CH5_PIN,INPUT);
  pinMode(CH6_PIN,INPUT);
  ServoSerial.begin(SERVO_BAUD,SERIAL_8N1,SERVO_RX,SERVO_TX);
  sts.pSerial=&ServoSerial;
  loadFixedHomePositions();
  readCurrentPositions();
  moveToHome(); // Initial home pose
  Serial.println("Robot Dog Demo Ready: HOME set");
}

// =====================================================
// LOOP
// =====================================================
void loop(){
  int ch5=readChannel(CH5_PIN);
  int ch6=readChannel(CH6_PIN);

  Serial.print("CH5=");
  Serial.print(ch5);
  Serial.print(" | CH6=");
  Serial.println(ch6);

  if(ch6>1700) hipSwayCycle();       // CH6 pose-based hip sway
  else if(ch5>1700) frontPushUpCycle(); // CH5 HIGH front push-up
  else if(ch5<1300) backPushUpCycle();  // CH5 LOW back push-up
  else moveToHome();                   // Neutral, return HOME
}