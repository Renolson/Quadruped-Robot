#include <INST.h>
#include <SCS.h>
#include <SCSCL.h>
#include <SCSerial.h>
#include <SCServo.h>
#include <SMS_STS.h>
#include <Arduino.h>
#include <math.h>

// =====================================================
// ESP32 + FE-URT-1 + 12 x STS3215
// Complete Robot Dog Demo Code (Walking + Gestures)
// CH1: Forward / Backward Trot
// CH5: Push-up / Back push-up
// CH6 HIGH: Hip sway
// CH6 LOW: Sitting + Heart / Hi gestures
// =====================================================

// ---------------- UART SETTINGS ----------------
#define SERVO_RX 16
#define SERVO_TX 17
#define SERVO_BAUD 1000000

HardwareSerial ServoSerial(1);
SMS_STS sts;

// ---------------- RC RECEIVER ----------------
#define CH1_PIN 32
#define CH5_PIN 27
#define CH6_PIN 14

// ---------------- MOTOR SETTINGS ----------------
const int NUM_MOTORS = 12;

int motorID[NUM_MOTORS] = {
  1, 2, 3,
  4, 5, 6,
  7, 8, 9,
  10, 11, 12
};

// =====================================================
// REAL MOTOR MAPPING
// =====================================================

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

// =====================================================
// FIXED HOME POSITION
// =====================================================

int fixedHomePos[NUM_MOTORS] = {
  2000, 2110, 2045,
  2000, 1885, 2120,
  2048, 2075, 2100,
  2100, 2048, 2048
};

int homePos[NUM_MOTORS];
int currentCmdPos[NUM_MOTORS];

// =====================================================
// SITTING POSES
// =====================================================

// Go here first when sitting
int intermediateSitPose[NUM_MOTORS] = {
  2549, 2596, 2076,
  2019, 2513, 2557,
  1526, 1442, 2048,
  2118, 1597, 1516
};

// Final sitting pose
int sittingPose[NUM_MOTORS] = {
  2349, 2567, 2098,
  1985, 1944, 2738,
  1326, 1978, 2095,
  2060, 1309, 1679
};

// Use this when returning from sitting
int intermediateReturnPose[NUM_MOTORS] = {
  1838, 2604, 2128,
  1955, 2195, 2961,
  1115, 1741, 2081,
  2105, 1645, 2216
};

// =====================================================
// SERVO & MOVEMENT SETTINGS
// =====================================================

int moveSpeed = 550;
int moveAcc   = 45;

int moveSteps = 30;
int moveDelay = 20;

int pushOffset = 600;
int supportOffset = 220;
int swayOffset = 220;
int maxOffsetLimit = 420;

// Right/left correction for upper leg pair movement
int rightSign = 1;
int leftSign  = -1;

// =====================================================
// DINGO-BASED TRAJECTORY TUNING PARAMETERS (IK)
// =====================================================
const float LINK_UPPER_MM = 40.0;
const float LINK_LOWER_MM = 40.0;
const float HOME_FOOT_X_MM = -20.0;
const float HOME_FOOT_Z_MM = -60.0;
const float STEP_LENGTH_MM = 15.0; 
const float STEP_HEIGHT_MM = 8.0; 
const float CYCLE_TIME_MS  = 2000.0; 
const int   GAIT_DT_MS     = 25;     
const float SERVO_COUNT_PER_DEG = 4096.0 / 360.0;

int SIGN_FR_UPPER =  1;
int SIGN_FR_LOWER =  1;
int SIGN_FL_UPPER = -1;
int SIGN_FL_LOWER = -1;
int SIGN_BR_UPPER =  1;
int SIGN_BR_LOWER =  1;
int SIGN_BL_UPPER = -1;
int SIGN_BL_LOWER = -1;

unsigned long lastPrintMs = 0;

struct IK_Result {
  int upperCount;
  int lowerCount;
  bool valid;
};

// =====================================================
// HELPER FUNCTIONS
// =====================================================

int idToIndex(int id) {
  return id - 1;
}

int limitPosition(int pos) {
  if (pos < 0) return 0;
  if (pos > 4095) return 4095;
  return pos;
}

int limitOffset(int offset) {
  if (offset > maxOffsetLimit) return maxOffsetLimit;
  if (offset < -maxOffsetLimit) return -maxOffsetLimit;
  return offset;
}

void writeMotorPosition(int id, int targetPos) {
  targetPos = limitPosition(targetPos);
  sts.WritePosEx(id, targetPos, moveSpeed, moveAcc);
}

int readChannelRaw(int pin) {
  return pulseIn(pin, HIGH, 25000);
}

int readChannel(int pin) {
  int value = readChannelRaw(pin);
  if (value < 900 || value > 2100) {
    return 1500;
  }
  return value;
}

void copyPose(int sourcePose[NUM_MOTORS], int targetPose[NUM_MOTORS]) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    targetPose[i] = sourcePose[i];
  }
}

// =====================================================
// HOME FUNCTIONS
// =====================================================

void loadFixedHomePositions() {
  Serial.println("Loading fixed HOME positions...");
  for (int i = 0; i < NUM_MOTORS; i++) {
    homePos[i] = fixedHomePos[i];
  }
}

void readCurrentPositions() {
  Serial.println("Reading current motor positions...");
  for (int i = 0; i < NUM_MOTORS; i++) {
    int id = motorID[i];
    int pos = sts.ReadPos(id);

    if (pos < 0 || pos > 4095) {
      pos = homePos[i];
    }
    currentCmdPos[i] = pos;
    delay(15);
  }
}

void moveToHome() {
  int startPose[NUM_MOTORS];
  for (int i = 0; i < NUM_MOTORS; i++) {
    startPose[i] = currentCmdPos[i];
  }

  for (int step = 1; step <= moveSteps; step++) {
    float t = (float)step / (float)moveSteps;

    for (int i = 0; i < NUM_MOTORS; i++) {
      int newPos = startPose[i] + (homePos[i] - startPose[i]) * t;
      newPos = limitPosition(newPos);
      writeMotorPosition(motorID[i], newPos);
      currentCmdPos[i] = newPos;
    }
    delay(moveDelay);
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    currentCmdPos[i] = homePos[i];
  }
}

// =====================================================
// INVERSE KINEMATICS & TRAJECTORY (TROT)
// =====================================================

IK_Result calculateIK(float x, float z, int homeUpper, int homeLower, int signUpper, int signLower) {
  IK_Result res;
  res.valid = false;

  float d_sq = x*x + z*z;
  float d = sqrt(d_sq);

  if (d > (LINK_UPPER_MM + LINK_LOWER_MM) || d < fabs(LINK_UPPER_MM - LINK_LOWER_MM)) {
    return res; 
  }

  float cos_lower = (d_sq - LINK_UPPER_MM*LINK_UPPER_MM - LINK_LOWER_MM*LINK_LOWER_MM) / (2.0 * LINK_UPPER_MM * LINK_LOWER_MM);
  float alpha_lower = acos(cos_lower); 

  float alpha_1 = atan2(x, -z);
  float cos_alpha2 = (LINK_UPPER_MM*LINK_UPPER_MM + d_sq - LINK_LOWER_MM*LINK_LOWER_MM) / (2.0 * LINK_UPPER_MM * d);
  float alpha_2 = acos(cos_alpha2);
  float alpha_upper = alpha_1 + alpha_2; 

  float d_sq_home = HOME_FOOT_X_MM*HOME_FOOT_X_MM + HOME_FOOT_Z_MM*HOME_FOOT_Z_MM;
  float d_home = sqrt(d_sq_home);
  float cos_lower_home = (d_sq_home - LINK_UPPER_MM*LINK_UPPER_MM - LINK_LOWER_MM*LINK_LOWER_MM) / (2.0 * LINK_UPPER_MM * LINK_LOWER_MM);
  float alpha_lower_home = acos(cos_lower_home);
  float alpha_1_home = atan2(HOME_FOOT_X_MM, -HOME_FOOT_Z_MM);
  float cos_alpha2_home = (LINK_UPPER_MM*LINK_UPPER_MM + d_sq_home - LINK_LOWER_MM*LINK_LOWER_MM) / (2.0 * LINK_UPPER_MM * d_home);
  float alpha_2_home = acos(cos_alpha2_home);
  float alpha_upper_home = alpha_1_home + alpha_2_home;

  float delta_upper_deg = (alpha_upper - alpha_upper_home) * 180.0 / M_PI;
  float delta_lower_deg = (alpha_lower - alpha_lower_home) * 180.0 / M_PI;

  int upperOffsetTicks = round(delta_upper_deg * SERVO_COUNT_PER_DEG);
  int lowerOffsetTicks = round(delta_lower_deg * SERVO_COUNT_PER_DEG);

  res.upperCount = limitPosition(homeUpper + (signUpper * upperOffsetTicks));
  res.lowerCount = limitPosition(homeLower + (signLower * lowerOffsetTicks));
  res.valid = true;
  return res;
}

void getTrajectoryPoint(float phase, float &x, float &z) {
  float halfLength = STEP_LENGTH_MM / 2.0;

  if (phase < 0.5) {
    float t_swing = phase / 0.5; 
    float x_start = halfLength; 
    float x_end   = -halfLength;
    
    x = HOME_FOOT_X_MM + x_start + (x_end - x_start) * t_swing;
    z = HOME_FOOT_Z_MM + STEP_HEIGHT_MM * sin(t_swing * M_PI);
  } else {
    float t_stance = (phase - 0.5) / 0.5;
    float x_start = -halfLength;
    float x_end   = halfLength;
    
    x = HOME_FOOT_X_MM + x_start + (x_end - x_start) * t_stance;
    z = HOME_FOOT_Z_MM;
  }
}

void executeTrajectoryTrot(int walkDirection) {
  unsigned long strideStartMs = millis();
  
  while (true) {
    int ch1 = readChannel(CH1_PIN);
    int ch5 = readChannel(CH5_PIN);
    int ch6 = readChannel(CH6_PIN);
    
    if (walkDirection > 0 && ch1 < 1700) break;
    if (walkDirection < 0 && ch1 > 1300) break;
    if (ch5 > 1700 || ch5 < 1300 || ch6 > 1700 || ch6 < 1300) break;

    unsigned long elapsedMs = millis() - strideStartMs;
    float basePhase = fmod((float)elapsedMs / CYCLE_TIME_MS, 1.0);

    float phase_FR_BL;
    float phase_FL_BR;

    if (walkDirection < 0) {
      phase_FR_BL = basePhase;
      phase_FL_BR = fmod(basePhase + 0.5, 1.0);
    } else {
      phase_FR_BL = fmod(1.0 - basePhase, 1.0);
      phase_FL_BR = fmod(phase_FR_BL + 0.5, 1.0);
    }

    float x_FR_BL, z_FR_BL;
    float x_FL_BR, z_FL_BR;

    getTrajectoryPoint(phase_FR_BL, x_FR_BL, z_FR_BL);
    getTrajectoryPoint(phase_FL_BR, x_FL_BR, z_FL_BR);

    IK_Result fr = calculateIK(x_FR_BL, z_FR_BL, homePos[idToIndex(FR_UPPER)], homePos[idToIndex(FR_LOWER)], SIGN_FR_UPPER, SIGN_FR_LOWER);
    IK_Result bl = calculateIK(x_FR_BL, z_FR_BL, homePos[idToIndex(BL_UPPER)], homePos[idToIndex(BL_LOWER)], SIGN_BL_UPPER, SIGN_BL_LOWER);
    IK_Result fl = calculateIK(x_FL_BR, z_FL_BR, homePos[idToIndex(FL_UPPER)], homePos[idToIndex(FL_LOWER)], SIGN_FL_UPPER, SIGN_FL_LOWER);
    IK_Result br = calculateIK(x_FL_BR, z_FL_BR, homePos[idToIndex(BR_UPPER)], homePos[idToIndex(BR_LOWER)], SIGN_BR_UPPER, SIGN_BR_LOWER);

    if (fr.valid && bl.valid && fl.valid && br.valid) {
      writeMotorPosition(FR_UPPER, fr.upperCount);
      writeMotorPosition(FR_LOWER, fr.lowerCount);
      writeMotorPosition(BL_UPPER, bl.upperCount);
      writeMotorPosition(BL_LOWER, bl.lowerCount);

      writeMotorPosition(FL_UPPER, fl.upperCount);
      writeMotorPosition(FL_LOWER, fl.lowerCount);
      writeMotorPosition(BR_UPPER, br.upperCount);
      writeMotorPosition(BR_LOWER, br.lowerCount);

      currentCmdPos[idToIndex(FR_UPPER)] = fr.upperCount;
      currentCmdPos[idToIndex(FR_LOWER)] = fr.lowerCount;
      currentCmdPos[idToIndex(BL_UPPER)] = bl.upperCount;
      currentCmdPos[idToIndex(BL_LOWER)] = bl.lowerCount;
      currentCmdPos[idToIndex(FL_UPPER)] = fl.upperCount;
      currentCmdPos[idToIndex(FL_LOWER)] = fl.lowerCount;
      currentCmdPos[idToIndex(BR_UPPER)] = br.upperCount;
      currentCmdPos[idToIndex(BR_LOWER)] = br.lowerCount;
    }

    delay(GAIT_DT_MS);
  }
  
  moveToHome();
}

// =====================================================
// CUSTOM POSE MOVEMENT
// =====================================================

void moveToCustomPoseSmooth(int targetPose[NUM_MOTORS], int steps, int stepDelay) {
  int startPose[NUM_MOTORS];
  for (int i = 0; i < NUM_MOTORS; i++) {
    startPose[i] = currentCmdPos[i];
  }

  for (int step = 1; step <= steps; step++) {
    float t = (float)step / (float)steps;
    for (int i = 0; i < NUM_MOTORS; i++) {
      int newPos = startPose[i] + (targetPose[i] - startPose[i]) * t;
      newPos = limitPosition(newPos);
      writeMotorPosition(motorID[i], newPos);
      currentCmdPos[i] = newPos;
    }
    delay(stepDelay);
  }
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    currentCmdPos[i] = targetPose[i];
  }
}

void holdPose(int targetPose[NUM_MOTORS]) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    writeMotorPosition(motorID[i], targetPose[i]);
    currentCmdPos[i] = targetPose[i];
  }
}

// =====================================================
// RECOVERY SEQUENCES 
// =====================================================

void moveSelectedMotorsToHome(int selectedMotors[], int count, int steps, int stepDelay) {
  int startPos[12];
  int targetPos[12];

  for (int i = 0; i < count; i++) {
    int id = selectedMotors[i];
    int index = idToIndex(id);
    startPos[i] = currentCmdPos[index];
    targetPos[i] = homePos[index];
  }

  for (int step = 1; step <= steps; step++) {
    float t = (float)step / (float)steps;
    for (int i = 0; i < count; i++) {
      int id = selectedMotors[i];
      int index = idToIndex(id);
      int newPos = startPos[i] + (targetPos[i] - startPos[i]) * t;
      newPos = limitPosition(newPos);
      writeMotorPosition(id, newPos);
      currentCmdPos[index] = newPos;
    }
    delay(stepDelay);
  }

  for (int i = 0; i < count; i++) {
    int index = idToIndex(selectedMotors[i]);
    currentCmdPos[index] = homePos[index];
  }
}

void moveBack5678ToRecoveryPose(int m5Target, int m6Target, int m7Target, int m8Target, int steps, int stepDelay) {
  int ids[4] = {5, 6, 7, 8};
  int targets[4] = {m5Target, m6Target, m7Target, m8Target};
  int starts[4];

  for (int i = 0; i < 4; i++) {
    int index = idToIndex(ids[i]);
    starts[i] = currentCmdPos[index];
    targets[i] = limitPosition(targets[i]);
  }

  for (int step = 1; step <= steps; step++) {
    float t = (float)step / (float)steps;
    for (int i = 0; i < 4; i++) {
      int id = ids[i];
      int index = idToIndex(id);
      int newPos = starts[i] + (targets[i] - starts[i]) * t;
      newPos = limitPosition(newPos);
      writeMotorPosition(id, newPos);
      currentCmdPos[index] = newPos;
    }
    delay(stepDelay);
  }

  for (int i = 0; i < 4; i++) {
    currentCmdPos[idToIndex(ids[i])] = targets[i];
  }
}

void back5678RecoverySequence() {
  moveBack5678ToRecoveryPose(2140, 2931, 1122, 1780, 28, 16);
  moveBack5678ToRecoveryPose(2290, 2733, 1322, 1630, 28, 16);
  moveBack5678ToRecoveryPose(2370, 2520, 1535, 1550, 28, 16);
  moveBack5678ToRecoveryPose(2290, 2420, 1635, 1630, 28, 16);
  moveBack5678ToRecoveryPose(2160, 2320, 1735, 1684, 28, 16);
}

// =====================================================
// UPPER LEG & PUSH-UP FUNCTIONS
// =====================================================

int getRightUpperTarget(int rightMotor, int offset) {
  int idx = idToIndex(rightMotor);
  return limitPosition(homePos[idx] + rightSign * limitOffset(offset));
}

int getLeftUpperTarget(int leftMotor, int offset) {
  int idx = idToIndex(leftMotor);
  return limitPosition(homePos[idx] + leftSign * limitOffset(offset));
}

void moveUpperPairFromHome(int rightUpperMotor, int leftUpperMotor, int offset) {
  offset = limitOffset(offset);
  int rightIndex = idToIndex(rightUpperMotor);
  int leftIndex  = idToIndex(leftUpperMotor);

  int rightTarget = getRightUpperTarget(rightUpperMotor, offset);
  int leftTarget  = getLeftUpperTarget(leftUpperMotor, offset);

  int rightStart = currentCmdPos[rightIndex];
  int leftStart  = currentCmdPos[leftIndex];

  for (int step = 1; step <= moveSteps; step++) {
    float t = (float)step / (float)moveSteps;
    int rightNew = limitPosition(rightStart + (rightTarget - rightStart) * t);
    int leftNew  = limitPosition(leftStart  + (leftTarget  - leftStart)  * t);

    writeMotorPosition(rightUpperMotor, rightNew);
    writeMotorPosition(leftUpperMotor, leftNew);

    currentCmdPos[rightIndex] = rightNew;
    currentCmdPos[leftIndex]  = leftNew;
    delay(moveDelay);
  }
}

void moveActivePairWithSupport(int activeRight, int activeLeft, int activeOffset, int supportRight, int supportLeft, int supportFixedOffset) {
  int activeRightIndex = idToIndex(activeRight);
  int activeLeftIndex  = idToIndex(activeLeft);
  int supportRightIndex = idToIndex(supportRight);
  int supportLeftIndex  = idToIndex(supportLeft);

  int activeRightTarget = getRightUpperTarget(activeRight, activeOffset);
  int activeLeftTarget  = getLeftUpperTarget(activeLeft, activeOffset);
  int supportRightTarget = getRightUpperTarget(supportRight, supportFixedOffset);
  int supportLeftTarget  = getLeftUpperTarget(supportLeft, supportFixedOffset);

  int activeRightStart = currentCmdPos[activeRightIndex];
  int activeLeftStart  = currentCmdPos[activeLeftIndex];

  for (int step = 1; step <= moveSteps; step++) {
    float t = (float)step / (float)moveSteps;
    int activeRightNew = limitPosition(activeRightStart + (activeRightTarget - activeRightStart) * t);
    int activeLeftNew  = limitPosition(activeLeftStart  + (activeLeftTarget  - activeLeftStart)  * t);

    writeMotorPosition(activeRight, activeRightNew);
    writeMotorPosition(activeLeft, activeLeftNew);
    writeMotorPosition(supportRight, supportRightTarget);
    writeMotorPosition(supportLeft, supportLeftTarget);

    currentCmdPos[activeRightIndex] = activeRightNew;
    currentCmdPos[activeLeftIndex]  = activeLeftNew;
    currentCmdPos[supportRightIndex] = supportRightTarget;
    currentCmdPos[supportLeftIndex]  = supportLeftTarget;
    delay(moveDelay);
  }
}

void frontPushUpCycle() {
  int backLowerOffset = supportOffset;
  moveUpperPairFromHome(BR_UPPER, BL_UPPER, backLowerOffset);

  while (readChannel(CH5_PIN) > 1700) {
    moveActivePairWithSupport(FR_UPPER, FL_UPPER, pushOffset, BR_UPPER, BL_UPPER, backLowerOffset);
    moveActivePairWithSupport(FR_UPPER, FL_UPPER, -pushOffset, BR_UPPER, BL_UPPER, backLowerOffset);
  }
  moveToHome();
}

void backPushUpCycle() {
  int frontLowerOffset = -supportOffset;
  moveUpperPairFromHome(FR_UPPER, FL_UPPER, frontLowerOffset);

  while (readChannel(CH5_PIN) < 1300) {
    moveActivePairWithSupport(BR_UPPER, BL_UPPER, pushOffset, FR_UPPER, FL_UPPER, frontLowerOffset);
    moveActivePairWithSupport(BR_UPPER, BL_UPPER, -pushOffset, FR_UPPER, FL_UPPER, frontLowerOffset);
  }
  moveToHome();
}

// =====================================================
// HIP SWAY FUNCTION
// =====================================================

void moveFourHipsFromHome(int motorA, int offsetA, int motorB, int offsetB, int motorC, int offsetC, int motorD, int offsetD) {
  offsetA = limitOffset(offsetA); offsetB = limitOffset(offsetB);
  offsetC = limitOffset(offsetC); offsetD = limitOffset(offsetD);

  int indexA = idToIndex(motorA); int indexB = idToIndex(motorB);
  int indexC = idToIndex(motorC); int indexD = idToIndex(motorD);

  int startA = currentCmdPos[indexA]; int startB = currentCmdPos[indexB];
  int startC = currentCmdPos[indexC]; int startD = currentCmdPos[indexD];

  int targetA = limitPosition(homePos[indexA] + offsetA);
  int targetB = limitPosition(homePos[indexB] + offsetB);
  int targetC = limitPosition(homePos[indexC] + offsetC);
  int targetD = limitPosition(homePos[indexD] + offsetD);

  for (int step = 1; step <= moveSteps; step++) {
    float t = (float)step / (float)moveSteps;

    int newA = limitPosition(startA + (targetA - startA) * t);
    int newB = limitPosition(startB + (targetB - startB) * t);
    int newC = limitPosition(startC + (targetC - startC) * t);
    int newD = limitPosition(startD + (targetD - startD) * t);

    writeMotorPosition(motorA, newA); writeMotorPosition(motorB, newB);
    writeMotorPosition(motorC, newC); writeMotorPosition(motorD, newD);

    currentCmdPos[indexA] = newA; currentCmdPos[indexB] = newB;
    currentCmdPos[indexC] = newC; currentCmdPos[indexD] = newD;
    delay(moveDelay);
  }
}

void hipSwayCycle() {
  while (readChannel(CH6_PIN) > 1700) {
    moveFourHipsFromHome(FR_HIP, swayOffset, BL_HIP, -swayOffset, FL_HIP, swayOffset, BR_HIP, -swayOffset);
    delay(300);
    moveToHome();
    delay(200);

    if (readChannel(CH6_PIN) <= 1700) break;

    moveFourHipsFromHome(FR_HIP, -swayOffset, BL_HIP, swayOffset, FL_HIP, -swayOffset, BR_HIP, swayOffset);
    delay(300);
    moveToHome();
    delay(200);
  }
  moveToHome();
}

// =====================================================
// SITTING GESTURES
// =====================================================

void sittingHeartGesture() {
  int heartOpen[NUM_MOTORS];
  int heartClose[NUM_MOTORS];

  copyPose(sittingPose, heartOpen);
  copyPose(sittingPose, heartClose);

  heartOpen[idToIndex(FR_HIP)]   = sittingPose[idToIndex(FR_HIP)] + 120;
  heartOpen[idToIndex(FR_UPPER)] = sittingPose[idToIndex(FR_UPPER)] - 180;
  heartOpen[idToIndex(FR_LOWER)] = sittingPose[idToIndex(FR_LOWER)] + 120;

  heartOpen[idToIndex(FL_HIP)]   = sittingPose[idToIndex(FL_HIP)] - 120;
  heartOpen[idToIndex(FL_UPPER)] = sittingPose[idToIndex(FL_UPPER)] + 180;
  heartOpen[idToIndex(FL_LOWER)] = sittingPose[idToIndex(FL_LOWER)] - 120;

  heartClose[idToIndex(FR_HIP)]   = sittingPose[idToIndex(FR_HIP)] - 120;
  heartClose[idToIndex(FR_UPPER)] = sittingPose[idToIndex(FR_UPPER)] - 260;
  heartClose[idToIndex(FR_LOWER)] = sittingPose[idToIndex(FR_LOWER)] + 220;

  heartClose[idToIndex(FL_HIP)]   = sittingPose[idToIndex(FL_HIP)] + 120;
  heartClose[idToIndex(FL_UPPER)] = sittingPose[idToIndex(FL_UPPER)] + 260;
  heartClose[idToIndex(FL_LOWER)] = sittingPose[idToIndex(FL_LOWER)] - 220;

  moveToCustomPoseSmooth(heartOpen, 25, 12);
  delay(150);
  moveToCustomPoseSmooth(heartClose, 25, 12);
  delay(300);
  moveToCustomPoseSmooth(sittingPose, 25, 12);
}

void sittingHiGesture() {
  int hiUp[NUM_MOTORS];
  int hiLeft[NUM_MOTORS];
  int hiRight[NUM_MOTORS];

  copyPose(sittingPose, hiUp);
  copyPose(sittingPose, hiLeft);
  copyPose(sittingPose, hiRight);

  hiUp[idToIndex(FL_HIP)]   = sittingPose[idToIndex(FL_HIP)] + 180;
  hiUp[idToIndex(FL_UPPER)] = sittingPose[idToIndex(FL_UPPER)] + 280;
  hiUp[idToIndex(FL_LOWER)] = sittingPose[idToIndex(FL_LOWER)] - 220;

  hiLeft[idToIndex(FL_HIP)]   = sittingPose[idToIndex(FL_HIP)] - 120;
  hiLeft[idToIndex(FL_UPPER)] = sittingPose[idToIndex(FL_UPPER)] + 280;
  hiLeft[idToIndex(FL_LOWER)] = sittingPose[idToIndex(FL_LOWER)] - 220;

  hiRight[idToIndex(FL_HIP)]   = sittingPose[idToIndex(FL_HIP)] + 220;
  hiRight[idToIndex(FL_UPPER)] = sittingPose[idToIndex(FL_UPPER)] + 280;
  hiRight[idToIndex(FL_LOWER)] = sittingPose[idToIndex(FL_LOWER)] - 220;

  moveToCustomPoseSmooth(hiUp, 25, 12);
  delay(150);
  moveToCustomPoseSmooth(hiLeft, 18, 10);
  delay(100);
  moveToCustomPoseSmooth(hiRight, 18, 10);
  delay(100);
  moveToCustomPoseSmooth(hiLeft, 18, 10);
  delay(100);
  moveToCustomPoseSmooth(hiRight, 18, 10);
  delay(100);
  moveToCustomPoseSmooth(sittingPose, 25, 12);
}

void hipSitSmooth() {
  moveToCustomPoseSmooth(intermediateSitPose, 50, 22);
  delay(250);
  moveToCustomPoseSmooth(sittingPose, 55, 24);

  while (readChannel(CH6_PIN) < 1300) {
    int ch5 = readChannel(CH5_PIN);
    if (ch5 > 1700) {
      sittingHeartGesture();
    } else if (ch5 < 1300) {
      sittingHiGesture();
    } else {
      holdPose(sittingPose);
      delay(100);
    }
  }

  moveToCustomPoseSmooth(intermediateReturnPose, 45, 20);
  delay(200);

  back5678RecoverySequence();
  delay(150);

  int backLowerPair[] = {BL_LOWER, BR_LOWER};
  moveSelectedMotorsToHome(backLowerPair, 2, 45, 20);
  delay(150);

  int backUpperPair[] = {BL_UPPER, BR_UPPER};
  moveSelectedMotorsToHome(backUpperPair, 2, 45, 20);
  delay(150);

  int backHipPair[] = {BL_HIP, BR_HIP};
  moveSelectedMotorsToHome(backHipPair, 2, 35, 18);
  delay(150);

  int frontLegs[] = {FR_HIP, FR_UPPER, FR_LOWER, FL_HIP, FL_UPPER, FL_LOWER};
  moveSelectedMotorsToHome(frontLegs, 6, 45, 20);
  delay(200);
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(CH1_PIN, INPUT);
  pinMode(CH5_PIN, INPUT);
  pinMode(CH6_PIN, INPUT);

  ServoSerial.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX, SERVO_TX);
  sts.pSerial = &ServoSerial;

  Serial.println("======================================");
  Serial.println("Robot Dog Complete Demo Ready");
  Serial.println("======================================");

  loadFixedHomePositions();
  readCurrentPositions();
  moveToHome();

  Serial.println("HOME set. Ready for RC commands.");
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  int ch1Raw = readChannelRaw(CH1_PIN);
  int ch5Raw = readChannelRaw(CH5_PIN);
  int ch6Raw = readChannelRaw(CH6_PIN);
  
  int ch1 = (ch1Raw >= 900 && ch1Raw <= 2100) ? ch1Raw : 1500;
  int ch5 = (ch5Raw >= 900 && ch5Raw <= 2100) ? ch5Raw : 1500;
  int ch6 = (ch6Raw >= 900 && ch6Raw <= 2100) ? ch6Raw : 1500;

  if (millis() - lastPrintMs > 350) {
    lastPrintMs = millis();
    Serial.print("CH1="); Serial.print(ch1);
    Serial.print("  CH5="); Serial.print(ch5);
    Serial.print("  CH6="); Serial.println(ch6);
  }

  // Hierarchy of movement commands
  if (ch1 > 1700) {
    executeTrajectoryTrot(1); 
  }
  else if (ch1 < 1300) {
    executeTrajectoryTrot(-1); 
  }
  else if (ch6 > 1700) {
    hipSwayCycle();
  }
  else if (ch6 < 1300) {
    hipSitSmooth();
  }
  else if (ch5 > 1700) {
    frontPushUpCycle();
  }
  else if (ch5 < 1300) {
    backPushUpCycle();
  }
  else {
    // Small delay to prevent tight-looping while reading pulseIn
    delay(20);
  }
}