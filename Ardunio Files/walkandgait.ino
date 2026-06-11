#include <INST.h>
#include <SCS.h>
#include <SCSCL.h>
#include <SCSerial.h>
#include <SCServo.h>
#include <SMS_STS.h>
#include <Arduino.h>
#include <math.h>

#define SERVO_RX 16
#define SERVO_TX 17
#define SERVO_BAUD 1000000

HardwareSerial ServoSerial(1);
SMS_STS sts;

#define CH1_PIN 32      
#define CH5_PIN 27      
#define CH6_PIN 14      

const int NUM_MOTORS = 12;
int motorID[NUM_MOTORS] = {1,2,3,4,5,6,7,8,9,10,11,12};

// =====================================================
// EXACT PROTOTYPE INSTALLATION MAPPING
// =====================================================
#define FL_LOWER 1
#define FL_UPPER 2
#define FL_HIP   3

#define FR_HIP   10
#define FR_UPPER 11
#define FR_LOWER 12

#define BR_HIP   9
#define BR_UPPER 7
#define BR_LOWER 8

#define BL_HIP   4
#define BL_UPPER 6
#define BL_LOWER 5

int fixedHomePos[NUM_MOTORS] = {
  2000, 2110, 2045,
  2000, 1885, 2120,
  2048, 2100, 2020,
  2150, 2048, 2060
};
int currentCmdPos[NUM_MOTORS];

int moveSpeed = 0; 
int moveAcc   = 0; 
int moveSteps = 30;
int moveDelay = 20;

int pushOffset = 600;
int supportOffset = 220;
int maxOffsetLimit = 420;

// =====================================================
// DINGO-BASED TRAJECTORY TUNING PARAMETERS
// =====================================================
const float LINK_UPPER_MM = 40.0;
const float LINK_LOWER_MM = 40.0;
const float HOME_FOOT_X_MM = -20.0;
const float HOME_FOOT_Z_MM = -60.0;
const float STEP_LENGTH_MM = 15.0; 
const float STEP_HEIGHT_MM = 8.0; 
const float CYCLE_TIME_MS  = 2000.0; // Total time for one complete stride loop
const int   GAIT_DT_MS     = 25;     // Discrete clock update rate (Delta time)
const float SERVO_COUNT_PER_DEG = 4096.0 / 360.0;

int SIGN_FR_UPPER =  1;
int SIGN_FR_LOWER =  1;
int SIGN_FL_UPPER = -1;
int SIGN_FL_LOWER = -1;
int SIGN_BR_UPPER =  1;
int SIGN_BR_LOWER =  1;
int SIGN_BL_UPPER = -1;
int SIGN_BL_LOWER = -1;

int rightSign = 1;
int leftSign  = -1;

unsigned long lastPrintMs = 0;

struct IK_Result {
  int upperCount;
  int lowerCount;
  bool valid;
};

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
  if (value < 900 || value > 2100) return 1500;
  return value;
}

void loadFixedHomePositions() {
  Serial.println("Loading fixed HOME positions...");
  for (int i = 0; i < NUM_MOTORS; i++) {
    currentCmdPos[i] = fixedHomePos[i];
  }
}

void readCurrentPositions() {
  Serial.println("Reading current motor positions...");
  for (int i = 0; i < NUM_MOTORS; i++) {
    int id = motorID[i];
    int pos = sts.ReadPos(id);
    if (pos < 0 || pos > 4095) {
      pos = fixedHomePos[i];
    }
    currentCmdPos[i] = pos;
    delay(15);
  }
}

void moveToHome() {
  Serial.println("Moving to HOME smoothly...");
  for (int step = 1; step <= moveSteps; step++) {
    float t = (float)step / (float)moveSteps;
    for (int i = 0; i < NUM_MOTORS; i++) {
      int startPos = currentCmdPos[i];
      int targetPos = fixedHomePos[i];
      int newPos = startPos + (targetPos - startPos) * t;
      writeMotorPosition(i + 1, newPos);
    }
    delay(moveDelay);
  }
  for (int i = 0; i < NUM_MOTORS; i++) currentCmdPos[i] = fixedHomePos[i];
}

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
  } 
  else {
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
    if (ch5 > 1700 || ch5 < 1300) break;
    if (ch6 > 1700 || ch6 < 1300) break;

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

    IK_Result fr = calculateIK(x_FR_BL, z_FR_BL, fixedHomePos[idToIndex(FR_UPPER)], fixedHomePos[idToIndex(FR_LOWER)], SIGN_FR_UPPER, SIGN_FR_LOWER);
    IK_Result bl = calculateIK(x_FR_BL, z_FR_BL, fixedHomePos[idToIndex(BL_UPPER)], fixedHomePos[idToIndex(BL_LOWER)], SIGN_BL_UPPER, SIGN_BL_LOWER);
    IK_Result fl = calculateIK(x_FL_BR, z_FL_BR, fixedHomePos[idToIndex(FL_UPPER)], fixedHomePos[idToIndex(FL_LOWER)], SIGN_FL_UPPER, SIGN_FL_LOWER);
    IK_Result br = calculateIK(x_FL_BR, z_FL_BR, fixedHomePos[idToIndex(BR_UPPER)], fixedHomePos[idToIndex(BR_LOWER)], SIGN_BR_UPPER, SIGN_BR_LOWER);

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

int getRightUpperTarget(int rightMotor, int offset) {
  int index = idToIndex(rightMotor);
  offset = limitOffset(offset);
  return limitPosition(fixedHomePos[index] + rightSign * offset);
}

int getLeftUpperTarget(int leftMotor, int offset) {
  int index = idToIndex(leftMotor);
  offset = limitOffset(offset);
  return limitPosition(fixedHomePos[index] + leftSign * offset);
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
    int rightNew = rightStart + (rightTarget - rightStart) * t;
    int leftNew  = leftStart  + (leftTarget  - leftStart)  * t;
    writeMotorPosition(rightUpperMotor, rightNew);
    writeMotorPosition(leftUpperMotor, leftNew);
    currentCmdPos[rightIndex] = rightNew;
    currentCmdPos[leftIndex]  = leftNew;
    delay(moveDelay);
  }
}

void moveActivePairWithSupport(int activeRightMotor, int activeLeftMotor, int activeOffset,
                               int supportRightMotor, int supportLeftMotor, int supportFixedOffset) {
  activeOffset = limitOffset(activeOffset);
  supportFixedOffset = limitOffset(supportFixedOffset);

  int activeRightIndex = idToIndex(activeRightMotor);
  int activeLeftIndex  = idToIndex(activeLeftMotor);
  int supportRightIndex = idToIndex(supportRightMotor);
  int supportLeftIndex  = idToIndex(supportLeftMotor);

  int activeRightTarget = getRightUpperTarget(activeRightMotor, activeOffset);
  int activeLeftTarget  = getLeftUpperTarget(activeLeftMotor, activeOffset);
  int supportRightTarget = getRightUpperTarget(supportRightMotor, supportFixedOffset);
  int supportLeftTarget  = getLeftUpperTarget(supportLeftMotor, supportFixedOffset);

  int activeRightStart = currentCmdPos[activeRightIndex];
  int activeLeftStart  = currentCmdPos[activeLeftIndex];
  for (int step = 1; step <= moveSteps; step++) {
    float t = (float)step / (float)moveSteps;
    int activeRightNew = activeRightStart + (activeRightTarget - activeRightStart) * t;
    int activeLeftNew  = activeLeftStart  + (activeLeftTarget  - activeLeftStart)  * t;

    writeMotorPosition(activeRightMotor, activeRightNew);
    writeMotorPosition(activeLeftMotor, activeLeftNew);
    writeMotorPosition(supportRightMotor, supportRightTarget);
    writeMotorPosition(supportLeftMotor, supportLeftTarget);

    currentCmdPos[activeRightIndex] = activeRightNew;
    currentCmdPos[activeLeftIndex]  = activeLeftNew;
    currentCmdPos[supportRightIndex] = supportRightTarget;
    currentCmdPos[supportLeftIndex]  = supportLeftTarget;
    delay(moveDelay);
  }
}

void frontPushUpCycle() {
  Serial.println("Front push-up: back side lowered and fixed");
  int backLowerOffset = supportOffset;
  moveUpperPairFromHome(BR_UPPER, BL_UPPER, backLowerOffset);
  while (readChannel(CH5_PIN) > 1700) {
    moveActivePairWithSupport(FR_UPPER, FL_UPPER, pushOffset, BR_UPPER, BL_UPPER, backLowerOffset);
    moveActivePairWithSupport(FR_UPPER, FL_UPPER, -pushOffset, BR_UPPER, BL_UPPER, backLowerOffset);
  }
  moveToHome();
}

void backPushUpCycle() {
  Serial.println("Back push-up: front side lowered and fixed");
  int frontLowerOffset = -supportOffset;
  moveUpperPairFromHome(FR_UPPER, FL_UPPER, frontLowerOffset);

  while (readChannel(CH5_PIN) < 1300) {
    moveActivePairWithSupport(BR_UPPER, BL_UPPER, pushOffset, FR_UPPER, FL_UPPER, frontLowerOffset);
    moveActivePairWithSupport(BR_UPPER, BL_UPPER, -pushOffset, FR_UPPER, FL_UPPER, frontLowerOffset);
  }
  moveToHome();
}


// =====================================================
// ADDED CH6 DEMO MODES
// These functions are isolated from the walking gait and
// CH5 push-up functions above. They only run when CH6 is active.
// =====================================================
const int HIP_SWAY_OFFSET = 160;
const int SIT_FRONT_UPPER_OFFSET = -120;
const int SIT_FRONT_LOWER_OFFSET = 80;
const int SIT_BACK_UPPER_OFFSET = 300;
const int SIT_BACK_LOWER_OFFSET = -260;
const int HEART_UPPER_OFFSET = 180;
const int HEART_LOWER_OFFSET = -140;
const int WAVE_HIP_OFFSET = 180;
const int WAVE_UPPER_OFFSET = -220;
const int WAVE_LOWER_OFFSET = 180;

void copyHomeToPose(int pose[]) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    pose[i] = fixedHomePos[i];
  }
}

void setPoseMotor(int pose[], int motor, int target) {
  pose[idToIndex(motor)] = limitPosition(target);
}

void addPoseMotor(int pose[], int motor, int offset) {
  int index = idToIndex(motor);
  pose[index] = limitPosition(pose[index] + offset);
}

void moveAllToPose(const int targetPose[], int steps, int stepDelay) {
  int startPose[NUM_MOTORS];
  for (int i = 0; i < NUM_MOTORS; i++) {
    startPose[i] = currentCmdPos[i];
  }

  for (int step = 1; step <= steps; step++) {
    float t = (float)step / (float)steps;
    for (int i = 0; i < NUM_MOTORS; i++) {
      int newPos = startPose[i] + (targetPose[i] - startPose[i]) * t;
      writeMotorPosition(i + 1, newPos);
      currentCmdPos[i] = newPos;
    }
    delay(stepDelay);
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    currentCmdPos[i] = limitPosition(targetPose[i]);
  }
}

void buildIntermediateSitPose(int pose[]) {
  copyHomeToPose(pose);

  addPoseMotor(pose, FR_UPPER, rightSign * (SIT_FRONT_UPPER_OFFSET / 2));
  addPoseMotor(pose, FL_UPPER, leftSign  * (SIT_FRONT_UPPER_OFFSET / 2));
  addPoseMotor(pose, FR_LOWER, rightSign * (SIT_FRONT_LOWER_OFFSET / 2));
  addPoseMotor(pose, FL_LOWER, leftSign  * (SIT_FRONT_LOWER_OFFSET / 2));

  addPoseMotor(pose, BR_UPPER, rightSign * (SIT_BACK_UPPER_OFFSET / 2));
  addPoseMotor(pose, BL_UPPER, leftSign  * (SIT_BACK_UPPER_OFFSET / 2));
  addPoseMotor(pose, BR_LOWER, rightSign * (SIT_BACK_LOWER_OFFSET / 2));
  addPoseMotor(pose, BL_LOWER, leftSign  * (SIT_BACK_LOWER_OFFSET / 2));
}

void buildSittingPose(int pose[]) {
  copyHomeToPose(pose);

  addPoseMotor(pose, FR_UPPER, rightSign * SIT_FRONT_UPPER_OFFSET);
  addPoseMotor(pose, FL_UPPER, leftSign  * SIT_FRONT_UPPER_OFFSET);
  addPoseMotor(pose, FR_LOWER, rightSign * SIT_FRONT_LOWER_OFFSET);
  addPoseMotor(pose, FL_LOWER, leftSign  * SIT_FRONT_LOWER_OFFSET);

  addPoseMotor(pose, BR_UPPER, rightSign * SIT_BACK_UPPER_OFFSET);
  addPoseMotor(pose, BL_UPPER, leftSign  * SIT_BACK_UPPER_OFFSET);
  addPoseMotor(pose, BR_LOWER, rightSign * SIT_BACK_LOWER_OFFSET);
  addPoseMotor(pose, BL_LOWER, leftSign  * SIT_BACK_LOWER_OFFSET);
}

void hipSwayCycle() {
  Serial.println("CH6 HIGH: Hip sway mode");

  int poseA[NUM_MOTORS];
  int poseB[NUM_MOTORS];
  copyHomeToPose(poseA);
  copyHomeToPose(poseB);

  addPoseMotor(poseA, FR_HIP,  HIP_SWAY_OFFSET);
  addPoseMotor(poseA, BR_HIP,  HIP_SWAY_OFFSET);
  addPoseMotor(poseA, FL_HIP, -HIP_SWAY_OFFSET);
  addPoseMotor(poseA, BL_HIP, -HIP_SWAY_OFFSET);

  addPoseMotor(poseB, FR_HIP, -HIP_SWAY_OFFSET);
  addPoseMotor(poseB, BR_HIP, -HIP_SWAY_OFFSET);
  addPoseMotor(poseB, FL_HIP,  HIP_SWAY_OFFSET);
  addPoseMotor(poseB, BL_HIP,  HIP_SWAY_OFFSET);

  while (readChannel(CH6_PIN) > 1700) {
    moveAllToPose(poseA, 18, 18);
    moveToHome();
    moveAllToPose(poseB, 18, 18);
    moveToHome();
  }

  moveToHome();
}

void heartGestureOnce() {
  int sitPose[NUM_MOTORS];
  int heartPose[NUM_MOTORS];
  buildSittingPose(sitPose);
  buildSittingPose(heartPose);

  addPoseMotor(heartPose, FR_UPPER, rightSign * HEART_UPPER_OFFSET);
  addPoseMotor(heartPose, FL_UPPER, leftSign  * HEART_UPPER_OFFSET);
  addPoseMotor(heartPose, FR_LOWER, rightSign * HEART_LOWER_OFFSET);
  addPoseMotor(heartPose, FL_LOWER, leftSign  * HEART_LOWER_OFFSET);

  moveAllToPose(heartPose, 16, 18);
  moveAllToPose(sitPose, 16, 18);
}

void hiGestureOnce() {
  int sitPose[NUM_MOTORS];
  int waveA[NUM_MOTORS];
  int waveB[NUM_MOTORS];
  buildSittingPose(sitPose);
  buildSittingPose(waveA);
  buildSittingPose(waveB);

  // Uses the front-left leg according to the current prototype mapping:
  // FL_LOWER=1, FL_UPPER=2, FL_HIP=3.
  addPoseMotor(waveA, FL_HIP,   WAVE_HIP_OFFSET);
  addPoseMotor(waveA, FL_UPPER, WAVE_UPPER_OFFSET);
  addPoseMotor(waveA, FL_LOWER, WAVE_LOWER_OFFSET);

  addPoseMotor(waveB, FL_HIP,  -WAVE_HIP_OFFSET);
  addPoseMotor(waveB, FL_UPPER, WAVE_UPPER_OFFSET);
  addPoseMotor(waveB, FL_LOWER, WAVE_LOWER_OFFSET);

  moveAllToPose(waveA, 12, 18);
  moveAllToPose(waveB, 12, 18);
  moveAllToPose(sitPose, 12, 18);
}

void sittingCycle() {
  Serial.println("CH6 LOW: Sitting mode");

  int intermediateSitPose[NUM_MOTORS];
  int sittingPose[NUM_MOTORS];
  buildIntermediateSitPose(intermediateSitPose);
  buildSittingPose(sittingPose);

  moveAllToPose(intermediateSitPose, 30, 20);
  moveAllToPose(sittingPose, 30, 20);

  while (readChannel(CH6_PIN) < 1300) {
    int ch5 = readChannel(CH5_PIN);

    if (ch5 > 1700) {
      Serial.println("CH6 LOW + CH5 HIGH: Heart gesture");
      heartGestureOnce();
    }
    else if (ch5 < 1300) {
      Serial.println("CH6 LOW + CH5 LOW: Hi / wave gesture");
      hiGestureOnce();
    }
    else {
      // Hold sitting pose gently.
      for (int i = 0; i < NUM_MOTORS; i++) {
        writeMotorPosition(i + 1, sittingPose[i]);
        currentCmdPos[i] = sittingPose[i];
      }
      delay(80);
    }
  }

  moveAllToPose(intermediateSitPose, 30, 20);
  moveToHome();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(CH1_PIN, INPUT);
  pinMode(CH5_PIN, INPUT);
  pinMode(CH6_PIN, INPUT);

  ServoSerial.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX, SERVO_TX);
  sts.pSerial = &ServoSerial;

  Serial.println("======================================");
  Serial.println("Robot Dog Combined Demo + Dingo-Inspired IK Trot Gait");
  Serial.println("======================================");

  loadFixedHomePositions();
  readCurrentPositions();

  moveToHome();
}

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

  // Priority order:
  // CH6 HIGH = hip sway
  // CH6 LOW  = sit, with CH5 HIGH/LOW gestures inside sitting mode
  // CH1 HIGH = walk forward
  // CH1 LOW  = walk backward
  // CH5 HIGH = front push-up
  // CH5 LOW  = back push-up
  if (ch6 > 1700) {
    hipSwayCycle();
  }
  else if (ch6 < 1300) {
    sittingCycle();
  }
  else if (ch1 > 1700) {
    executeTrajectoryTrot(1); 
  }
  else if (ch1 < 1300) {
    executeTrajectoryTrot(-1); 
  }
  else if (ch5 > 1700) {
    frontPushUpCycle();
  }
  else if (ch5 < 1300) {
    backPushUpCycle();
  }
  else {
    delay(20);
  }
}
