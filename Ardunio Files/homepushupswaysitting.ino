#include <Arduino.h>
#include <SMS_STS.h>

// =====================================================
// ESP32 + FE-URT-1 + 12 x STS3215
// CH5 Push-up + CH6 Hip Sway + Smooth Sitting Demo
// Intermediate 4-leg sitting pose added
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
  2048, 2110, 2045,
  2000, 2048, 2120,
  2048, 2048, 2100,
  2100, 2048, 2048
};

int homePos[NUM_MOTORS];
int currentCmdPos[NUM_MOTORS];

// =====================================================
// INTERMEDIATE 4-LEG SITTING POSE
// You gave this pose.
// Robot goes here first before final sitting.
// =====================================================

int intermediateSitPose[NUM_MOTORS] = {
  2549, 2596, 2076,
  2019, 2513, 2557,
  1526, 1442, 2048,
  2118, 1597, 1516
};

// =====================================================
// FINAL SITTING POSE
// Your previous final sitting pose.
// =====================================================

int sittingPose[NUM_MOTORS] = {
  2349, 2567, 2098,
  1985, 1944, 2738,
  1326, 1978, 2095,
  2060, 1309, 1679
};

// ---------------- SERVO SETTINGS ----------------
int moveSpeed = 550;
int moveAcc   = 45;

// General movement smoothness
int moveSteps = 30;
int moveDelay = 20;

// Push-up movement size
int pushOffset = 600;

// Support side lowering amount
int supportOffset = 220;

// Hip sway size
int swayOffset = 220;

// Safety limit
int maxOffsetLimit = 420;

// Right/left motor correction for upper-leg pair movements
int rightSign = 1;
int leftSign  = -1;

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

int readChannel(int pin) {
  int value = pulseIn(pin, HIGH, 25000);

  // If receiver signal missing, return neutral / HOME zone
  if (value < 900 || value > 2100) {
    return 1500;
  }

  return value;
}

// =====================================================
// HOME FUNCTIONS
// =====================================================

void loadFixedHomePositions() {
  Serial.println("Loading fixed HOME positions...");

  for (int i = 0; i < NUM_MOTORS; i++) {
    homePos[i] = fixedHomePos[i];

    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.print(" HOME = ");
    Serial.println(homePos[i]);
  }
}

void readCurrentPositions() {
  Serial.println("Reading current motor positions...");

  for (int i = 0; i < NUM_MOTORS; i++) {
    int id = motorID[i];
    int pos = sts.ReadPos(id);

    if (pos < 0 || pos > 4095) {
      Serial.print("WARNING: Could not read motor ");
      Serial.print(id);
      Serial.println(". Using HOME as current.");
      pos = homePos[i];
    }

    currentCmdPos[i] = pos;

    Serial.print("Motor ");
    Serial.print(id);
    Serial.print(" CURRENT = ");
    Serial.println(pos);

    delay(30);
  }
}

void moveToHome() {
  Serial.println("Moving to HOME...");

  for (int step = 1; step <= moveSteps; step++) {
    float t = (float)step / (float)moveSteps;

    for (int i = 0; i < NUM_MOTORS; i++) {
      int startPos = currentCmdPos[i];
      int targetPos = homePos[i];

      int newPos = startPos + (targetPos - startPos) * t;
      newPos = limitPosition(newPos);

      writeMotorPosition(motorID[i], newPos);
      delay(2);
    }

    delay(moveDelay);
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    currentCmdPos[i] = homePos[i];
  }

  Serial.println("HOME reached.");
}

// =====================================================
// CUSTOM POSE SMOOTH MOVEMENT
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

      delay(2);
    }

    delay(stepDelay);
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    currentCmdPos[i] = targetPose[i];
  }
}

// =====================================================
// UPPER PAIR TARGET FUNCTIONS
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

    int rightNew = rightStart + (rightTarget - rightStart) * t;
    int leftNew  = leftStart  + (leftTarget  - leftStart)  * t;

    rightNew = limitPosition(rightNew);
    leftNew  = limitPosition(leftNew);

    writeMotorPosition(rightUpperMotor, rightNew);
    writeMotorPosition(leftUpperMotor, leftNew);

    currentCmdPos[rightIndex] = rightNew;
    currentCmdPos[leftIndex]  = leftNew;

    delay(moveDelay);
  }
}

// =====================================================
// ACTIVE PUSH-UP PAIR + SUPPORT PAIR HOLD
// =====================================================

void moveActivePairWithSupport(
  int activeRight,
  int activeLeft,
  int activeOffset,
  int supportRight,
  int supportLeft,
  int supportFixedOffset
) {
  int aR = idToIndex(activeRight);
  int aL = idToIndex(activeLeft);
  int sR = idToIndex(supportRight);
  int sL = idToIndex(supportLeft);

  int aRTarget = getRightUpperTarget(activeRight, activeOffset);
  int aLTarget = getLeftUpperTarget(activeLeft, activeOffset);

  int sRTarget = getRightUpperTarget(supportRight, supportFixedOffset);
  int sLTarget = getLeftUpperTarget(supportLeft, supportFixedOffset);

  int aRStart = currentCmdPos[aR];
  int aLStart = currentCmdPos[aL];

  for (int step = 1; step <= moveSteps; step++) {
    float t = (float)step / (float)moveSteps;

    int aRNew = aRStart + (aRTarget - aRStart) * t;
    int aLNew = aLStart + (aLTarget - aLStart) * t;

    aRNew = limitPosition(aRNew);
    aLNew = limitPosition(aLNew);

    // Active pair moves
    writeMotorPosition(activeRight, aRNew);
    writeMotorPosition(activeLeft, aLNew);

    // Support pair stays fixed
    writeMotorPosition(supportRight, sRTarget);
    writeMotorPosition(supportLeft, sLTarget);

    currentCmdPos[aR] = aRNew;
    currentCmdPos[aL] = aLNew;
    currentCmdPos[sR] = sRTarget;
    currentCmdPos[sL] = sLTarget;

    delay(moveDelay);
  }
}

// =====================================================
// CH5 HIGH - FRONT PUSH-UP
// Back upper legs lower first and stay fixed
// =====================================================

void frontPushUpCycle() {
  Serial.println("CH5 HIGH: Front push-up");

  int backLowerOffset = supportOffset;

  moveUpperPairFromHome(BR_UPPER, BL_UPPER, backLowerOffset);

  while (readChannel(CH5_PIN) > 1700) {
    moveActivePairWithSupport(
      FR_UPPER, FL_UPPER,
      pushOffset,
      BR_UPPER, BL_UPPER,
      backLowerOffset
    );

    moveActivePairWithSupport(
      FR_UPPER, FL_UPPER,
      -pushOffset,
      BR_UPPER, BL_UPPER,
      backLowerOffset
    );
  }

  moveToHome();
}

// =====================================================
// CH5 LOW - BACK PUSH-UP
// Front upper legs lower first and stay fixed
// =====================================================

void backPushUpCycle() {
  Serial.println("CH5 LOW: Back push-up");

  int frontLowerOffset = -supportOffset;

  moveUpperPairFromHome(FR_UPPER, FL_UPPER, frontLowerOffset);

  while (readChannel(CH5_PIN) < 1300) {
    moveActivePairWithSupport(
      BR_UPPER, BL_UPPER,
      pushOffset,
      FR_UPPER, FL_UPPER,
      frontLowerOffset
    );

    moveActivePairWithSupport(
      BR_UPPER, BL_UPPER,
      -pushOffset,
      FR_UPPER, FL_UPPER,
      frontLowerOffset
    );
  }

  moveToHome();
}

// =====================================================
// FOUR HIP MOVE HELPER
// M3 and M10 same direction
// M4 and M9 opposite direction
// =====================================================

void moveFourHipsFromHome(
  int motorA, int offsetA,
  int motorB, int offsetB,
  int motorC, int offsetC,
  int motorD, int offsetD
) {
  offsetA = limitOffset(offsetA);
  offsetB = limitOffset(offsetB);
  offsetC = limitOffset(offsetC);
  offsetD = limitOffset(offsetD);

  int indexA = idToIndex(motorA);
  int indexB = idToIndex(motorB);
  int indexC = idToIndex(motorC);
  int indexD = idToIndex(motorD);

  int startA = currentCmdPos[indexA];
  int startB = currentCmdPos[indexB];
  int startC = currentCmdPos[indexC];
  int startD = currentCmdPos[indexD];

  int targetA = limitPosition(homePos[indexA] + offsetA);
  int targetB = limitPosition(homePos[indexB] + offsetB);
  int targetC = limitPosition(homePos[indexC] + offsetC);
  int targetD = limitPosition(homePos[indexD] + offsetD);

  for (int step = 1; step <= moveSteps; step++) {
    float t = (float)step / (float)moveSteps;

    int newA = startA + (targetA - startA) * t;
    int newB = startB + (targetB - startB) * t;
    int newC = startC + (targetC - startC) * t;
    int newD = startD + (targetD - startD) * t;

    newA = limitPosition(newA);
    newB = limitPosition(newB);
    newC = limitPosition(newC);
    newD = limitPosition(newD);

    writeMotorPosition(motorA, newA);
    writeMotorPosition(motorB, newB);
    writeMotorPosition(motorC, newC);
    writeMotorPosition(motorD, newD);

    currentCmdPos[indexA] = newA;
    currentCmdPos[indexB] = newB;
    currentCmdPos[indexC] = newC;
    currentCmdPos[indexD] = newD;

    delay(moveDelay);
  }
}

// =====================================================
// CH6 HIGH - HIP SWAY
// Sway right -> HOME -> sway left -> HOME
// Loops while CH6 is HIGH
// =====================================================

void hipSwayCycle() {
  Serial.println("CH6 HIGH: Hip sway right-home-left-home");

  while (readChannel(CH6_PIN) > 1700) {
    // Sway right
    // M3 and M10 same, M4 and M9 opposite
    moveFourHipsFromHome(
      FR_HIP,  swayOffset,
      BL_HIP, -swayOffset,
      FL_HIP,  swayOffset,
      BR_HIP, -swayOffset
    );

    delay(300);

    moveToHome();
    delay(200);

    if (readChannel(CH6_PIN) <= 1700) break;

    // Sway left
    moveFourHipsFromHome(
      FR_HIP, -swayOffset,
      BL_HIP,  swayOffset,
      FL_HIP, -swayOffset,
      BR_HIP,  swayOffset
    );

    delay(300);

    moveToHome();
    delay(200);
  }

  moveToHome();
}

// =====================================================
// CH6 LOW - SMOOTH SITTING
// Intermediate 4-leg pose -> final sitting pose
// Holds sitting while CH6 remains LOW
// =====================================================

void hipSitSmooth() {
  Serial.println("CH6 LOW: Smooth sitting with intermediate pose");

  // Step 1: intermediate 4-leg sitting pose
  moveToCustomPoseSmooth(intermediateSitPose, 50, 22);
  delay(250);

  // Step 2: final sitting pose
  moveToCustomPoseSmooth(sittingPose, 55, 24);

  // Hold final sitting while CH6 stays LOW
  while (readChannel(CH6_PIN) < 1300) {
    for (int i = 0; i < NUM_MOTORS; i++) {
      writeMotorPosition(motorID[i], sittingPose[i]);
      currentCmdPos[i] = sittingPose[i];
      delay(2);
    }

    delay(100);
  }

  moveToHome();
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(CH5_PIN, INPUT);
  pinMode(CH6_PIN, INPUT);

  ServoSerial.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX, SERVO_TX);
  sts.pSerial = &ServoSerial;

  Serial.println("======================================");
  Serial.println("Robot Dog Demo Ready");
  Serial.println("CH5 Push-up | CH6 Sway/Sit");
  Serial.println("Intermediate sit pose added");
  Serial.println("======================================");

  loadFixedHomePositions();
  readCurrentPositions();

  // Initial HOME pose at power ON
  moveToHome();

  Serial.println("HOME set. Ready for RC commands.");
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  int ch5 = readChannel(CH5_PIN);
  int ch6 = readChannel(CH6_PIN);

  Serial.print("CH5 = ");
  Serial.print(ch5);
  Serial.print(" | CH6 = ");
  Serial.println(ch6);

  // CH6 has priority
  if (ch6 > 1700) {
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
    moveToHome();
    delay(100);
  }
}