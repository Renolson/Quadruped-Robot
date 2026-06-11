#include <Arduino.h>
#include <SMS_STS.h>

// =====================================================
// ESP32 + FE-URT-1 + 12 x STS3215
// Complete Robot Dog Demo Code
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

// M10, M11, M12 front side
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
// SERVO SETTINGS
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

      delay(2);
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
    delay(2);
  }
}

// =====================================================
// MOVE SELECTED MOTORS TO HOME
// Used after sitting: back legs first, front legs after
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

      delay(2);
    }

    delay(stepDelay);
  }

  for (int i = 0; i < count; i++) {
    int id = selectedMotors[i];
    int index = idToIndex(id);
    currentCmdPos[index] = homePos[index];
  }
}

// =====================================================
// MOVE BACK LEG MOTORS M5, M6, M7, M8 TO A RECOVERY POSE
// Used when returning from sitting to HOME.
// Only motors 5, 6, 7, 8 are moved; other motors hold position.
// =====================================================

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
      delay(2);
    }

    delay(stepDelay);
  }

  for (int i = 0; i < 4; i++) {
    currentCmdPos[idToIndex(ids[i])] = targets[i];
  }
}

void back5678RecoverySequence() {
  Serial.println("Back M5-M8 recovery intermediate sequence");

// Pose 1: M5=2143, M6=2931, M7=1122, M8=1778
moveBack5678ToRecoveryPose(2140, 2931, 1122, 1780, 28, 16);

// Pose 2: M5=2240, M6=2733, M7=1300, M8=1632
moveBack5678ToRecoveryPose(2290, 2733, 1322, 1630, 28, 16);

// Pose 3: M5=2311, M6=2620, M7=1443, M8=1560
moveBack5678ToRecoveryPose(2370, 2520, 1535, 1550, 28, 16);

// Pose 4: M5=2240, M6=2578, M7=1561, M8=1646
moveBack5678ToRecoveryPose(2290, 2420, 1635, 1630, 28, 16);

// Pose 5: M5=2186, M6=2576, M7=1574, M8=1810
moveBack5678ToRecoveryPose(2160, 2320, 1735, 1684, 28, 16);
}

// =====================================================
// UPPER LEG PAIR FUNCTIONS
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
// ACTIVE PUSH-UP PAIR WITH SUPPORT PAIR HOLD
// =====================================================

void moveActivePairWithSupport(
  int activeRight,
  int activeLeft,
  int activeOffset,
  int supportRight,
  int supportLeft,
  int supportFixedOffset
) {
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

    int activeRightNew = activeRightStart + (activeRightTarget - activeRightStart) * t;
    int activeLeftNew  = activeLeftStart  + (activeLeftTarget  - activeLeftStart)  * t;

    activeRightNew = limitPosition(activeRightNew);
    activeLeftNew  = limitPosition(activeLeftNew);

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

// =====================================================
// CH5 HIGH - FRONT PUSH-UP
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
// HIP SWAY FUNCTION
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
// =====================================================

void hipSwayCycle() {
  Serial.println("CH6 HIGH: Hip sway");

  while (readChannel(CH6_PIN) > 1700) {
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
// SITTING HEART GESTURE
// CH6 LOW + CH5 HIGH
// =====================================================

void sittingHeartGesture() {
  Serial.println("Sitting: Heart gesture");

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

// =====================================================
// SITTING HI GESTURE
// CH6 LOW + CH5 LOW
// Uses M10, M11, M12
// =====================================================

void sittingHiGesture() {
  Serial.println("Sitting: Hi gesture with M10 M11 M12");

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

// =====================================================
// CH6 LOW - SITTING MODE
// =====================================================

void hipSitSmooth() {
  Serial.println("CH6 LOW: Sitting mode");

  moveToCustomPoseSmooth(intermediateSitPose, 50, 22);
  delay(250);

  moveToCustomPoseSmooth(sittingPose, 55, 24);

  while (readChannel(CH6_PIN) < 1300) {
    int ch5 = readChannel(CH5_PIN);

    if (ch5 > 1700) {
      sittingHeartGesture();
    }
    else if (ch5 < 1300) {
      sittingHiGesture();
    }
    else {
      holdPose(sittingPose);
      delay(100);
    }
  }

  // Return from sitting using intermediate return pose
  moveToCustomPoseSmooth(intermediateReturnPose, 45, 20);
  delay(200);

  // Back legs recover through your M5-M8 intermediate poses first
  back5678RecoverySequence();
  delay(150);

  // Then finish HOME in two stages:
  // Stage 1: lower links first -> M5 and M8
  int backLowerPair[] = {
    BL_LOWER, BR_LOWER
  };

  moveSelectedMotorsToHome(backLowerPair, 2, 45, 20);
  delay(150);

  // Stage 2: upper links next -> M6 and M7
  int backUpperPair[] = {
    BL_UPPER, BR_UPPER
  };

  moveSelectedMotorsToHome(backUpperPair, 2, 45, 20);
  delay(150);

  // Optional: hip motors M4 and M9 return after upper/lower links are stable
  int backHipPair[] = {
    BL_HIP, BR_HIP
  };

  moveSelectedMotorsToHome(backHipPair, 2, 35, 18);
  delay(150);

  // Front legs go HOME after
  int frontLegs[] = {
    FR_HIP, FR_UPPER, FR_LOWER,
    FL_HIP, FL_UPPER, FL_LOWER
  };

  moveSelectedMotorsToHome(frontLegs, 6, 45, 20);
  delay(200);

  Serial.println("Returned HOME from sitting.");
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
  int ch5 = readChannel(CH5_PIN);
  int ch6 = readChannel(CH6_PIN);

  Serial.print("CH5 = ");
  Serial.print(ch5);
  Serial.print(" | CH6 = ");
  Serial.println(ch6);

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