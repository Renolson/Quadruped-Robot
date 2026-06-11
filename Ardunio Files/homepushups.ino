#include <Arduino.h>
#include <SMS_STS.h>

// =====================================================
// ESP32 + FE-URT-1 + 12 x STS3215
// CH5 Push-up Demo
// Initial HOME pose at power ON
// Front push-up: back side lowers and stays lowered
// Back push-up : front side lowers and stays lowered
// =====================================================

// ---------------- UART SETTINGS ----------------
#define SERVO_RX 16
#define SERVO_TX 17
#define SERVO_BAUD 1000000

HardwareSerial ServoSerial(1);
SMS_STS sts;

// ---------------- RC RECEIVER ----------------
#define CH5_PIN 27

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
// Your current HOME pose values

int fixedHomePos[NUM_MOTORS] = {
  2000, 2110, 2045,
  2000, 1885, 2120,
  2048, 2075, 2100,
  2100, 2048, 2048
};

int homePos[NUM_MOTORS];
int currentCmdPos[NUM_MOTORS];

// ---------------- SERVO SETTINGS ----------------
int moveSpeed = 550;
int moveAcc   = 45;

// Movement smoothness
int moveSteps = 30;
int moveDelay = 20;

// Main push-up movement size
int pushOffset = 600;

// Opposite side lowering amount
// If support side is still going high, change sign of support offset below.
int supportOffset = 220;

// Safety limit
int maxOffsetLimit = 420;

// =====================================================
// RIGHT / LEFT DIRECTION
// =====================================================
// Your right and left motors are mounted opposite.

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

  // If receiver signal missing, return HOME zone
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

      writeMotorPosition(i + 1, newPos);
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
// TARGET CALCULATION FOR RIGHT / LEFT UPPER PAIR
// =====================================================

int getRightUpperTarget(int rightMotor, int offset) {
  int index = idToIndex(rightMotor);
  offset = limitOffset(offset);
  return limitPosition(homePos[index] + rightSign * offset);
}

int getLeftUpperTarget(int leftMotor, int offset) {
  int index = idToIndex(leftMotor);
  offset = limitOffset(offset);
  return limitPosition(homePos[index] + leftSign * offset);
}

// =====================================================
// MOVE UPPER PAIR FROM HOME
// =====================================================

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
// active pair moves up/down
// support pair stays fixed in lowered pose

void moveActivePairWithSupport(
  int activeRightMotor,
  int activeLeftMotor,
  int activeOffset,
  int supportRightMotor,
  int supportLeftMotor,
  int supportFixedOffset
) {
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

    activeRightNew = limitPosition(activeRightNew);
    activeLeftNew  = limitPosition(activeLeftNew);

    // Move active push-up pair
    writeMotorPosition(activeRightMotor, activeRightNew);
    writeMotorPosition(activeLeftMotor, activeLeftNew);

    // Keep support pair fixed in lowered pose
    writeMotorPosition(supportRightMotor, supportRightTarget);
    writeMotorPosition(supportLeftMotor, supportLeftTarget);

    currentCmdPos[activeRightIndex] = activeRightNew;
    currentCmdPos[activeLeftIndex]  = activeLeftNew;

    currentCmdPos[supportRightIndex] = supportRightTarget;
    currentCmdPos[supportLeftIndex]  = supportLeftTarget;

    delay(moveDelay);
  }
}

// =====================================================
// FRONT PUSH-UP
// Back side lowers first and stays lowered
// Active motors: front upper M2 and M11
// Support motors: back upper M7 and M6
// =====================================================

void frontPushUpCycle() {
  Serial.println("Front push-up: back side lowered and fixed");

  // IMPORTANT:
  // Previously back stayed high. So here support offset is POSITIVE.
  // If your back still goes high, change this to: int backLowerOffset = -supportOffset;
  int backLowerOffset = supportOffset;

  // Step 1: lower back upper legs once
  moveUpperPairFromHome(BR_UPPER, BL_UPPER, backLowerOffset);

  // Step 2: front upper legs do push-up while back stays lowered
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
// BACK PUSH-UP
// Front side lowers first and stays lowered
// Active motors: back upper M7 and M6
// Support motors: front upper M2 and M11
// =====================================================

void backPushUpCycle() {
  Serial.println("Back push-up: front side lowered and fixed");

  // You said back push-up is okay, so keeping this sign.
  int frontLowerOffset = -supportOffset;

  // Step 1: lower front upper legs once
  moveUpperPairFromHome(FR_UPPER, FL_UPPER, frontLowerOffset);

  // Step 2: back upper legs do push-up while front stays lowered
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
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(CH5_PIN, INPUT);

  ServoSerial.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX, SERVO_TX);
  sts.pSerial = &ServoSerial;

  Serial.println("======================================");
  Serial.println("Robot Dog CH5 Push-up Demo");
  Serial.println("Initial HOME pose + support side hold");
  Serial.println("======================================");

  loadFixedHomePositions();
  readCurrentPositions();

  // At power ON, robot goes to HOME pose
  moveToHome();

  Serial.println("Ready.");
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  int ch5 = readChannel(CH5_PIN);

  Serial.print("CH5 = ");
  Serial.println(ch5);

  if (ch5 > 1700) {
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