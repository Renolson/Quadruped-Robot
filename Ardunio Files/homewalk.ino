#include <Arduino.h>
#include <SMS_STS.h>

// =====================================================
// ESP32 + FE-URT-1 + 12 x STS3215 Smooth Walking
// Fixed HOME position at power ON
// Hip motors hold HOME position
// =====================================================

// ---------------- UART SETTINGS ----------------
#define SERVO_RX 16
#define SERVO_TX 17
#define SERVO_BAUD 1000000

HardwareSerial ServoSerial(1);
SMS_STS sts;

// ---------------- MOTOR SETTINGS ----------------
const int NUM_MOTORS = 12;

int motorID[NUM_MOTORS] = {
  1, 2, 3,
  4, 5, 6,
  7, 8, 9,
  10, 11, 12
};

// =====================================================
// FIXED HOME POSITIONS
// =====================================================
// IMPORTANT:
// Replace these 2048 values with your real standing HOME values.
//
// To find real values:
// 1. Put robot manually in correct standing pose.
// 2. Upload old read-home code.
// 3. Open Serial Monitor.
// 4. Copy the HOME values printed for M1 to M12.
// 5. Paste them here.

int fixedHomePos[NUM_MOTORS] = {
  2048, 2048, 2048,
  2048, 2048, 2048,
  2048, 2048, 2048,
  2048, 2048, 2048
};

// Home/current standing positions
int homePos[NUM_MOTORS];
int currentCmdPos[NUM_MOTORS];

// Servo movement settings
int moveSpeed = 900;
int moveAcc   = 50;

// Startup home movement settings
int startupMoveSpeed = 500;
int startupMoveAcc   = 40;
int startupSteps     = 30;
int startupDelay     = 20;

// Smooth walking settings
int interpolationSteps = 15;
int interpolationDelay = 6;

// Direction correction
int direction[NUM_MOTORS] = {
   1, 1, 1,
   1, 1, 1,
   1, 1, 1,
   1, 1, 1
};

// Safety movement limit
// 60 = small step
// 200 = bigger step
// 300 = higher step
int maxOffsetLimit = 60;

// Hip motors
int hipMotors[] = {3, 4, 9, 10};
int hipCount = 4;

// =====================================================
// HELPER FUNCTIONS
// =====================================================

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

int idToIndex(int id) {
  return id - 1;
}

void writeMotorPosition(int id, int targetPos) {
  targetPos = limitPosition(targetPos);
  sts.WritePosEx(id, targetPos, moveSpeed, moveAcc);
}

void writeMotorPositionCustom(int id, int targetPos, int speed, int acc) {
  targetPos = limitPosition(targetPos);
  sts.WritePosEx(id, targetPos, speed, acc);
}

// =====================================================
// LOAD FIXED HOME POSITIONS
// =====================================================

void loadFixedHomePositions() {
  Serial.println();
  Serial.println("Loading FIXED HOME positions...");

  for (int i = 0; i < NUM_MOTORS; i++) {
    homePos[i] = fixedHomePos[i];

    Serial.print("Motor ID ");
    Serial.print(motorID[i]);
    Serial.print(" FIXED HOME = ");
    Serial.println(homePos[i]);
  }

  Serial.println("Fixed HOME positions loaded.");
  Serial.println();
}

// =====================================================
// READ CURRENT MOTOR POSITIONS
// =====================================================

void readCurrentPositions() {
  Serial.println("Reading current motor positions...");

  for (int i = 0; i < NUM_MOTORS; i++) {
    int id = motorID[i];
    int pos = sts.ReadPos(id);

    if (pos < 0 || pos > 4095) {
      Serial.print("WARNING: Could not read motor ID ");
      Serial.print(id);
      Serial.println(". Using fixed HOME as current position.");
      pos = homePos[i];
    }

    currentCmdPos[i] = pos;

    Serial.print("Motor ID ");
    Serial.print(id);
    Serial.print(" CURRENT = ");
    Serial.println(currentCmdPos[i]);

    delay(30);
  }

  Serial.println();
}

// =====================================================
// MOVE ALL MOTORS TO FIXED HOME AT POWER ON
// =====================================================

void moveAllMotorsToHomeAtStartup() {
  Serial.println("Moving all motors to FIXED HOME position...");

  for (int step = 1; step <= startupSteps; step++) {
    float t = (float)step / (float)startupSteps;

    for (int i = 0; i < NUM_MOTORS; i++) {
      int id = motorID[i];

      int startPos = currentCmdPos[i];
      int targetPos = homePos[i];

      int newPos = startPos + (targetPos - startPos) * t;
      newPos = limitPosition(newPos);

      writeMotorPositionCustom(id, newPos, startupMoveSpeed, startupMoveAcc);

      delay(2);
    }

    delay(startupDelay);
  }

  for (int i = 0; i < NUM_MOTORS; i++) {
    currentCmdPos[i] = homePos[i];
  }

  Serial.println("Robot is now at FIXED HOME position.");
  Serial.println();
}

// =====================================================
// HOLD HIP MOTORS AT HOME POSITION
// =====================================================

void holdHipMotorsAtHome() {
  for (int i = 0; i < hipCount; i++) {
    int id = hipMotors[i];
    int index = idToIndex(id);

    writeMotorPosition(id, homePos[index]);
    currentCmdPos[index] = homePos[index];

    delay(3);
  }
}

// =====================================================
// OPTIONAL: MEASURE / PRINT HIP POSITIONS
// =====================================================

void printHipPositions() {
  Serial.println("Hip motor current positions:");

  for (int i = 0; i < hipCount; i++) {
    int id = hipMotors[i];
    int index = idToIndex(id);

    int currentPos = sts.ReadPos(id);

    Serial.print("Hip Motor ID ");
    Serial.print(id);
    Serial.print(" | HOME = ");
    Serial.print(homePos[index]);
    Serial.print(" | Current = ");
    Serial.println(currentPos);

    delay(20);
  }
}

// =====================================================
// SMOOTH MOVE TWO MOTORS FROM FIXED HOME
// Hip motors are held during interpolation
// =====================================================

void smoothMoveTwoMotorsFromHome(int idA, int offsetA, int idB, int offsetB) {
  int indexA = idToIndex(idA);
  int indexB = idToIndex(idB);

  int startA = currentCmdPos[indexA];
  int startB = currentCmdPos[indexB];

  int targetA = homePos[indexA] + direction[indexA] * limitOffset(offsetA);
  int targetB = homePos[indexB] + direction[indexB] * limitOffset(offsetB);

  targetA = limitPosition(targetA);
  targetB = limitPosition(targetB);

  for (int step = 1; step <= interpolationSteps; step++) {
    float t = (float)step / (float)interpolationSteps;

    int newA = startA + (targetA - startA) * t;
    int newB = startB + (targetB - startB) * t;

    newA = limitPosition(newA);
    newB = limitPosition(newB);

    writeMotorPosition(idA, newA);
    writeMotorPosition(idB, newB);

    currentCmdPos[indexA] = newA;
    currentCmdPos[indexB] = newB;

    // Keep hips powered and locked at HOME
    holdHipMotorsAtHome();

    delay(interpolationDelay);
  }
}

// =====================================================
// WALKING STEP 1: M1,M2 and M7,M8
// =====================================================

void walkingStep1() {
  smoothMoveTwoMotorsFromHome(1, 280, 8, -280);
  delay(1);

  smoothMoveTwoMotorsFromHome(2, 200, 7, -200);
  delay(1);

  smoothMoveTwoMotorsFromHome(1, -480, 8, 480);
  delay(1);

  smoothMoveTwoMotorsFromHome(2, -200, 7, 200);
  delay(1);

  smoothMoveTwoMotorsFromHome(1, 680, 8, -680);
  delay(1);
}

// =====================================================
// WALKING STEP 2: M5,M6 and M11,M12
// =====================================================

void walkingStep2() {
  smoothMoveTwoMotorsFromHome(5, 280, 12, -280);
  delay(1);

  smoothMoveTwoMotorsFromHome(6, 200, 11, -200);
  delay(1);

  smoothMoveTwoMotorsFromHome(5, -480, 12, 480);
  delay(1);

  smoothMoveTwoMotorsFromHome(6, -200, 11, 200);
  delay(1);

  smoothMoveTwoMotorsFromHome(5, 680, 12, -680);
  delay(1);
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  ServoSerial.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX, SERVO_TX);
  sts.pSerial = &ServoSerial;

  Serial.println("ESP32 + 12 DOF STS3215 Robot Dog Walking Test");
  Serial.println("Fixed HOME position at power ON");

  // Load fixed HOME values from array
  loadFixedHomePositions();

  // Read actual current positions
  readCurrentPositions();

  // Move robot to fixed HOME position at power ON
  moveAllMotorsToHomeAtStartup();

  Serial.println("Holding hip motors at HOME...");
  holdHipMotorsAtHome();

  printHipPositions();

  Serial.println("Waiting 5 seconds before walking...");
  delay(5000);

  Serial.println("Starting walking sequence...");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  holdHipMotorsAtHome();

  walkingStep1();
  delay(200);

  holdHipMotorsAtHome();

  walkingStep2();
  delay(200);

  // Optional measurement
  // printHipPositions();
}