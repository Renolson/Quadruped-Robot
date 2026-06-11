#include <Arduino.h>
#include <SMS_STS.h>

// =====================================================
// ESP32 + FE-URT-1 + STS3215 Pose Finder Code
// Torque OFF -> move motors by hand
// Serial Monitor prints M1-M12 positions
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

int currentPos[NUM_MOTORS];

// =====================================================
// TORQUE CONTROL
// =====================================================

void torqueOffAllMotors() {
  Serial.println("Turning torque OFF for all motors...");
  Serial.println("Now you can move the legs by hand.");

  for (int i = 0; i < NUM_MOTORS; i++) {
    int id = motorID[i];

    // Torque OFF
    sts.EnableTorque(id, 0);

    Serial.print("Motor ");
    Serial.print(id);
    Serial.println(" torque OFF");

    delay(30);
  }

  Serial.println();
}

void torqueOnAllMotors() {
  Serial.println("Turning torque ON for all motors...");

  for (int i = 0; i < NUM_MOTORS; i++) {
    int id = motorID[i];

    // Torque ON
    sts.EnableTorque(id, 1);

    Serial.print("Motor ");
    Serial.print(id);
    Serial.println(" torque ON");

    delay(30);
  }

  Serial.println();
}

// =====================================================
// READ AND PRINT POSITIONS
// =====================================================

void readAllPositions() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    int id = motorID[i];

    int pos = sts.ReadPos(id);

    if (pos < 0 || pos > 4095) {
      currentPos[i] = -1;
    } else {
      currentPos[i] = pos;
    }

    delay(20);
  }
}

void printPositionsNormal() {
  Serial.println("Current motor positions:");

  for (int i = 0; i < NUM_MOTORS; i++) {
    Serial.print("M");
    Serial.print(motorID[i]);
    Serial.print(" = ");
    Serial.println(currentPos[i]);
  }

  Serial.println();
}

void printHomeArray() {
  Serial.println("Copy this fixedHomePos array:");

  Serial.println("int fixedHomePos[NUM_MOTORS] = {");

  Serial.print("  ");
  Serial.print(currentPos[0]); Serial.print(", ");
  Serial.print(currentPos[1]); Serial.print(", ");
  Serial.print(currentPos[2]); Serial.println(",");

  Serial.print("  ");
  Serial.print(currentPos[3]); Serial.print(", ");
  Serial.print(currentPos[4]); Serial.print(", ");
  Serial.print(currentPos[5]); Serial.println(",");

  Serial.print("  ");
  Serial.print(currentPos[6]); Serial.print(", ");
  Serial.print(currentPos[7]); Serial.print(", ");
  Serial.print(currentPos[8]); Serial.println(",");

  Serial.print("  ");
  Serial.print(currentPos[9]); Serial.print(", ");
  Serial.print(currentPos[10]); Serial.print(", ");
  Serial.print(currentPos[11]); Serial.println();

  Serial.println("};");
  Serial.println();
}

// =====================================================
// SERIAL COMMANDS
// =====================================================

void checkSerialCommand() {
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'r') {
      readAllPositions();
      printPositionsNormal();
      printHomeArray();
    }

    else if (cmd == 'f') {
      torqueOffAllMotors();
    }

    else if (cmd == 't') {
      torqueOnAllMotors();
    }

    else if (cmd == 'h') {
      readAllPositions();
      printHomeArray();
    }
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  ServoSerial.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX, SERVO_TX);
  sts.pSerial = &ServoSerial;

  Serial.println("======================================");
  Serial.println("STS3215 Robot Dog Pose Finder");
  Serial.println("======================================");
  Serial.println();

  torqueOffAllMotors();

  Serial.println("Commands:");
  Serial.println("r = read and print all motor positions");
  Serial.println("f = torque OFF, free motors");
  Serial.println("t = torque ON, lock motors");
  Serial.println("h = print fixedHomePos array only");
  Serial.println();

  Serial.println("Move the robot by hand, then type r in Serial Monitor.");
  Serial.println();
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  checkSerialCommand();

  static unsigned long lastPrintTime = 0;

  if (millis() - lastPrintTime > 2000) {
    lastPrintTime = millis();

    readAllPositions();
    printPositionsNormal();
    printHomeArray();
  }
}