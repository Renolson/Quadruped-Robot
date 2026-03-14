#include <SCServo.h>

SMS_STS st;

// Pins
#define S_RXD 18
#define S_TXD 19
#define RC_CH1 34 // Pin connected to Receiver Signal

// Servo IDs
byte ID[] = {1, 2, 3};
s16 Position[3];
u16 Speed[] = {2000, 2000, 2000};
byte ACC[] = {50, 50, 50};

void setup() {
  Serial.begin(1000000);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  
  pinMode(RC_CH1, INPUT);
  delay(1000);
}

void loop() {
  // Read the PWM signal from the receiver
  int ch1_value = pulseIn(RC_CH1, HIGH);
  
  if (ch1_value > 1600) {
    Serial.println("Walking FORWARD");
    walkForward();
  } 
  else if (ch1_value < 1400 && ch1_value > 800) {
    Serial.println("Walking REVERSE");
    walkReverse();
  } 
  else {
    // Neutral / Stop position (All servos to center 2048)
    Position[0] = 2048; Position[1] = 2048; Position[2] = 2048;
    st.SyncWritePosEx(ID, 3, Position, Speed, ACC);
  }
  delay(10); 
}

void walkForward() {
  // Simple 2-Step Gait Sequence for 3 motors
  // Step 1
  Position[0] = 2500; Position[1] = 1500; Position[2] = 2500;
  st.SyncWritePosEx(ID, 3, Position, Speed, ACC);
  delay(300);
  
  // Step 2
  Position[0] = 1500; Position[1] = 2500; Position[2] = 1500;
  st.SyncWritePosEx(ID, 3, Position, Speed, ACC);
  delay(300);
}

void walkReverse() {
  // Opposite of forward
  Position[0] = 1500; Position[1] = 2500; Position[2] = 1500;
  st.SyncWritePosEx(ID, 3, Position, Speed, ACC);
  delay(300);

  Position[0] = 2500; Position[1] = 1500; Position[2] = 2500;
  st.SyncWritePosEx(ID, 3, Position, Speed, ACC);
  delay(300);
}
