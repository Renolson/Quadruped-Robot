#define CH1_PIN 32
#define CH2_PIN 33
#define CH3_PIN 25
#define CH4_PIN 26
#define CH5_PIN 27
#define CH6_PIN 14

int readChannel(int pin) {
  int value = pulseIn(pin, HIGH, 25000);

  if (value < 900 || value > 2100) {
    return 1500;
  }

  return value;
}

void setup() {
  Serial.begin(115200);

  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
  pinMode(CH3_PIN, INPUT);
  pinMode(CH4_PIN, INPUT);
  pinMode(CH5_PIN, INPUT);
  pinMode(CH6_PIN, INPUT);

  Serial.println("MC-7RE V2 receiver PWM test");
}

void loop() {
  Serial.print("CH1: ");
  Serial.print(readChannel(CH1_PIN));

  Serial.print(" | CH2: ");
  Serial.print(readChannel(CH2_PIN));

  Serial.print(" | CH3: ");
  Serial.print(readChannel(CH3_PIN));

  Serial.print(" | CH4: ");
  Serial.print(readChannel(CH4_PIN));

  Serial.print(" | CH5: ");
  Serial.print(readChannel(CH5_PIN));

  Serial.print(" | CH6: ");
  Serial.println(readChannel(CH6_PIN));

  delay(200);
}