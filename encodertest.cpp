#include <Arduino.h>
//PIN DEFINITIONS

// TB6612FNG motor driver 
constexpr uint8_t PWMA = 35;
constexpr uint8_t AIN2 = 36;
constexpr uint8_t AIN1 = 37;
constexpr uint8_t BIN1 = 39;
constexpr uint8_t BIN2 = 40;
constexpr uint8_t PWMB = 41;

// Encoders 
constexpr uint8_t LEFT_ENC_A  = 47;
constexpr uint8_t LEFT_ENC_B  = 48;
constexpr uint8_t RIGHT_ENC_A = 42;
constexpr uint8_t RIGHT_ENC_B = 45;

constexpr uint8_t  LEDC_CH_A = 0;
constexpr uint8_t  LEDC_CH_B = 1;
constexpr uint32_t LEDC_FREQ = 1000;
constexpr uint8_t  LEDC_RES  = 8;


constexpr int TICKS_PER_90  = 115; // tune to taste, depends on wheel diameter and gear ratio
constexpr int TICKS_PER_180 = 200;  // tune to taste, depends on wheel diameter and gear ratio
constexpr int TURN_SPEED    = 200;   // PWM 0-255, tune to taste
constexpr int NUDGE_SPEED   = 200;   // PWM 0-255, tune to taste
constexpr int NUDGE_TICKS   = 50;    // ticks to move forward when nudging
constexpr uint32_t TURN_GAP_MS = 500;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 50;

// Flip either value to -1 if a wheel's tick sign is reversed in your wiring.
constexpr int LEFT_ENCODER_POLARITY = 1;
constexpr int RIGHT_ENCODER_POLARITY = 1;

// ENCODER STATE

volatile int leftTicks  = 0;
volatile int rightTicks = 0;

void IRAM_ATTR leftEncISR() {
  if (digitalRead(LEFT_ENC_B) == HIGH) {
    leftTicks += LEFT_ENCODER_POLARITY;
  } else {
    leftTicks -= LEFT_ENCODER_POLARITY;
  }
}

void IRAM_ATTR rightEncISR() {
  if (digitalRead(RIGHT_ENC_B) == HIGH) {
    rightTicks += RIGHT_ENCODER_POLARITY;
  } else {
    rightTicks -= RIGHT_ENCODER_POLARITY;
  }
}

void resetEncoders() {
  leftTicks  = 0;
  rightTicks = 0;
}

void printEncoderTicks(const char *label) {
  Serial.printf("%s | leftTicks=%d | rightTicks=%d\n", label, leftTicks, rightTicks);
}

void setMotors(int speedA, int speedB) {
  // --- Left motor (A channel) ---
  if (speedA >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    speedA = -speedA;
  }
  ledcWrite(LEDC_CH_A, static_cast<uint8_t>(constrain(speedA, 0, 255)));

  //  Right motor (B channel) 
  if (speedB >= 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    speedB = -speedB;
  }
  ledcWrite(LEDC_CH_B, static_cast<uint8_t>(constrain(speedB, 0, 255)));
}

void stopMotors() {
  setMotors(0, 0);
}
// TURN FUNCTIONS (in-place, one wheel forward / one wheel back)

void turnRight90(int speed) {
  resetEncoders();
  setMotors(speed, -speed);
  while (abs(leftTicks) < TICKS_PER_90) {
    delayMicroseconds(100);
  }
  stopMotors();
  // printEncoderTicks("Right 90 done.");
}

void turnLeft90(int speed) {
  resetEncoders();
  setMotors(-speed, speed);
  while (abs(rightTicks) < TICKS_PER_90) {
    delayMicroseconds(100);
  }
  stopMotors();
  // printEncoderTicks("Left 90 done.");
}

void turnAround180(int speed) {
  resetEncoders();
  setMotors(speed, -speed);
  while (abs(leftTicks) < TICKS_PER_180) {
    delayMicroseconds(100);
  }
  stopMotors();
  // printEncoderTicks("180 done.");
}

void turnRightFourTimes(int speed) {
  for (int turnIndex = 0; turnIndex < 4; ++turnIndex) {
    turnRight90(speed);
    if (turnIndex < 3) {
      delay(TURN_GAP_MS);
    }
  }
}

void turnLeftFourTimes(int speed) {
  for (int turnIndex = 0; turnIndex < 4; ++turnIndex) {
    turnLeft90(speed);
    if (turnIndex < 3) {
      delay(TURN_GAP_MS);
    }
  }
}


void nudgeforward(int speed) {
  resetEncoders();
  setMotors(speed, speed);
  while (abs(leftTicks) < NUDGE_TICKS && abs(rightTicks) < NUDGE_TICKS) {
    delayMicroseconds(100);
  }
  stopMotors();
}

// SETUP

void setup() {
  Serial.begin(115200);
  delay(500);

  // GPIO — TB6612FNG
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Encoders
  pinMode(LEFT_ENC_A,  INPUT);
  pinMode(LEFT_ENC_B,  INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  leftEncISR,  RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncISR, RISING);

  // ledc PWM
  ledcSetup(LEDC_CH_A, LEDC_FREQ, LEDC_RES);
  ledcSetup(LEDC_CH_B, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(PWMA, LEDC_CH_A);
  ledcAttachPin(PWMB, LEDC_CH_B);

  stopMotors();
  delay(3000);
}

// LOOP 

void loop() {
  static int lastRightButtonState = HIGH;
  static int lastLeftButtonState = HIGH;
  static uint32_t lastRightButtonChangeMs = 0;
  static uint32_t lastLeftButtonChangeMs = 0;

  const uint32_t now = millis();
  delay(2000);
    turnLeftFourTimes(TURN_SPEED); // use whatever function you want to test here
}