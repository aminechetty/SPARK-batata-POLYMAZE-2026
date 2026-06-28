#include <Arduino.h>
#include <QTRSensors.h>

//  TB6612FNG motor driver 
constexpr uint8_t PWMA = 35;
constexpr uint8_t AIN2 = 36;
constexpr uint8_t AIN1 = 37;
constexpr uint8_t BIN1 = 39;
constexpr uint8_t BIN2 = 40;
constexpr uint8_t PWMB = 41;

// motor driver PWM channel configuration
constexpr uint8_t  LEDC_CH_A = 0;
constexpr uint8_t  LEDC_CH_B = 1;
constexpr uint32_t LEDC_FREQ = 1000;
constexpr uint8_t  LEDC_RES  = 8;

// Neopixel status LED pin
constexpr uint8_t  STATUS_LED_PIN = PIN_NEOPIXEL;

constexpr uint16_t maxreflec = 800;
constexpr uint16_t minreflec = 300;

// Encoder pins (A/B).
constexpr uint8_t LEFT_ENC_A  = 47;
constexpr uint8_t LEFT_ENC_B  = 48;
constexpr uint8_t RIGHT_ENC_A = 42;
constexpr uint8_t RIGHT_ENC_B = 45;

bool right = false;
bool left = false;
bool once = false;

constexpr int TICKS_PER_90  = 120;
constexpr int TICKS_PER_180 = 200;
constexpr int TURN_SPEED    = 200;   // PWM 0-255, tune to taste
constexpr int NUDGE_SPEED   = 150;   // PWM 0-255, tune to taste
constexpr int NUDGE_TICKS   = 30;    // ticks to move forward when nudging
constexpr uint32_t TURN_GAP_MS = 500;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 50;

// Flip either value to -1 if a wheel's tick sign is reversed in your wiring.
constexpr int LEFT_ENCODER_POLARITY = 1;
constexpr int RIGHT_ENCODER_POLARITY = 1;

// maze state machine & path recording

enum RobotState {
  DISCOVERING, // Explore maze using Right Hand Rule & record decisions
  OPTIMIZING,  // Simplify path, output speed details, and countdown to next run
  SOLVING,     // Replay the optimized path to solve the maze quickly
  FINISHED     // Stop and signal completion (unused in infinite looping mode)
};

RobotState currentState = DISCOVERING;

String path = "";
int pathIndex = 0;
// ENCODER STATE

volatile int leftTicks  = 0;
volatile int rightTicks = 0;

// Forward declarations
void turnright(int speed);
void turnleft(int speed);
void turnAround(int speed);
void resetEncoders();
void nudgeforward(int speed);
void setMotors(int speedA, int speedB);
void stopMotors();
void PID_control();
void discover();
void solve();
void simplifyPath();

void setStatusLed(uint8_t red, uint8_t green, uint8_t blue) {
  neopixelWrite(STATUS_LED_PIN, red, green, blue);
}

// QTR SENSOR SETUP

constexpr uint8_t NUM_SENSORS = 13;

static const uint8_t sensorPins[NUM_SENSORS] = {
  4,   // S1  (right-most)
  18,  // S2  
  5,   // S3  
  8,   // S4  
  6,   // S5  
  9,   // S6  
  7,   // S7  (Center)
  10,  // S8  
  15,  // S9  
  11,  // S10 
  16,  // S11 
  12,  // S12 
  17   // S13 (left-most)
};

QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS];

inline uint16_t qtr_readLine()  { return qtr.readLineBlack(sensorValues); }
inline void     qtr_calibrate() { qtr.calibrate(); }

// PID + SPEED GLOBALS


int BASE_SPEED = 120;   // PWM 0-255, forward cruise speed (increases by 10 per run)
int MAX_SPEED  = 200;   // PWM limit for PID control
int MIN_SPEED  = -180;  // allow reverse correction on tight turns

constexpr int LINE_CENTER = 6000;

float Kp = 0.058f;
float Ki = 0.0f;
float Kd = 0.25f;

int P = 0;
int I = 0;
int D = 0;
int lastError = 0;

// MOTOR DRIVER CONTROL

void setMotors(int speedA, int speedB) {
  // Left motor (A channel)
  if (speedA >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    speedA = -speedA;
  }
  ledcWrite(LEDC_CH_A, static_cast<uint8_t>(constrain(speedA, 0, 255)));

  // Right motor (B channel) 
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
  delay(5);  // allow time for motors to settle
}

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

//  PID LINE FOLLOWING

void PID_control() {
  uint16_t pos = qtr_readLine();
  int error = LINE_CENTER - static_cast<int>(pos);

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;

  I = constrain(I, -10000, 10000);

  int correction = static_cast<int>(P * Kp + I * Ki + D * Kd);

  int speedA = BASE_SPEED + correction;
  int speedB = BASE_SPEED - correction;

  speedA = constrain(speedA, MIN_SPEED, MAX_SPEED);
  speedB = constrain(speedB, MIN_SPEED, MAX_SPEED);

  setMotors(speedA, speedB);
}
// CALIBRATION

void runCalibration() {
  stopMotors();
  for (uint16_t i = 0; i < 250; i++) {
    qtr_calibrate();
    delay(5);
  }
  stopMotors();
}

// SECTION 9 — SETUP

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

  // ledc PWM
  ledcSetup(LEDC_CH_A, LEDC_FREQ, LEDC_RES);
  ledcSetup(LEDC_CH_B, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(PWMA, LEDC_CH_A);
  ledcAttachPin(PWMB, LEDC_CH_B);
  
  // Encoders
  pinMode(LEFT_ENC_A,  INPUT);
  pinMode(LEFT_ENC_B,  INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  leftEncISR,  RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncISR, RISING);

  // QTR
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, NUM_SENSORS);

  stopMotors();
  setStatusLed(0, 0, 255); // Blue during calibration
  runCalibration();
  setStatusLed(0, 255, 0); // Green when ready
  delay(2000);
  setStatusLed(255, 0, 0); // Red at start
  lastError = 0;
  P = I = D = 0;
}

// SECTION 10 — MAIN LOOP

void loop() {
  switch (currentState) {
    case DISCOVERING:
      discover();
      break;

    case OPTIMIZING:
      stopMotors();
      setStatusLed(0, 0, 255); // Solid Blue while processing
      
      // Serial.print("Current Path String: ");
      // Serial.println(path);
      
      simplifyPath();

      // Serial.print("Optimized Path String: ");
      // Serial.println(path);
      
      // Serial.print("Starting solve run with BASE_SPEED: ");
      // Serial.print(BASE_SPEED);
      // Serial.print(" | MAX_SPEED: ");
      // Serial.println(MAX_SPEED);
      delay(1000);

      // Countdown to place the robot back at the start of the maze
      for (int i = 0; i < 10; i++) {
        setStatusLed(0, 255, 0); // Flashing green countdown
        delay(500);
        setStatusLed(0, 0, 0);
        delay(500);
      }

      pathIndex = 0;
      currentState = SOLVING;
      setStatusLed(255, 0, 0); // Solid red for the solve run
      break;

    case SOLVING:
      solve();
      break;

    case FINISHED:
      stopMotors();
      setStatusLed(0, 255, 255); // Unused, runs infinitely
      delay(200);
      setStatusLed(0, 0, 0);
      delay(200);
      break;
  }
}

// Discover the maze using the right-hand rule and record the path

void discover() {
  uint16_t pos = qtr_readLine();

  // 1. Check for T-intersection or Cross (+) Intersection
  bool fullCross = true;
  for (int i = 1; i <= 11; i++) {
    if (sensorValues[i] <= maxreflec) {
      fullCross = false;
      break;
    }
  }

  if (fullCross) {
    setStatusLed(255, 255, 0); // Yellow (Intersection detected)
    stopMotors();
    nudgeforward(NUDGE_SPEED);
    qtr_readLine();

    // Check if we still see black across the board after nudging (End of Maze)
    bool stillFullCross = true;
    for (int i = 1; i <= 11; i++) {
      if (sensorValues[i] <= maxreflec) {
        stillFullCross = false;
        break;
      }
    }

    if (stillFullCross) {
      setStatusLed(255, 0, 255); // Magenta (Found End of Maze)
      stopMotors();
      currentState = OPTIMIZING;
    } else {
      // It was an intersection. Record decision and execute Right Turn (RHR)
      path += 'R';
      right = true;
      left = false;
      turnright(100);
      delay(100);
    }
    return;
  }

  // 2. Check for Left Branch
  bool leftBranch = (sensorValues[8] > maxreflec) &&
                    (sensorValues[9] > maxreflec) &&
                    (sensorValues[10] > maxreflec) &&
                    (sensorValues[11] > maxreflec) &&
                    (sensorValues[12] > maxreflec);

  bool noRightBranch = (sensorValues[0] < minreflec) && 
                      (sensorValues[1] < minreflec) && 
                      (sensorValues[2] < minreflec) && 
                      (sensorValues[3] < minreflec);

  if (leftBranch && noRightBranch) {
    nudgeforward(NUDGE_SPEED);
    stopMotors();
    qtr_readLine();

    // Verify if there is a straight path ahead
    bool straightBranch = false;
    for (int i = 4; i <= 8; i++) {
      if (sensorValues[i] >= minreflec) {
        straightBranch = true;
        break;
      }
    }

    if (!straightBranch) {
      // Forced 90-degree left turn (No choices, do not record in path)
      left = true;
      right = false;
      turnleft(100);
      delay(100);
    } else {
      // Left-T junction. Choice is Left vs Straight.
      // RHR priorities prefer straight over left. Record 'S' decision.
      path += 'S';
      left = false;
      right = false;
      PID_control();
    }
    return;
  }

  // 3. Check for Right Branch
  bool rightBranch = (sensorValues[0] > maxreflec) &&
                      (sensorValues[1] > maxreflec) &&
                      (sensorValues[2] > maxreflec) &&
                      (sensorValues[3] > maxreflec) && 
                      (sensorValues[4] > maxreflec);

  if (rightBranch) {
    nudgeforward(NUDGE_SPEED);
    stopMotors();
    qtr_readLine();

    // Verify if there is a straight path ahead
    bool straightBranch = false;
    for (int i = 4; i <= 8; i++) {
      if (sensorValues[i] >= minreflec) {
        straightBranch = true;
        break;
      }
    }

    if (straightBranch) {
      // Right-T. Options: Right or Straight. Choice is 'R'.
      path += 'R';
    } else {
      // Forced 90-degree right turn (No choices, do not record in path)
    }
    right = true;
    left = false;
    turnright(100);
    delay(150);
    return;
  }

  // 4. Check for Dead End
  bool isDeadEnd = true;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] >= minreflec) {
      isDeadEnd = false;
      break;
    }
  }

  if (isDeadEnd) {
    setStatusLed(255, 0, 0); // Red
    stopMotors();
    nudgeforward(NUDGE_SPEED);
    qtr_readLine();

    bool stillDeadEnd = true;
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (sensorValues[i] >= minreflec) {
        stillDeadEnd = false;
        break;
      }
    }

    if (stillDeadEnd) {
      // Confirmed dead end. Record backtracking step ('B') and perform U-turn.
      path += 'B';
      turnAround(120);
    }
    return;
  }

  // Default: Line tracking using PID
  once = true;
  left = false;
  right = false;
  PID_control();
}

//  MAZE SOLVING 

void solve() {
  uint16_t pos = qtr_readLine();

  // 1. Check for T-intersection or Cross (+) Intersection
  bool fullCross = true;
  for (int i = 1; i <= 11; i++) {
    if (sensorValues[i] <= maxreflec) {
      fullCross = false;
      break;
    }
  }

  if (fullCross) {
    setStatusLed(0, 255, 255); // Cyan
    stopMotors();
    nudgeforward(NUDGE_SPEED);
    qtr_readLine();

    bool stillFullCross = true;
    for (int i = 1; i <= 11; i++) {
      if (sensorValues[i] <= maxreflec) {
        stillFullCross = false;
        break;
      }
    }

    if (stillFullCross) {
      // Reached the end of the maze again!
      setStatusLed(0, 255, 0); // Green
      stopMotors();

      // Increment speeds for the next solve run (capped at 255)
      BASE_SPEED = constrain(BASE_SPEED + 10, 0, 255);
      MAX_SPEED  = constrain(MAX_SPEED + 10, 0, 255);

      // Serial.println("==================================================");
      // Serial.println("Solve run completed!");
      // Serial.print("Adjusting speeds for next run -> ");
      // Serial.print("BASE: "); Serial.print(BASE_SPEED);
      // Serial.print(" | MAX: "); Serial.println(MAX_SPEED);
      // Serial.println("==================================================");

      // Loop back to the countdown state so you can place the robot back at the start
      currentState = OPTIMIZING;
    } else {
      // T or Cross Intersection. Replay stored decision.
      if (pathIndex < path.length()) {
        char decision = path[pathIndex];
        pathIndex++;
        if (decision == 'R') {
          turnright(100);
          delay(100);
        } else if (decision == 'L') {
          turnleft(100);
          delay(100);
        } else if (decision == 'S') {
          // Continue straight
        }
      } else {
        stopMotors();
        setStatusLed(255, 0, 0); // Error: path out of bounds
      }
    }
    return;
  }

  // 2. Check for Left Branch
  bool leftBranch = (sensorValues[8] > maxreflec) &&
                    (sensorValues[9] > maxreflec) &&
                    (sensorValues[10] > maxreflec) &&
                    (sensorValues[11] > maxreflec) &&
                    (sensorValues[12] > maxreflec);

  bool noRightBranch = (sensorValues[0] < minreflec) && 
                        (sensorValues[1] < minreflec) && 
                        (sensorValues[2] < minreflec) && 
                        (sensorValues[3] < minreflec);

  if (leftBranch && noRightBranch) {
    nudgeforward(NUDGE_SPEED);
    stopMotors();
    qtr_readLine();

    bool straightBranch = false;
    for (int i = 4; i <= 8; i++) {
      if (sensorValues[i] >= minreflec) {
        straightBranch = true;
        break;
      }
    }

    if (!straightBranch) {
      // Forced turn: turn left naturally without reading path
      turnleft(100);
      delay(100);
    } else {
      // Left-T Intersection. Replay path decision.
      if (pathIndex < path.length()) {
        char decision = path[pathIndex];
        pathIndex++;
        if (decision == 'L') {
          turnleft(100);
          delay(100);
        } else if (decision == 'S') {
          // Continue straight
        } else if (decision == 'R') {
          turnright(100);
          delay(100);
        }
      } else {
        stopMotors();
        setStatusLed(255, 0, 0);
      }
    }
    return;
  }

  // 3. Check for Right Branch
  bool rightBranch = (sensorValues[0] > maxreflec) &&
                      (sensorValues[1] > maxreflec) &&
                      (sensorValues[2] > maxreflec) &&
                      (sensorValues[3] > maxreflec) && 
                      (sensorValues[4] > maxreflec);

  if (rightBranch) {
    nudgeforward(NUDGE_SPEED);
    stopMotors();
    qtr_readLine();

    bool straightBranch = false;
    for (int i = 4; i <= 8; i++) {
      if (sensorValues[i] >= minreflec) {
        straightBranch = true;
        break;
      }
    }

    if (straightBranch) {
      // Right-T. Replay path decision.
      if (pathIndex < path.length()) {
        char decision = path[pathIndex];
        pathIndex++;
        if (decision == 'R') {
          turnright(100);
          delay(150);
        } else if (decision == 'S') {
          // Continue straight
        } else if (decision == 'L') {
          turnleft(100);
          delay(150);
        }
      } else {
        stopMotors();
        setStatusLed(255, 0, 0);
      }
    } else {
      // Forced turn: turn right naturally
      turnright(100);
      delay(150);
    }
    return;
  }

  // Default: Line tracking using PID
  PID_control();
}
//  SECTION 13 — PATH SIMPLIFICATION

void simplifyPath() {
  bool changed = true;
  while (changed) {
    changed = false;
    int lenBefore = path.length();

    // Standard maze reduction rules (replaces sequences with a dead-end 'B')
    path.replace("LBR", "B");
    path.replace("RBL", "B");
    path.replace("SBL", "R");
    path.replace("LBL", "S");
    path.replace("RBR", "S");
    path.replace("SBR", "L");
    path.replace("LBS", "R");
    path.replace("RBS", "L");
    path.replace("SBS", "B");

    if (path.length() != lenBefore) {
      changed = true;
    }
  }
}
// turn functions using encoders

void turnright(int speed) {
  resetEncoders();
  setMotors(speed, -speed);
  while (abs(leftTicks) < TICKS_PER_90) {
    delayMicroseconds(100);
  }
  stopMotors();
}

void turnleft(int speed) {
  resetEncoders();
  setMotors(-speed, speed);
  while (abs(rightTicks) < TICKS_PER_90) {
    delayMicroseconds(100);
  }
  stopMotors();
}

void turnAround(int speed) {
  resetEncoders();
  setMotors(speed, -speed);
  while (abs(leftTicks) < TICKS_PER_180) {
    delayMicroseconds(100);
  }
  stopMotors();
}

void resetEncoders() {
  leftTicks  = 0;
  rightTicks = 0;
}

void nudgeforward(int speed) {
  resetEncoders();
  setMotors(speed, speed);
  while (abs(leftTicks) < NUDGE_TICKS && abs(rightTicks) < NUDGE_TICKS) {
    delayMicroseconds(100);
  }
  stopMotors();
}