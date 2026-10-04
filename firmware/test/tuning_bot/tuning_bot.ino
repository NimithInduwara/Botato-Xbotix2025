#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <VL53L0X_mod.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

// =============================================================
// HARDWARE PINS (Arduino Nano V3.0)
// =============================================================
// Left Motor (A)
#define PWMA_PIN 6
#define AIN1_PIN 8
#define AIN2_PIN 7

// Right Motor (B)
#define PWMB_PIN 9
#define BIN1_PIN 12
#define BIN2_PIN 10

// Encoders
#define encoder_left_A 2   // (yellow wire) - MUST be Pin 2 (Interrupt 0)
#define encoder_left_B 4   // (green wire)
#define encoder_right_A 3  // (yellow wire) - MUST be Pin 3 (Interrupt 1)
#define encoder_right_B 5  // (green wire)

#define TCAADDR 0x70  // I2C Multiplexer for ToF

// Arm Servos
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 170
#define SERVOMAX 700
#define SERVO_FREQ 50

// =============================================================
// GLOBALS: ARM SERVO
// =============================================================
// We will start them at 92 degrees (center).
int currentAngles[4] = { 0, 20, 60, 30 };


// =============================================================
// GLOBALS: ROBOT DIMENSIONS & MOTOR PID
// =============================================================
const float WHEEL_CIRCUMFERENCE_CM = 21.75;
const float TRACK_WIDTH_CM = 19;

volatile long left_pulses = 0;
volatile long right_pulses = 0;

// Motor PID Variables (Suffixed with _motor)
float Kp_motor = 4.0;
float Ki_motor = 0.000;
float Kd_motor = 0.01;
const float anglePerCount = 360.0 / 2138.0;
volatile bool positionMoveActive = false;

// =============================================================
// GLOBALS: WALL FOLLOWER
// =============================================================
float Kp_wall = 1;    // Proportional: How hard it turns towards/away from the wall
float Kd_wall = 4.0;  // Derivative: Dampens the steering to prevent "snaking"
int last_wall_error = 0;


struct MotorState {
  float lastError = 0;
  float integral = 0;
  float targetDeg = 0;
  unsigned long lastTime = 0;
};

MotorState leftMotorState;
MotorState rightMotorState;

// =============================================================
// NEW: INDEPENDENT VELOCITY PID CONTROL VARIABLES
// =============================================================
volatile int targetLeftSpeed = 0;
volatile int targetRightSpeed = 0;

// Velocity state tracking
volatile long prev_left_pulses = 0;
volatile long prev_right_pulses = 0;
float integral_left = 0;
float integral_right = 0;

// Velocity Tuning Parameters
float Kp_vel = 1.5;  // Proportional correction
float Ki_vel = 0.1;  // Integral correction

// SCALAR: Maps your 0-255 PWM input to "Expected Encoder Pulses per 10ms"
float PULSES_PER_PWM = 0.0860;

// =============================================================
// INTERRUPT SERVICE ROUTINES (ENCODERS)
// =============================================================
// Standard AVR ISRs (No IRAM_ATTR needed for Nano)
void leftEncoderISR() {
  bool stateA = digitalRead(encoder_left_A);
  bool stateB = digitalRead(encoder_left_B);
  if (stateA == stateB) left_pulses--;
  else left_pulses++;
}

void rightEncoderISR() {
  bool stateA = digitalRead(encoder_right_A);
  bool stateB = digitalRead(encoder_right_B);
  if (stateA == stateB) right_pulses++;
  else right_pulses--;
}

// =============================================================
// MOTOR CONTROLLER FUNCTIONS
// =============================================================
void driveMotor(int speed, int in1, int in2, int en) {
  if (speed == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(en, 0);
    return;
  }

  speed = constrain(speed, -255, 255);
  int pwm = abs(speed);

  if (speed > 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  analogWrite(en, pwm);
}

void drive(int leftSpeed, int rightSpeed) {
  targetLeftSpeed = leftSpeed;
  targetRightSpeed = rightSpeed;
}

//auto-correct function (runs automatically every 10ms in the background for motors)
void updateSpeedControl() {
  // If we are commanding a full stop, kill power and reset the integrals
  // so the motors don't "wind up" and twitch while standing still.
  if (targetLeftSpeed == 0 && targetRightSpeed == 0) {
    driveMotor(0, AIN2_PIN, AIN1_PIN, PWMA_PIN);
    driveMotor(0, BIN2_PIN, BIN1_PIN, PWMB_PIN);
    integral_left = 0;
    integral_right = 0;
    return;
  }

  // 1. Calculate actual velocity (pulses traveled in the last 10ms)
  long current_left = left_pulses;
  long current_right = right_pulses;
  long vel_left = current_left - prev_left_pulses;
  long vel_right = current_right - prev_right_pulses;

  prev_left_pulses = current_left;
  prev_right_pulses = current_right;

  // 2. Calculate target velocity (Expected pulses per 10ms)
  float target_vel_left = targetLeftSpeed * PULSES_PER_PWM;
  float target_vel_right = targetRightSpeed * PULSES_PER_PWM;

  // 3. Calculate Error (Target vs Actual)
  float error_left = target_vel_left - vel_left;
  float error_right = target_vel_right - vel_right;

  // 4. Update Integrals (with windup protection capped at 50)
  integral_left += error_left;
  integral_right += error_right;
  integral_left = constrain(integral_left, -50, 50);
  integral_right = constrain(integral_right, -50, 50);

  // 5. Calculate new PWM (Base target + Correction)
  int newLeftPWM = targetLeftSpeed + (Kp_vel * error_left) + (Ki_vel * integral_left);
  int newRightPWM = targetRightSpeed + (Kp_vel * error_right) + (Ki_vel * integral_right);

  // Constrain outputs to valid PWM ranges
  newLeftPWM = constrain(newLeftPWM, -255, 255);
  newRightPWM = constrain(newRightPWM, -255, 255);

  // Apply to motors
  driveMotor(newLeftPWM, AIN2_PIN, AIN1_PIN, PWMA_PIN);
  driveMotor(newRightPWM, BIN2_PIN, BIN1_PIN, PWMB_PIN);
}

// =============================================================
// BACKGROUND SYNC TIMER (Timer2)
// =============================================================
void setupBackgroundSync() {
  noInterrupts();  // Disable interrupts while configuring
  TCCR2A = 0;      // Reset entire TCCR2A to 0
  TCCR2B = 0;      // Reset entire TCCR2B to 0
  TCNT2 = 0;       // Initialize counter value to 0

  // Set compare match register for ~10ms intervals
  // 16MHz / 1024 prescaler = 15625 Hz. 15625 / 156 = ~100 Hz
  OCR2A = 156;

  TCCR2A |= (1 << WGM21);                             // Turn on CTC (Clear Timer on Compare) mode
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);  // Set CS22, CS21, CS20 for 1024 prescaler
  TIMSK2 |= (1 << OCIE2A);                            // Enable timer compare interrupt
  interrupts();                                       // Re-enable interrupts
}

// =============================================================
// BACKGROUND SYNC TIMER ISR — respect position move flag
// =============================================================
ISR(TIMER2_COMPA_vect) {
  if (!positionMoveActive) {
    updateSpeedControl();   // velocity PID only runs during free driving
  }
}
// =============================================================
// SETUP & LOOP
// =============================================================
void initMotors() {
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);

  pinMode(encoder_left_A, INPUT_PULLUP);
  pinMode(encoder_left_B, INPUT_PULLUP);
  pinMode(encoder_right_A, INPUT_PULLUP);
  pinMode(encoder_right_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder_left_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_right_A), rightEncoderISR, CHANGE);
}

// =============================================================
// GLOBALS: TIME OF FLIGHT (VL53L0X)
// =============================================================
VL53L0X_mod tofSensors[5];
uint16_t tofDistances[5] = { 0, 0, 0, 0, 0 };
bool sensorReady[5] = { false, false, false, false, false };
int16_t tofOffsets[5];

// --- Legacy-Safe Channel & Index Mapping ---
const uint8_t RIGHT_CH = 0;   // Legacy
const uint8_t FRONT_CH = 1;   // Legacy
const uint8_t LEFT_CH = 2;    // Legacy
const uint8_t FRONTR_CH = 3;  // New
const uint8_t FRONTL_CH = 4;  // New

// --- Filtering Variables ---
float filteredDistances[5] = { 0, 0, 0, 0, 0 };
const float EMA_ALPHA = 0.4;  // 0.1 (Smooth) to 1.0 (Raw)

// =============================================================
// I2C MULTIPLEXER FUNCTION (For ToF)
// =============================================================
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
  delay(2);
}

bool onCenterLine = false;
float armLength = 12;
bool task1done = false;
bool task2done = false;
bool task3done = false;
float IRtoWheel = 16;
float ToFtoWheel = 13;
float LinetoPlatform = 21;
int hutti = -1;
unsigned long ignoreRightUntil = 0;
unsigned long ignoreLeftUntil = 0;
unsigned long rightDetectStartTime = 0;
unsigned long leftDetectStartTime = 0;
unsigned long frontDetectStartTime = 0;
const unsigned long DETECTION_THRESHOLD_MS = 150;
const unsigned long DETECTION_THRESHOLD_MS_front = 100;
int BoxCount = 1;



// =============================================================
// TOF SENSOR FUNCTIONS
// =============================================================
// 2=left back tof
// 1=center tof
// 0=right back tof

void updateToF() {
  for (int i = 0; i < 5; i++) {
    tcaselect(i);
    uint16_t rawMeasurement;

    if (tofSensors[i].readRangeNoBlocking(rawMeasurement)) {
      sensorReady[i] = true;

      uint16_t currentTarget = 800;  // Default to 'clear path'

      if (!tofSensors[i].timeoutOccurred()) {
        int16_t calibratedValue = rawMeasurement - tofOffsets[i];

        if (rawMeasurement > 800 || calibratedValue >= 800) {
          currentTarget = 800;
        } else if (calibratedValue <= 0) {
          currentTarget = 0;
        } else {
          currentTarget = calibratedValue;
        }
      }

      // Exponential Moving Average Filter
      if (filteredDistances[i] == 0 || currentTarget == 800) {
        filteredDistances[i] = currentTarget;
      } else {
        filteredDistances[i] = (EMA_ALPHA * currentTarget) + ((1.0 - EMA_ALPHA) * filteredDistances[i]);
      }

      tofDistances[i] = (uint16_t)filteredDistances[i];
    }
  }
}


void printToFData() {
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 50) {
    Serial.print("L(2): ");
    if (!sensorReady[LEFT_CH]) Serial.print("Wait");
    else Serial.print(tofDistances[LEFT_CH]);
    Serial.print(" | ");
    Serial.print("FL(4): ");
    if (!sensorReady[FRONTL_CH]) Serial.print("Wait");
    else Serial.print(tofDistances[FRONTL_CH]);
    Serial.print(" | ");
    Serial.print("F(1): ");
    if (!sensorReady[FRONT_CH]) Serial.print("Wait");
    else Serial.print(tofDistances[FRONT_CH]);
    Serial.print(" | ");
    Serial.print("FR(3): ");
    if (!sensorReady[FRONTR_CH]) Serial.print("Wait");
    else Serial.print(tofDistances[FRONTR_CH]);
    Serial.print(" | ");
    Serial.print("R(0): ");
    if (!sensorReady[RIGHT_CH]) Serial.print("Wait");
    else Serial.print(tofDistances[RIGHT_CH]);
    Serial.println();
    lastPrintTime = millis();
  }
}

void initToFSensors() {
  EEPROM.get(0, tofOffsets);
  Serial.println("\n--- Loaded EEPROM Offsets ---");
  Serial.print("Right  (0): ");
  Serial.println(tofOffsets[RIGHT_CH]);
  Serial.print("Front  (1): ");
  Serial.println(tofOffsets[FRONT_CH]);
  Serial.print("Left   (2): ");
  Serial.println(tofOffsets[LEFT_CH]);
  Serial.print("FrontR (3): ");
  Serial.println(tofOffsets[FRONTR_CH]);
  Serial.print("FrontL (4): ");
  Serial.println(tofOffsets[FRONTL_CH]);
  Serial.println("-----------------------------\n");
  Serial.println("Powering up 5 ToF sensors...");
  delay(500);
  for (int i = 0; i < 5; i++) {
    tcaselect(i);
    delay(20);
    tofSensors[i].setTimeout(500);
    bool initSuccess = false;
    for (int retry = 0; retry < 3; retry++) {
      if (tofSensors[i].init()) {
        initSuccess = true;
        break;
      }
      delay(100);
    }
    if (!initSuccess) {
      Serial.print("Failed to init ToF on channel ");
      Serial.println(i);
    } else {
      tofSensors[i].startContinuous(50);
    }
  }
  Serial.println("ToF Init Complete. Running in background.");
}

uint16_t getToFDis(int tofIndex) {
  updateToF();
  return tofDistances[tofIndex];
}

int calculateMotorSpeedMaze(MotorState &M, long currentPulses) {
  unsigned long now = millis();
  float dt = (now - M.lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;
  M.lastTime = now;

  float motorDeg = currentPulses * anglePerCount;
  float error = M.targetDeg - motorDeg;

  if (abs(error) < 80) {
    M.integral += error * dt;
  }
  M.integral = constrain(M.integral, -120, 120);

  float derivative = (error - M.lastError) / dt;
  M.lastError = error;

  float control = Kp_motor * error + Ki_motor * M.integral + Kd_motor * derivative;

  // --- CHANGE: Dead-zone tightened from 3 → 1.5 so PID keeps correcting overshoot ---
  if (abs(error) < 1.5f) return 0;

  int pwmVal = min(abs(control), 255.0f);

  // --- CHANGE: Near-target? Use a gentler speed range so corrections aren't too aggressive ---
  int finalSpeed;
  if (abs(error) < 30) {
    finalSpeed = map(pwmVal, 0, 255, 30, 80);  // fine-correction range
  } else {
    finalSpeed = map(pwmVal, 0, 255, 45, 120); // normal cruise range
  }

  if (control < 0) finalSpeed = -finalSpeed;
  return finalSpeed;
}




// =============================================================
// UPDATED executeMovement — drives motors directly, no ISR conflict
// =============================================================
void executeMovement(float leftTargetDeg, float rightTargetDeg) {
  leftMotorState.targetDeg  = leftTargetDeg;
  rightMotorState.targetDeg = rightTargetDeg;

  left_pulses  = 0;
  right_pulses = 0;

  leftMotorState.lastTime   = millis();
  rightMotorState.lastTime  = millis();
  leftMotorState.integral   = 0;
  rightMotorState.integral  = 0;
  leftMotorState.lastError  = 0;
  rightMotorState.lastError = 0;

  // Suspend velocity PID so it doesn't fight the position loop
  positionMoveActive = true;
  // Also zero the velocity loop's integrals so they don't wind up
  // while the ISR is paused, and don't cause a lurch when it resumes
  integral_left  = 0;
  integral_right = 0;

  const float         SETTLE_TOLERANCE_DEG = 2.0f;  // degrees
  const unsigned long SETTLE_TIME_MS       = 120;    // ms
  unsigned long settledSince = 0;

  while (true) {
    noInterrupts();
    long current_left  = left_pulses;
    long current_right = right_pulses;
    interrupts();

    int leftSpeed  = calculateMotorSpeedMaze(leftMotorState,  current_left);
    int rightSpeed = calculateMotorSpeedMaze(rightMotorState, current_right);

    // Drive motors DIRECTLY — bypasses the velocity PID entirely
    driveMotor(leftSpeed,  AIN2_PIN, AIN1_PIN, PWMA_PIN);
    driveMotor(rightSpeed, BIN2_PIN, BIN1_PIN, PWMB_PIN);

    float leftErr  = abs(leftMotorState.targetDeg  - current_left  * anglePerCount);
    float rightErr = abs(rightMotorState.targetDeg - current_right * anglePerCount);

    if (leftErr < SETTLE_TOLERANCE_DEG && rightErr < SETTLE_TOLERANCE_DEG) {
      if (settledSince == 0) {
        settledSince = millis();
      } else if (millis() - settledSince >= SETTLE_TIME_MS) {
        break;
      }
    } else {
      settledSince = 0;
    }

    delay(10);
  }

  // Hard stop
  driveMotor(0, AIN2_PIN, AIN1_PIN, PWMA_PIN);
  driveMotor(0, BIN2_PIN, BIN1_PIN, PWMB_PIN);

  // Also sync the drive() target so the velocity loop doesn't
  // lurch when it resumes
  targetLeftSpeed  = 0;
  targetRightSpeed = 0;

  // Resume velocity PID for wall-following etc.
  positionMoveActive = false;
}


void TurnBot(float BotTurnDeg) {
  float WheelTurnDeg = BotTurnDeg * (22.0 / 7.0) * TRACK_WIDTH_CM / WHEEL_CIRCUMFERENCE_CM;
  executeMovement(WheelTurnDeg, -WheelTurnDeg);
}

void GoDistance(float BotGoDis) {
  float WheelGoDeg = BotGoDis * 360.0 / WHEEL_CIRCUMFERENCE_CM;
  executeMovement(WheelGoDeg, WheelGoDeg);
}

void stop(int t) {
  drive(0, 0);
  delay(t * 1000);
}

void OneSidedWallMaze(char wallside, float distance_cm) {
  int target_mm = distance_cm * 10;
  int current_mm = (wallside == 'L' || wallside == 'l') ? tofDistances[LEFT_CH] : tofDistances[RIGHT_CH];

  if (current_mm > (target_mm + 50)) {
    drive(120, 120);
    return;
  }

  int error = current_mm - target_mm;
  int correction = (error * Kp_wall) + ((error - last_wall_error) * Kd_wall);
  last_wall_error = error;

  correction = constrain(correction, -60, 60);

  int base_speed = 120;
  if (wallside == 'R' || wallside == 'r') {
    drive(base_speed + correction, base_speed - correction);
  } else {
    drive(base_speed - correction, base_speed + correction);
  }
}

static uint16_t stableToF(int channel, int samples = 4) {
  uint16_t minVal = 800;
  for (int i = 0; i < samples; i++) {
    updateToF();
    uint16_t v = tofDistances[channel];
    if (v < minVal) minVal = v;
    delay(8);
  }
  return minVal;
}

#define MAZE_OPEN_MM 220       // mm  — farther than this = open path
#define MAZE_WALL_MM 150       // mm  — closer than this  = wall confirmed
#define JUNCTION_OFFSET 15.0f  // cm  — overshoot before turning at junction

void MazeSolverRightHand() {

  Serial.println("MazeSolver: starting right-hand rule");

  for (int i = 0; i < 10; i++) {
    updateToF();
    delay(20);
  }

  last_wall_error = 0;

  while (true) {

    // ---- 1. Read all three directions with stable multi-samples ----
    uint16_t rightDist = stableToF(RIGHT_CH);
    uint16_t frontDist = stableToF(FRONT_CH);
    uint16_t leftDist = stableToF(LEFT_CH);

    bool rightOpen = (rightDist > MAZE_OPEN_MM);
    bool frontOpen = (frontDist > MAZE_OPEN_MM);
    bool leftOpen = (leftDist > MAZE_OPEN_MM);

    Serial.print("R:");
    Serial.print(rightDist);
    Serial.print(" F:");
    Serial.print(frontDist);
    Serial.print(" L:");
    Serial.println(leftDist);

    // ---- 2. END CONDITION — customise for your maze ----------------

    // ---- 3. Right-hand rule decision tree --------------------------

    if (rightOpen) {
      // A corridor opened on the right — take it.
      Serial.println("-> Turn RIGHT");
      GoDistance(JUNCTION_OFFSET);
      delay(150);
      TurnBot(90);
      stop(1);
      delay(150);
      GoDistance(JUNCTION_OFFSET);

    } else if (frontOpen) {
      // Right wall is present; straight ahead is clear.
      OneSidedWallMaze('R', 14);

    } else if (leftOpen) {
      // Right and front are blocked; only left is clear — turn left.
      stop(1);
      Serial.println("-> Turn LEFT");
      TurnBot(-90);
      stop(1);
      delay(150);

    } else {
      // All three directions blocked — dead end, do a U-turn.
      Serial.println("-> U-TURN (dead end)");
      stop(1);
      TurnBot(180);
      delay(150);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  initMotors();
  initToFSensors();
  setupBackgroundSync();
}

void loop() {
  TurnBot(90);
  stop(3);
}


