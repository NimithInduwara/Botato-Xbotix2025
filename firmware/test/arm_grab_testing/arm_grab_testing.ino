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
const float WHEEL_CIRCUMFERENCE_CM = 18.85;
const float TRACK_WIDTH_CM = 17.675;

volatile long left_pulses = 0;
volatile long right_pulses = 0;

// Motor PID Variables (Suffixed with _motor)
float Kp_motor = 4.0;
float Ki_motor = 0.000;
float Kd_motor = 0.01;
const float anglePerCount = 360.0 / 2138.0;

// =============================================================
// GLOBALS: WALL FOLLOWER
// =============================================================
float Kp_wall = 4;    // Proportional: How hard it turns towards/away from the wall
float Kd_wall = 2.0;  // Derivative: Dampens the steering to prevent "snaking"
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

// This is the background task that runs automatically every 10ms
ISR(TIMER2_COMPA_vect) {
  updateSpeedControl();
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
// GLOBALS: LINE FOLLOWER (ADS1115)
// =============================================================
Adafruit_ADS1115 ads1;  // Sensors 1-4 (Address 0x48)
Adafruit_ADS1115 ads2;  // Sensors 5-8 (Address 0x49)

constexpr uint8_t SensorCount = 8;
int16_t sensorValues[SensorCount];
int16_t calibMin[SensorCount];
int16_t calibMax[SensorCount];
int16_t calibRange[SensorCount];
uint16_t lastPosition = 0;

// Line Follower PID Variables (Suffixed with _line)
float kp_line = 0.25;
float kd_line = 0.008;
unsigned long prevTime_line = 0;
int last_value_line = 0;

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
// =============================================================
// LINE FOLLOWER FUNCTIONS
// =============================================================
void readRawSensors(int16_t *rawArray) {
  for (uint8_t i = 0; i < 4; i++) {
    rawArray[i] = ads1.readADC_SingleEnded(i);
    rawArray[i + 4] = ads2.readADC_SingleEnded(i);
  }
}

void readCalibrated(int16_t *calibratedValues) {
  int16_t raw[SensorCount];
  readRawSensors(raw);
  for (uint8_t i = 0; i < SensorCount; i++) {
    int32_t val = 0;
    if (calibRange[i] != 0) {
      val = ((int32_t)raw[i] - calibMin[i]) * 1000 / calibRange[i];
    }
    if (val < 0) val = 0;
    else if (val > 1000) val = 1000;
    calibratedValues[i] = val;
  }
}

uint16_t readLineWhite(int16_t *sensorArray) {
  bool onLine = false;
  uint32_t avg = 0;
  uint16_t sum = 0;
  readCalibrated(sensorArray);

  for (uint8_t i = 0; i < SensorCount; i++) {
    sensorArray[i] = 1000 - sensorArray[i];  // Invert for white line
    if (sensorArray[i] > 200) onLine = true;
    if (sensorArray[i] > 50) {
      avg += (uint32_t)sensorArray[i] * (i * 1000);
      sum += sensorArray[i];
    }
  }

  if (!onLine) {
    if (lastPosition < (SensorCount - 1) * 1000 / 2) return 0;
    else return (SensorCount - 1) * 1000;
  }

  lastPosition = avg / sum;
  return lastPosition;
}

void read_line() {
  unsigned long currentTime = micros();
  unsigned long dt_micros = currentTime - prevTime_line;
  if (dt_micros == 0) dt_micros = 1;
  prevTime_line = currentTime;

  // Added the 1,000,000.0 fix here so your math translates directly to seconds
  float dt_line = dt_micros / 1000000.0;

  uint16_t position = readLineWhite(sensorValues);
  Serial.println(position);
  int err = map(position, 0, 7000, -1000, 1000);

  int m1 = 150;
  int m2 = 150;
  int diff = (err * kp_line) + ((err - last_value_line) * kd_line / dt_line);
  last_value_line = err;

  drive(m1 + diff, m2 - diff);
}

void initLineFollower() {
  if (!ads1.begin(0x48) || !ads2.begin(0x49)) {
    Serial.println("Failed to find ADS1115 modules!");
    while (1)
      ;  // Freezes here if they fail to boot
  }

  ads1.setDataRate(RATE_ADS1115_860SPS);
  ads2.setDataRate(RATE_ADS1115_860SPS);
  ads1.setGain(GAIN_ONE);
  ads2.setGain(GAIN_ONE);

  for (uint8_t i = 0; i < SensorCount; i++) {
    calibMin[i] = 32767;
    calibMax[i] = 0;
  }

  Serial.println("\n--- CALIBRATION STARTING ---");
  Serial.println("Keep bot FLAT at 5mm. Sweep far left/right.");
  delay(1000);

  int16_t currentRaw[SensorCount];
  for (uint16_t i = 0; i < 80; i++) {
    readRawSensors(currentRaw);
    for (uint8_t j = 0; j < SensorCount; j++) {
      if (currentRaw[j] < calibMin[j]) calibMin[j] = currentRaw[j];
      if (currentRaw[j] > calibMax[j]) calibMax[j] = currentRaw[j];
    }
    if (i % 8 == 0) Serial.print(".");
  }
  Serial.println();

  for (uint8_t i = 0; i < SensorCount; i++) {
    calibMin[i] += 25;
    calibMax[i] -= 50;
    if (calibMin[i] < 0) calibMin[i] = 0;
    if (calibMax[i] < 0) calibMax[i] = 0;
    calibRange[i] = calibMax[i] - calibMin[i];
  }

  Serial.println("--- CALIBRATION COMPLETE ---");
  // Set the initial dt values so the robot doesn't jerk on the first loop
  prevTime_line = micros();
  last_value_line = 0;
  delay(2000);
}

// =============================================================
// ARM FUNCTIONS
// =============================================================

int angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// New function to handle the slow sweeping motion
void moveServoSlowly_arm(int servoNum, int targetAngle) {
  int startAngle = currentAngles[servoNum];


  // If the target is higher than current, sweep up
  if (startAngle < targetAngle) {
    for (int angle = startAngle; angle <= targetAngle; angle++) {
      pwm.setPWM(servoNum, 0, angleToPulse(angle));
      delay(5);  // Wait a bit before taking the next step
    }
  }
  // If the target is lower than current, sweep down
  else if (startAngle > targetAngle) {
    for (int angle = startAngle; angle >= targetAngle; angle--) {
      pwm.setPWM(servoNum, 0, angleToPulse(angle));
      delay(15);  // Wait a bit before taking the next step
    }
  }

  // Update the array so the Arduino remembers where the servo is now
  currentAngles[servoNum] = targetAngle;
}

void moveServoSlowly_grabber(int servoNum, int targetAngle) {
  int startAngle = currentAngles[servoNum];


  // If the target is higher than current, sweep up
  if (startAngle < targetAngle) {
    for (int angle = startAngle; angle <= targetAngle; angle++) {
      pwm.setPWM(servoNum, 0, angleToPulse(angle));
      delay(50);  // Wait a bit before taking the next step
    }
  }
  // If the target is lower than current, sweep down
  else if (startAngle > targetAngle) {
    for (int angle = startAngle; angle >= targetAngle; angle--) {
      pwm.setPWM(servoNum, 0, angleToPulse(angle));
      delay(50);  // Wait a bit before taking the next step
    }
  }

  // Update the array so the Arduino remembers where the servo is now
  currentAngles[servoNum] = targetAngle;
}


//----------------------------------
//ARM MOVEMENTS
//----------------------------------
void arm_box() {
  moveServoSlowly_arm(1, 180);
}

void arm_hole() {
  moveServoSlowly_arm(1, 120);
}

void arm_mid() {
  moveServoSlowly_arm(1, 70);
}

void arm_storage() {
  moveServoSlowly_arm(1, 0);
}


//----------------------------------
//GRABBER MOVEMENTS
//----------------------------------
void grab() {
  moveServoSlowly_grabber(2, 16);
}

void release() {
  moveServoSlowly_grabber(2, 30);
}

void release_max() {
  moveServoSlowly_grabber(2, 80);
}

void release_slight() {
  moveServoSlowly_grabber(2, 28);
}


//----------------------------------
//SLIDER MOVEMENTS
//----------------------------------
void slider_horizontal() {
  moveServoSlowly_grabber(3, 30);
}

void slider_up() {
  moveServoSlowly_grabber(3, 0);
}

void slider_down() {
  moveServoSlowly_grabber(3, 105);
}


//----------------------------------
//COMBINED MOVEMENTS
//----------------------------------
void grab_box_and_place() {
  delay(300);
  arm_mid();
  delay(300);
  release_max();
  delay(300);
  arm_box();
  delay(300);
  grab();
  delay(300);
  arm_storage();
  delay(300);
  release();
  delay(500);
}

//123
void grab_box_and_keep() {
  delay(100);
  arm_box();
  delay(100);
  release_max();
  delay(300);
  //forward
  GoDistance(6);
  delay(300);
  grab();
  delay(100);
  arm_storage();
  delay(300);
}

void grab_box_to_hole() {
  delay(200);
  release();
  delay(300);
  arm_storage();
  grab();
  delay(300);
  arm_hole();
}

//123
void put_first_box_to_hole() {
  GoDistance(-3);
  delay(300);
  grab();
  delay(300);
  arm_hole();
  delay(1000);
  //go forward
  GoDistance(9);
  stop(1);
  release_slight();
  delay(300);
  GoDistance(-5);
  //go back
}

void put_second_box_to_hole() {
  delay(300);
  grab_box_to_hole();
  //fo front
  release_slight();
  //go back
  //arm_push();
  //go front
  //go back
}

void init_servos() {
  Serial.println("PCA9685 Slow Servo Controller Ready!");
  Serial.println("Type the Servo Number (0 or 1) followed by a Space, then the Target Angle (0-180).");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  // Move both servos to their starting positions (92 degrees) immediately on startup
  pwm.setPWM(1, 0, angleToPulse(currentAngles[1]));
  pwm.setPWM(2, 0, angleToPulse(currentAngles[2]));
  pwm.setPWM(3, 0, angleToPulse(currentAngles[3]));
  delay(500);  // Give them a moment to get to the center
}


//


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

//check if atleeast one IR is detecting white
bool isWhiteLine() {
  // Get the freshest data (0 = White, 1000 = Black)
  readCalibrated(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.println(sensorValues[i]);
    if (sensorValues[i] <= 150) {
      return true;
    }
  }
  return false;
}

//align the bot's wheel parallel to the white line
void centerAlign() {
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    read_line();
  }
}


//align the bot's wheel perpendicular to the white line
void straightAlign() {
  readCalibrated(sensorValues);
  while ((sensorValues[0] >= 125)
         || (sensorValues[1] >= 125)
         || (sensorValues[2] >= 125)
         || (sensorValues[3] >= 125)
         || (sensorValues[4] >= 125)
         || (sensorValues[5] >= 125)
         || (sensorValues[6] >= 125)
         || (sensorValues[7] >= 125)) {
    readCalibrated(sensorValues);
    drive(150, -150);
  }
  drive(0, 0);
}

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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// =============================================================
// PID CALCULATION
// =============================================================
int calculateMotorSpeed(MotorState &M, long currentPulses) {
  unsigned long now = millis();
  float dt = (now - M.lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;
  M.lastTime = now;

  float motorDeg = currentPulses * anglePerCount;
  float error = M.targetDeg - motorDeg;

  if (abs(error) < 80) {
    M.integral += error * dt;
  }
  M.integral = constrain(M.integral, -150, 150);

  float derivative = (error - M.lastError) / dt;
  M.lastError = error;

  float control = Kp_motor * error + Ki_motor * M.integral + Kd_motor * derivative;

  int pwmVal = min(abs(control), 255.0f);
  int finalSpeed = map(pwmVal, 0, 255, 45, 150);

  if (abs(error) < 3) finalSpeed = 0;
  if (control < 0) finalSpeed = -finalSpeed;

  return finalSpeed;
}

// =============================================================
// SHARED MOVEMENT EXECUTION
// =============================================================
void executeMovement(float leftTargetDeg, float rightTargetDeg) {
  leftMotorState.targetDeg = leftTargetDeg;
  rightMotorState.targetDeg = rightTargetDeg;

  left_pulses = 0;
  right_pulses = 0;
  leftMotorState.lastTime = millis();
  rightMotorState.lastTime = millis();

  while (true) {
    noInterrupts();
    long current_left = left_pulses;
    long current_right = right_pulses;
    interrupts();

    int leftSpeed = calculateMotorSpeed(leftMotorState, current_left);
    int rightSpeed = calculateMotorSpeed(rightMotorState, current_right);

    drive(leftSpeed, rightSpeed);

    bool left_done = abs(leftMotorState.targetDeg - current_left * anglePerCount) < 3;
    bool right_done = abs(rightMotorState.targetDeg - current_right * anglePerCount) < 3;

    if (left_done && right_done) break;
    delay(10);
  }
  drive(0, 0);
}

// =============================================================
// MOVEMENT FUNCTIONS
// =============================================================
void TurnBot(float BotTurnDeg) {
  // Uses Track Width of 17.5cm and Wheel Circumference of 20cm
  float WheelTurnDeg = BotTurnDeg * (22.0 / 7.0) * TRACK_WIDTH_CM / WHEEL_CIRCUMFERENCE_CM;
  executeMovement(WheelTurnDeg, -WheelTurnDeg);
}

void GoDistance(float BotGoDis) {
  // 360 degrees of rotation = 20cm of travel
  float WheelGoDeg = BotGoDis * 360.0 / WHEEL_CIRCUMFERENCE_CM;
  executeMovement(WheelGoDeg, WheelGoDeg);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// //Rotate in the same position when angle is given
// void TurnBot(float degrees) {
//   if (degrees == 0) return;  // Nothing to do

//   // 1. Calculate the physical distance each wheel needs to roll
//   float turn_circumference = PI * TRACK_WIDTH_CM;
//   float distance_to_travel = (abs(degrees) / 360.0) * turn_circumference;

//   // 2. Convert that distance to encoder pulses
//   long target_pulses = (distance_to_travel / WHEEL_CIRCUMFERENCE_CM) * 2138.0;

//   // 3. Snapshot the current encoder positions
//   long start_left = left_pulses;
//   long start_right = right_pulses;

//   // 4. Set turning speed and determine direction
//   int turn_speed = 150;  // You can lower this if it turns too aggressively

//   if (degrees > 0) {
//     // Turn Right: Left wheel drives forward, Right wheel drives backward
//     drive(turn_speed, -turn_speed);
//   } else {
//     // Turn Left: Left wheel drives backward, Right wheel drives forward
//     drive(-turn_speed, turn_speed);
//   }

//   // 5. Wait until the wheels have traveled the required arc distance
//   while (true) {
//     long left_traveled = labs(left_pulses - start_left);
//     long right_traveled = labs(right_pulses - start_right);

//     // Average them to account for any micro-slippage
//     long avg_traveled = (left_traveled + right_traveled) / 2;

//     if (avg_traveled >= target_pulses) {
//       break;
//     }

//     delay(1);  // Small delay to keep the background timer running smoothly
//   }

//   // 6. Stop the motors
//   drive(0, 0);
//   delay(200);  // Let the momentum settle before the next command
// }


// //Go specific distance when distance is given in cm
// void GoDistance(float distance_cm) {
//   if (distance_cm == 0) return;  // Nothing to do

//   long target_pulses = (abs(distance_cm) / WHEEL_CIRCUMFERENCE_CM) * 2138.0;

//   // Snapshot the current encoder positions as our starting line
//   long start_left = left_pulses;
//   long start_right = right_pulses;

//   // Choose a cruising speed (positive for forward, negative for reverse)
//   int drive_speed = 150;
//   if (distance_cm < 0) {
//     drive_speed = -150;
//   }

//   drive(drive_speed, drive_speed);

//   while (true) {
//     // labs() : absolute value function for 'long' integers.
//     long left_traveled = labs(left_pulses - start_left);
//     long right_traveled = labs(right_pulses - start_right);

//     // Average them together in case one wheel slipped a tiny bit
//     long avg_traveled = (left_traveled + right_traveled) / 2;

//     // Check if we crossed the finish line
//     if (avg_traveled >= target_pulses) {
//       break;
//     }

//     // A tiny 1ms delay prevents this while loop from completely locking up the Arduino
//     delay(1);
//   }

//   drive(0, 0);
//   delay(200);
// }

//stops the bot when a time is given in seconds
void stop(int t) {
  drive(0, 0);
  delay(t * 1000);
}

//--------------------
//ToF align function
//--------------------
float Kp_align = 0.05;  // Retune this!
float Kd_align = 0.01;  // Retune this!
unsigned long prevTime_align = 0;
float last_align_error = 0;

void AlignWithPlatform(float EndDistance_cm) {
  int target_mm = EndDistance_cm * 10;
  updateToF();
  int current_mm = getToFDis(FRONT_CH);

  if (prevTime_align == 0) prevTime_align = micros();

  while (current_mm > target_mm) {
    unsigned long currentTime = micros();
    unsigned long dt_micros = currentTime - prevTime_align;
    if (dt_micros == 0) dt_micros = 1;
    prevTime_align = currentTime;
    float dt_align = dt_micros / 1000000.0;

    updateToF();
    current_mm = getToFDis(FRONT_CH);
    int distL = getToFDis(FRONTL_CH);
    int distR = getToFDis(FRONTR_CH);

    if (current_mm <= target_mm || current_mm < 15) {
      break;
    }

    int min_dist = min(current_mm, min(distL, distR));

    int diffL = distL - min_dist;
    int diffC = current_mm - min_dist;
    int diffR = distR - min_dist;

    long raw_error = (diffL * -100) + (diffC * 0) + (diffR * 100);

    raw_error = constrain(raw_error, -10000, 10000);
    float err = map(raw_error, -10000, 10000, -1000, 1000);

    float correction_float = (err * Kp_align) + ((err - last_align_error) * Kd_align / dt_align);
    last_align_error = err;

    int correction = constrain((int)correction_float, -60, 60);
    int base_speed = 120;

    int leftPWM = base_speed - correction;
    int rightPWM = base_speed + correction;

    drive(leftPWM, rightPWM);  // This feeds safely into your Timer2 Velocity PID

    delay(5);
  }

  drive(0, 0);
  prevTime_align = 0;
  last_align_error = 0;
}

//bot at the center line turns to the side where the box is located,
//drives towards the box,grabs the box and come back t the line and aligns with the white line
void GoForBox(int Direction) {
  stop(1);
  GoDistance(ToFtoWheel);
  stop(1);
  TurnBot(Direction * 93);
  stop(1);

  AlignWithPlatform(20);
  stop(1);

  //arm code huttooooo
  grab_box_and_keep();

  TurnBot(-182);
  stop(1);
  BoxCount -= 1;
  while (!isWhiteLine()) {
    drive(150, 150);
    onCenterLine = false;
  }
}

// Drives parallel to a specified wall ('L' or 'R') keeping a distance_cm gap.
// NOTE: This must be called repeatedly inside a loop to function properly.
void OneSidedWall(char wallside, float distance_cm) {
  // 1. Force a ToF update to get fresh data
  updateToF();

  // 2. Convert target distance from cm to mm (ToF outputs in mm)
  int target_mm = distance_cm * 10;
  int current_mm = 0;

  // 3. Read the correct sensor based on input
  if (wallside == 'L' || wallside == 'l') {
    current_mm = getToFDis(2);  // 2 = Left ToF
  } else if (wallside == 'R' || wallside == 'r') {
    current_mm = getToFDis(0);  // 0 = Right ToF
  } else {
    return;  // Invalid side input, do nothing
  }

  // Optional: Ignore complete drop-offs (e.g., if the wall suddenly ends)
  if (current_mm >= 600) {
    drive(150, 150);  // Just drive straight if the wall is lost
    return;
  }

  // 4. Calculate error and PD correction
  int error = current_mm - target_mm;
  int correction = (error * Kp_wall) + ((error - last_wall_error) * Kd_wall);
  last_wall_error = error;

  // Constrain correction to prevent the bot from jerking too violently
  correction = constrain(correction, -100, 100);

  int base_speed = 150;
  int leftSpeed = base_speed;
  int rightSpeed = base_speed;

  // 5. Apply correction depending on which wall we are tracking
  if (wallside == 'R' || wallside == 'r') {
    // Too far from Right wall -> error is positive -> Turn Right
    // Turn Right = Left wheel faster, Right wheel slower
    leftSpeed = base_speed + correction;
    rightSpeed = base_speed - correction;
  } else if (wallside == 'L' || wallside == 'l') {
    // Too far from Left wall -> error is positive -> Turn Left
    // Turn Left = Left wheel slower, Right wheel faster
    leftSpeed = base_speed - correction;
    rightSpeed = base_speed + correction;
  }

  // 6. Send the adjusted speeds to your existing motor control
  drive(leftSpeed, rightSpeed);
}

/////////////////////////////
//tasks functions
void task1() {
  while (!task1done) {
    while (isWhiteLine()) {
      OneSidedWall('R', 10);
    }
    stop(1);
    for (int i = 0; i < 5; i++) {
      OneSidedWall('R', 10);
    }
    task1done = true;
  }
}

void task2() {
  while (!task2done) {
    if ((isWhiteLine()) && (!onCenterLine)) {
      stop(1);
      GoDistance(IRtoWheel);
      TurnBot(hutti * 92);
      stop(1);
      GoDistance(-10);
      stop(1);
      for (int i = 0; i < 5; i++) {
        updateToF();
        delay(10);
      }
      onCenterLine = true;


      if (BoxCount == 0) {
        while (!task2done) {
          read_line();
          updateToF();
          int frontDist = getToFDis(1);
          if (frontDist < 100) {
            stop(1);
            TurnBot(-105);
            stop(1);
            GoDistance(-20);
            stop(1);
            for (int i = 0; i < 5; i++) {
              updateToF();
              delay(10);
            }
            for (int i = 0; i < 40; i++) {
              OneSidedWall('R', 10);
            }
            while (!isWhiteLine()) {
              OneSidedWall('R', 10);
            }
            GoDistance(5);
            task2done = true;
            stop(1);
          }
        }
      }

      // Give both a small initial cooldown when first hitting the line
      ignoreRightUntil = millis() + 1500;
      ignoreLeftUntil = millis() + 1500;

    } else if (onCenterLine) {
      while (onCenterLine) {
        read_line();  // Keep following the line

        int rightDist = getToFDis(0);
        int leftDist = getToFDis(2);
        int frontDist = getToFDis(1);  // Keeping this if you plan to uncomment front logic

        // --- RIGHT SIDE CHECK ---
        if (millis() > ignoreRightUntil) {
          if (rightDist < 500 && rightDist > 400) {
            GoForBox(1);
            hutti = 1;
            // Set cooldown ONLY for the right side after returning
            ignoreRightUntil = millis() + 1500;
          }
        }

        // --- LEFT SIDE CHECK ---
        if (millis() > ignoreLeftUntil) {
          if (leftDist < 500 && leftDist > 400) {
            GoForBox(-1);
            hutti = -1;
            // Set cooldown ONLY for the left side after returning
            ignoreLeftUntil = millis() + 1500;
          }
        }

        // Front logic can go here, entirely unaffected by side cooldowns
        if (frontDist < 100) {
          stop(1);
          TurnBot(-105);
          stop(1);
          GoDistance(-20);
          stop(1);
          for (int i = 0; i < 5; i++) {
            updateToF();
            delay(10);
          }
          for (int i = 0; i < 40; i++) {
            OneSidedWall('R', 10);
          }
          while (!isWhiteLine()) {
            OneSidedWall('R', 10);
          }
          GoDistance(5);
          task2done = true;
          stop(1);
        }
      }
    } else {
      OneSidedWall('R', 10);
    }
  }
}

// =============================================================
// NEW: LINE FOLLOWER WITH JUNCTION DETECTION
// =============================================================

void task3() {
  while (!task3done) {
    readCalibrated(sensorValues);

    bool w[8];
    int white_count = 0;

    for (int i = 0; i < SensorCount; i++) {
      // Using 200 as the white threshold (based on your isWhiteLine function)
      w[i] = (sensorValues[i] < 200);
      if (w[i]) white_count++;
    }


    bool dead_end = (white_count >= 6 && w[0] && w[7]);

    bool t_left = (w[0] && w[1] && w[2] && !w[6] && !w[7]);


    // 3. Act on Junctions
    if (dead_end) {
      stop(1);
      stop(1);
      put_first_box_to_hole();
      while (1)
        ;

    } else if (t_left) {
      stop(1);
      Serial.println("Left T-Junction Detected!");
      GoDistance(IRtoWheel);
      stop(1);
      TurnBot(-90);
      stop(1);
    }

    unsigned long currentTime = micros();
    unsigned long dt_micros = currentTime - prevTime_line;
    if (dt_micros == 0) dt_micros = 1;
    prevTime_line = currentTime;

    float dt_line = dt_micros / 1000000.0;

    uint16_t position = readLineWhite(sensorValues);
    int err = map(position, 0, 7000, -1000, 1000);

    int m1 = 150;
    int m2 = 150;
    int diff = (err * kp_line) + ((err - last_value_line) * kd_line / dt_line);
    last_value_line = err;

    drive(m1 + diff, m2 - diff);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Clean, modular setup calls
  initMotors();
  initLineFollower();
  initToFSensors();
  init_servos();
  setupBackgroundSync();
}

void loop() {
  task1();
  task2();
  task3();
}
