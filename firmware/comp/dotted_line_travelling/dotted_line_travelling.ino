#include <QTRSensors.h>

#define leftIR 52
#define rightIR 50

#define mr1 5
#define mr2 4
#define ml1 6
#define ml2 7
#define enl 11
#define enr 12

#define mrEncA 3
#define mrEncB 2
#define mlEncA 19
#define mlEncB 18

#define trigger 48
#define echoL 42
#define echoR 46
#define echoF 44

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


float kp_line = 0.08;
float kd_line = 3;
float dt_line;
unsigned long prevTime_line = 0;
int last_value_line;


const float anglePerCount = 360.0 / (48.0 * 34.0);

float Kp = 3.5;
float Ki = 0.02;
float Kd = 0.15;

struct Motor {
  int IN1, IN2, EN;
  int encA, encB;

  volatile long encoderPos = 0;

  float lastError = 0;
  float integral = 0;
  float targetDeg = 0;

  unsigned long lastTime = 0;
};

Motor motorA;
Motor motorB;

//Variable defining for task 1
int armLength = 18;
bool GoBack = false;
bool FirstBox = false;
bool SecondBox = false;
bool SecondBlue = false;
String FirstBoxColour = "UNKNOWN";
String SecondBoxColour = "UNKNOWN";
int box1try = 0;
int box2try = 0;
bool DoneCross = false;
bool task1done = false;

//Variable defining for task 2
bool RightTurned=false;
String TowerColours[3]={"UNKNOWN","UNKNOWN","UNKNOWN"};
int TowerCount=0;

enum dirType { NONE_DIR,
               BOTH,
               LEFT,
               RIGHT };

enum juncType { NONE,
                DEAD_END,
                T_JUNC,
                CROSS,
                LEFT_BEND,
                RIGHT_BEND,
                LEFT_BRANCH,
                RIGHT_BRANCH };


// Define the pins for each segment
const int segA = 34;
const int segB = 32;
const int segC = 24;
const int segD = 28;
const int segE = 30;
const int segF = 36;
const int segG = 38;


//-------Setting up the servo-------

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create the PCA9685 driver object (default I2C address 0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- Servo Configuration ---
// Typical values for MG90S / MG995 at 50 Hz; calibrate these for your servos!
#define SERVOMIN 120   // ~0.8–1.0 ms (0°)
#define SERVOMAX 605   // ~2.0–2.1 ms (180°)
#define SERVO_FREQ 50  // Standard 50 Hz for analog servos

// Define which PCA9685 channel each servo is on
#define SERVO1_CHANNEL 7  // Gripper (MG90S)
#define SERVO2_CHANNEL 8  // Arm (MG995)

// Current angle tracking (for smooth moves)
int servo1_angle = 40;   // Initial angle for gripper
int servo2_angle = 68;  // Initial angle for arm

#include <EEPROM.h>
#include "Adafruit_TCS34725.h"
#define led1 8
#define led2 9
#define led3 10

// ---------------------- MULTIPLEXER ----------------------
#define TCA_ADDR 0x70  // Default address for TCA9548A

void tcaSelect(uint8_t channel) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);  // Select channel (0–7)
  Wire.endTransmission();
}

// ---------------------- SENSOR OBJECTS ----------------------
Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// ---------------------- EEPROM ADDRESSES ----------------------
#define EEPROM_FLAG2 16
#define EEPROM_R2 20
#define EEPROM_G2 24
#define EEPROM_B2 28

// ---------------------- CALIBRATION VARIABLES ----------------------
float cal1_R, cal1_G, cal1_B;  // Sensor 1 calibration (live)
float cal2_R, cal2_G, cal2_B;
float cal3_R, cal3_G, cal3_B;  // Sensor 2 calibration (EEPROM)

void livebackcalibration() {

  tcaSelect(0);  // TCS34725 on channel 0 back colour sensor
  if (!tcs.begin()) {
    Serial.println("Sensor back NOT found!");

  } else {
    Serial.println("Sensor back Detected");
    delay(200);
  }

  float sumR = 0, sumG = 0, sumB = 0;
  uint16_t r, g, b, c;
  Serial.println("Measuring...");

  for (int i = 0; i < 20; i++) {
    tcs.getRawData(&r, &g, &b, &c);
    if (c == 0) c = 1;
    sumR += (float)r / c * 256.0;
    sumG += (float)g / c * 256.0;
    sumB += (float)b / c * 256.0;
    delay(200);
  }

  cal1_R = sumR / 20.0;
  cal1_G = sumG / 20.0;
  cal1_B = sumB / 20.0;

  Serial.print("Ambient Calibration Values:\n");
  Serial.print("R: ");
  Serial.print(cal1_R);
  Serial.print("  G: ");
  Serial.print(cal1_G);
  Serial.print("  B: ");
  Serial.println(cal1_B);
  delay(3000);

  Serial.println("Sensor 1 Live Calibration Done!");

  delay(200);
}

void livefrontcalibration() {
  tcaSelect(1);  // TCS34725 on channel 0 back colour sensor
  if (!tcs.begin()) {
    Serial.println("Sensor front NOT found!");

  } else {
    Serial.println("Sensor front Detected");
    delay(200);
  }

  float sumR = 0, sumG = 0, sumB = 0;
  uint16_t r, g, b, c;
  Serial.println("Measuring...");

  for (int i = 0; i < 20; i++) {
    tcs.getRawData(&r, &g, &b, &c);
    if (c == 0) c = 1;
    sumR += (float)r / c * 256.0;
    sumG += (float)g / c * 256.0;
    sumB += (float)b / c * 256.0;
    delay(200);
  }

  cal2_R = sumR / 20.0;
  cal2_G = sumG / 20.0;
  cal2_B = sumB / 20.0;

  Serial.print("front Calibration Values:\n");
  Serial.print("R: ");
  Serial.print(cal2_R);
  Serial.print("  G: ");
  Serial.print(cal2_G);
  Serial.print("  B: ");
  Serial.println(cal2_B);
  delay(3000);

  Serial.println("Sensor 1 Live Calibration Done!");

  delay(200);
}

void loadcalibrationdownsensor() {
  tcaSelect(2);
  if (!tcs.begin()) {
    Serial.println("Sensor down NOT found!");

  } else {
    Serial.println("Sensor down Detected");
    delay(200);
  }

  if (EEPROM.read(EEPROM_FLAG2) == 0xBB) {
    EEPROM.get(EEPROM_R2, cal3_R);
    EEPROM.get(EEPROM_G2, cal3_G);
    EEPROM.get(EEPROM_B2, cal3_B);

    Serial.println("Loaded Sensor 2 Calibration from EEPROM:");
    Serial.print("R=");
    Serial.println(cal3_R);
    Serial.print("G=");
    Serial.println(cal3_G);
    Serial.print("B=");
    Serial.println(cal3_B);
    delay(3000);
  } else {
    Serial.println("EEPROM calibration missing for Sensor 2!");
    cal3_R = cal3_G = cal3_B = 1;
  }
}

void init_colorsens() {
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);

  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(led3, HIGH);
}

//-----------------------

void initDisplay() {
  pinMode(segA, OUTPUT);
  pinMode(segB, OUTPUT);
  pinMode(segC, OUTPUT);
  pinMode(segD, OUTPUT);
  pinMode(segE, OUTPUT);
  pinMode(segF, OUTPUT);
  pinMode(segG, OUTPUT);

  // Turn everything OFF initially (HIGH is OFF for Common Anode)
  clearDisplay();
}

//----------------------
//initializing ping sensors
void init_ping() {
  pinMode(trigger, OUTPUT);
  pinMode(echoL, INPUT);
  pinMode(echoR, INPUT);
  pinMode(echoF, INPUT);
}


void setup() {
  init_IR();
  init_motors();
  init_ping();
  initDisplay();
  init_colorsens();
  calibrateBot();
  Serial.begin(115200);

  //for servos
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  // Move to a safe start position
  setServoAngle(SERVO1_CHANNEL, 40);
  setServoAngle(SERVO2_CHANNEL, 68);
  servo1_angle = 40;
  servo2_angle = 68;

  release();
  delay(1000);
  //for colour sensors
  Wire.begin();
  livebackcalibration();

  displayValue('0');
  delay(3000);
  displayValue('1');
  livefrontcalibration();
  displayValue('2');
  loadcalibrationdownsensor();
  clearDisplay();

}


void loop() {
  task1();
}


// =============================================================
// PID LINE FOLLOWING
// =============================================================
void read_line() {
  unsigned long currentTime_line = millis();
  dt_line = currentTime_line - prevTime_line;
  prevTime_line = currentTime_line;

  uint16_t pos_line = qtr.readLineBlack(sensorValues);
  int err_line = map(pos_line, 0, 7000, -1000, 1000);

  int baseSpeed = 20;
  int diff = (err_line * kp_line) + ((err_line - last_value_line) * kd_line / dt_line);
  last_value_line = err_line;

  drive(baseSpeed - diff, baseSpeed + diff);
  delay(2);
}

// =============================================================
// CHECK FOR JUNCTION TYPE
// =============================================================
juncType junction() {
  dirType dir = frontDir(leftIR, rightIR, 100);
  bool front = isFrontBlack();

  uint16_t p = qtr.readLineBlack(sensorValues);
  int e = map(p, 0, 7000, 1000, -1000);

  if (!(dir == NONE_DIR) && abs(e) < 600) {
    if (dir == BOTH) {
      GoDistance(12);
      front = isFrontBlack();
      if (front) {
        return CROSS;
      }
      return T_JUNC;
    }

    if (dir == LEFT) {
      GoDistance(8);
      front = isFrontBlack();
      if (front) {
        return LEFT_BRANCH;
      }
      return LEFT_BEND;
    }

    if (dir == RIGHT) {
      GoDistance(8);
      front = isFrontBlack();
      if (front) {
        return RIGHT_BRANCH;
      }
      return RIGHT_BEND;
    }

  } else if (dir == NONE_DIR) {
    front = isFrontBlack();
    if (!front) {
      return DEAD_END;
    } else {
      return NONE;
    }
  } else {
    return NONE;
  }
}


// =============================================================
// INITIALIZATION FOR IR SENSORS
// =============================================================
void init_IR() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5, A6, A7 }, SensorCount);

  pinMode(leftIR, INPUT);
  pinMode(rightIR, INPUT);


  delay(500);
}


// =============================================================
// CALIBRATION OF SENSORS
// =============================================================
void calibrateBot() {
  drive(80, -80);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++) qtr.calibrate();
  digitalWrite(LED_BUILTIN, LOW);
  adjustCalibration();
  drive(0, 0);
}


// =============================================================
// ADJUST THRESHOLD ERROR OF THE QTR LIBRARY
// =============================================================
void adjustCalibration() {
  for (uint8_t i = 0; i < SensorCount; i++) {

    qtr.calibrationOn.minimum[i] += 1;
    qtr.calibrationOn.maximum[i] -= 1;

    // Safety limit: do not go below 0
    if ((int)qtr.calibrationOn.minimum[i] < 0)
      qtr.calibrationOn.minimum[i] = 0;

    if ((int)qtr.calibrationOn.maximum[i] < 0)
      qtr.calibrationOn.maximum[i] = 0;
  }
}


// =============================================================
// READ STABLE FRONT IR SENSOR OUTPUTS
// =============================================================
dirType frontDir(int leftPin, int rightPin, int requiredTime) {
  bool left = digitalRead(leftPin);
  bool right = digitalRead(rightPin);

  if ((left == HIGH && right == LOW) || (left == LOW && right == HIGH) || (left == HIGH && right == HIGH)) {
    if (left == HIGH && right == HIGH) {
      return BOTH;
    }

    if (left == HIGH && right == LOW) {
      unsigned long start = millis();
      while (millis() - start < requiredTime) {
        bool left = digitalRead(leftPin);
        bool right = digitalRead(rightPin);

        if (right == HIGH) {
          return BOTH;
        }
      }
      return LEFT;
    }

    if (left == LOW && right == HIGH) {
      unsigned long start = millis();
      while (millis() - start < requiredTime) {
        bool left = digitalRead(leftPin);
        bool right = digitalRead(rightPin);

        if (left == HIGH) {
          return BOTH;
        }
      }
      return RIGHT;
    }
  } else {
    return NONE_DIR;
  }
}


// =============================================================
// CHECK FOR FRONT LINE
// =============================================================
bool isFrontBlack() {
  qtr.readCalibrated(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > 200) return true;
  }
  return false;
}


// =============================================================
// INITIALIZATION FOR MOTORS
// =============================================================
void init_motors() {
  motorA.IN1 = mr1;
  motorA.IN2 = mr2;
  motorA.EN = enr;
  motorA.encA = mrEncA;
  motorA.encB = mrEncB;

  motorB.IN1 = ml2;
  motorB.IN2 = ml1;
  motorB.EN = enl;
  motorB.encA = mlEncA;
  motorB.encB = mlEncB;

  pinMode(mr1, OUTPUT);
  pinMode(mr2, OUTPUT);
  pinMode(ml1, OUTPUT);
  pinMode(ml2, OUTPUT);
  pinMode(enl, OUTPUT);
  pinMode(enr, OUTPUT);

  pinMode(motorA.encA, INPUT_PULLUP);
  pinMode(motorA.encB, INPUT_PULLUP);
  pinMode(motorB.encA, INPUT_PULLUP);
  pinMode(motorB.encB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(motorA.encA), updateMotorA_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorA.encB), updateMotorA_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorB.encA), updateMotorB_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorB.encB), updateMotorB_B, CHANGE);
}


// =============================================================
// ENCODER ISR FUNCTIONS
// =============================================================
void updateMotorA_A() {
  int A = digitalRead(motorA.encA);
  int B = digitalRead(motorA.encB);
  motorA.encoderPos += (A == B) ? 1 : -1;
}
void updateMotorA_B() {
  int A = digitalRead(motorA.encA);
  int B = digitalRead(motorA.encB);
  motorA.encoderPos += (A != B) ? 1 : -1;
}
void updateMotorB_A() {
  int A = digitalRead(motorB.encA);
  int B = digitalRead(motorB.encB);
  motorB.encoderPos += (A == B) ? 1 : -1;
}
void updateMotorB_B() {
  int A = digitalRead(motorB.encA);
  int B = digitalRead(motorB.encB);
  motorB.encoderPos += (A != B) ? 1 : -1;
}

// =============================================================
// SINGLE MOTOR PID FUNCTION
// =============================================================
void runMotor(Motor &M) {

  unsigned long now = millis();
  float dt = (now - M.lastTime) / 1000.0;
  M.lastTime = now;

  float motorDeg = M.encoderPos * anglePerCount;
  float error = M.targetDeg - motorDeg;

  // integral with windup protection
  if (abs(error) < 80) {
    M.integral += error * dt;
  }
  M.integral = constrain(M.integral, -150, 150);

  float derivative = (error - M.lastError) / dt;
  M.lastError = error;

  float control = Kp * error + Ki * M.integral + Kd * derivative;

  bool dir = (control >= 0);
  int pwmVal = min(abs(control), 255);
  int PWM = map(pwmVal, 0, 255, 155, 160);

  if (abs(error) < 3) PWM = 0;

  if (PWM == 0) {
    digitalWrite(M.IN1, LOW);
    digitalWrite(M.IN2, LOW);
    analogWrite(M.EN, 0);
  } else if (dir) {
    digitalWrite(M.IN1, HIGH);
    digitalWrite(M.IN2, LOW);
    analogWrite(M.EN, PWM);
  } else {
    digitalWrite(M.IN1, LOW);
    digitalWrite(M.IN2, HIGH);
    analogWrite(M.EN, PWM);
  }
}


// =============================================================
// ROTATE BOT IN A REQUIRED ANGLE
// =============================================================
void rotateMotors(float BotTurnDeg) {

  float WheelTurnDeg = BotTurnDeg * 2 * (22.0 / 7.0) * 9.5 / 21;
  // set targets
  motorA.targetDeg = WheelTurnDeg;
  motorB.targetDeg = WheelTurnDeg;

  // reset encoder tracking
  motorA.encoderPos = 0;
  motorB.encoderPos = 0;
  motorA.lastTime = millis();
  motorB.lastTime = millis();

  // Run both motors until they reach target
  while (true) {

    runMotor(motorA);
    runMotor(motorB);

    bool A_done = abs(motorA.targetDeg - motorA.encoderPos * anglePerCount) < 3;
    bool B_done = abs(motorB.targetDeg - motorB.encoderPos * anglePerCount) < 3;

    if (A_done && B_done) break;

    delay(10);
  }

  // Stop motors
  digitalWrite(motorA.IN1, LOW);
  digitalWrite(motorA.IN2, LOW);
  analogWrite(motorA.EN, 0);

  digitalWrite(motorB.IN1, LOW);
  digitalWrite(motorB.IN2, LOW);
  analogWrite(motorB.EN, 0);
}


// =============================================================
// MOVE BOT FOR A GIVEN DISTANCE
// =============================================================
void GoDistance(float BotGoDis) {

  float WheelGoDeg = BotGoDis * 360 / 20;
  // set targets
  motorA.targetDeg = -WheelGoDeg;
  motorB.targetDeg = WheelGoDeg;

  // reset encoder tracking
  motorA.encoderPos = 0;
  motorB.encoderPos = 0;
  motorA.lastTime = millis();
  motorB.lastTime = millis();

  // Run both motors until they reach target
  while (true) {

    runMotor(motorA);
    runMotor(motorB);

    bool A_done = abs(motorA.targetDeg - motorA.encoderPos * anglePerCount) < 3;
    bool B_done = abs(motorB.targetDeg - motorB.encoderPos * anglePerCount) < 3;

    if (A_done && B_done) break;

    delay(10);
  }

  // Stop motors
  digitalWrite(motorA.IN1, LOW);
  digitalWrite(motorA.IN2, LOW);
  analogWrite(motorA.EN, 0);

  digitalWrite(motorB.IN1, LOW);
  digitalWrite(motorB.IN2, LOW);
  analogWrite(motorB.EN, 0);
}


// =============================================================
// SINGLE MOTOR (SUPPORT) DRIVE FUNCTION
// =============================================================
void driveMotor(int speed, int in1, int in2, int en, int minPWM, int maxPWM) {
  if (speed == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(en, 0);
    return;
  }

  speed = constrain(speed, -255, 255);
  int pwm = map(abs(speed), 1, 255, minPWM, maxPWM);

  if (speed > 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  analogWrite(en, pwm);
}


// =============================================================
// MAIN DRIVE FUNCTION FOR LINE FOLLOWING
// =============================================================
void drive(int s1, int s2) {
  driveMotor(s1, ml1, ml2, enl, 150, 210);
  driveMotor(s2, mr1, mr2, enr, 150, 210);
}

bool isFrontBlackSquare() {
  qtr.readCalibrated(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] < 200) return false;
  }
  return true;
}

// --- Servo Movement Functions ---
// --- Helper Functions ---

// Set servo angle (0–180) using PCA9685
void setServoAngle(uint8_t channel, int angle) {
  // Clamp angle to 0–180
  angle = constrain(angle, 0, 180);
  // Map 0–180 to SERVOMIN–SERVOMAX
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}

// Move a servo smoothly from current angle to target
void moveServoToAngle(uint8_t channel, int targetAngle, int delayMs) {
  int currentAngle = (channel == SERVO1_CHANNEL) ? servo1_angle : servo2_angle;

  // Use bigger steps for smoother motion under load
  int step = (targetAngle > currentAngle) ? 2 : -2;

  for (int angle = currentAngle;
       (step > 0 ? angle <= targetAngle : angle >= targetAngle);
       angle += step) {

    setServoAngle(channel, angle);

    if (channel == SERVO1_CHANNEL) {
      servo1_angle = angle;
    } else if (channel == SERVO2_CHANNEL) {
      servo2_angle = angle;
    }

    delay(delayMs);  // for arm try 10–20 ms instead of 50
  }

  // Ensure exact final angle
  setServoAngle(channel, targetAngle);
  if (channel == SERVO1_CHANNEL) servo1_angle = targetAngle;
  else if (channel == SERVO2_CHANNEL) servo2_angle = targetAngle;
}


// --- Arm Pose Functions ---

void poseUpStraight() {
  moveServoToAngle(SERVO2_CHANNEL, 120, 30);
}

void poseUpStraight2() {
  moveServoToAngle(SERVO2_CHANNEL, 90, 30);
}

void poseUpStorage() {
  moveServoToAngle(SERVO2_CHANNEL, 65, 5);
}

void poseDownBox() {
  moveServoToAngle(SERVO2_CHANNEL, 180, 30);
}

void poseDownBall() {
  moveServoToAngle(SERVO2_CHANNEL, 165, 30);
}

// --- Gripper Functions ---

void grab() {
  moveServoToAngle(SERVO1_CHANNEL, 110, 20);
}

void grabBall() {
  moveServoToAngle(SERVO1_CHANNEL, 110, 20);
}

void release() {
  moveServoToAngle(SERVO1_CHANNEL, 30, 60);
}

// --- Task Sequences ---

void grab_first_box() {
  delay(1000);
  release();
  delay(100);
  poseDownBox();
  delay(1000);
  grab();
  delay(100);
  //detecting the colour
  FirstBoxColour = detectColor2();
  box1try += 1;
  grab();
  delay(1000);
  poseUpStorage();
  delay(1000);

  release();
  delay(1000);
}

void grab_second_box() {
  delay(1000);
  release();
  delay(1000);
  poseUpStraight();
  delay(1000);
  poseDownBox();
  delay(1000);
  grab();
  delay(100);
  //detecting the colour
  FirstBoxColour = detectColor2();
  box1try += 1;
  grab();
  delay(1000);
  poseUpStraight2();
  delay(1000);
  //detecting the colour
  SecondBoxColour = detectColor2();
  box2try += 1;
  delay(1000);
}

void grab_ball() {
  delay(1000);
  release();
  delay(100);
  poseDownBall();
  delay(1000);
  grabBall();
  delay(100);
  poseUpStorage();
  delay(1000);
  release();
  delay(1000);
}

void place_first_box() {
  delay(1000);
  release();
  delay(1000);
  poseUpStorage();
  delay(1000);
  grab();
  delay(30);
  delay(1000);
  poseDownBox();
  delay(1000);
  release();
  delay(1000);
  //for testing
  poseUpStorage();
  delay(1000);
}

void place_second_box() {
  delay(1000);
  grab();
  delay(30);
  grab();
  delay(1000);
  poseDownBox();
  delay(1000);
  release();
  delay(1000);
  //for testing
  poseUpStraight2();
  delay(1000);
}

float readUltrasonic(int echoPin) {

  // Send trigger pulse (common trigger)
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);  // timeout for safety
  float dist = duration * 0.034 / 2.0;

  if (dist == 0 || dist > 50) dist = 50;

  return dist;
}

void CheckForBox() {
  float obs = readUltrasonic(echoF);

  if (obs <= 7) {
    if (FirstBox) {
      GoDistance(2);
      delay(300);
      obs = readUltrasonic(echoF);
      GoDistance(obs - armLength);
      delay(200);
      grab_second_box();
      if ((SecondBoxColour != "UNKNOWN") || (box2try > 2)) {
        SecondBox = true;
      }
      GoDistance(armLength - obs);
      //for testing
      if (SecondBoxColour == "RED") {
        clearDisplay();
        displayValue('2');
        delay(3000);
        clearDisplay();
      } else if (SecondBoxColour == "BLUE") {
        clearDisplay();
        displayValue('3');
        delay(3000);
        clearDisplay();
      } else{
        clearDisplay();
        displayValue('0');
        delay(3000);
        clearDisplay();        
      }

    } else {
      GoDistance(2);
      delay(300);
      obs = readUltrasonic(echoF);
      GoDistance(obs - armLength);
      delay(200);
      grab_first_box();
      if ((FirstBoxColour != "UNKNOWN") || (box1try > 2)) {
        FirstBox = true;
      }
      GoDistance(armLength - obs);
      //for testing
      if (FirstBoxColour == "RED") {
        clearDisplay();
        displayValue('r');
        delay(3000);
        clearDisplay();
      } else if (FirstBoxColour == "BLUE") {
        clearDisplay();
        displayValue('g');
        delay(3000);
        clearDisplay();
      } else{
        clearDisplay();
        displayValue('0');
        delay(3000);
        clearDisplay();        
      }
    }
  }
}







// ---------------------- SENSOR 1 COLOR DETECTION ----------------------
String detectColor1() {  //backsensor
  tcaSelect(0);
  tcs.begin();
  float red, green, blue;

  uint16_t r, g, b, c;
  for (int i = 0; i < 4; i++) {
    tcs.getRawData(&r, &g, &b, &c);
    delay(100);

    if (c == 0) c = 1;

    red = (((float)r / c) * 256.0 / cal1_R) * 1000;
    green = (((float)g / c) * 256.0 / cal1_G) * 1000;
    blue = (((float)b / c) * 256.0 / cal1_B) * 1000;
    delay(500);
  }

  Serial.print("R: ");
  Serial.print((int)red);
  Serial.print("  G: ");
  Serial.print((int)green);
  Serial.print("  B: ");
  Serial.println((int)blue);


  if (red > green && red > blue) {
    return "RED";
  }

  if (green > red && green > blue) {
    return "GREEN";
  }

  if (blue > red && blue > green) {
    return "BLUE";
  }

  return "UNKNOWN";
}


// ---------------------- SENSOR 2 COLOR DETECTION ----------------------
String detectColor2() {  //front sensor
  tcaSelect(1);
  tcs.begin();
  float red, green, blue;
  uint16_t r, g, b, c;

  for (int i = 0; i < 4; i++) {
    tcs.getRawData(&r, &g, &b, &c);
    delay(100);

    if (c == 0) c = 1;

    // Normalize using clear channel, apply calibration
    red = (((float)r / c) * 256.0 / cal2_R) * 1000;
    green = (((float)g / c) * 256.0 / cal2_G) * 1000;
    blue = (((float)b / c) * 256.0 / cal2_B) * 1000;
    delay(500);
  }

  Serial.print("R: ");
  Serial.print((int)red);
  Serial.print("  G: ");
  Serial.print((int)green);
  Serial.print("  B: ");
  Serial.println((int)blue);

  if ((red > green && red > blue)) {
    return "RED";
  }

  if ((green > red && green > blue)) {
    return "GREEN";
  }

  if ((blue > red && blue > green)) {
    return "BLUE";
  }

  return "UNKNOWN";
}

String detectColor3() {  //down sensor
  tcaSelect(2);
  tcs.begin();
  float red, green, blue;  // Activate sensor 2

  uint16_t r, g, b, c;
  for (int i = 0; i < 4; i++) {
    tcs.getRawData(&r, &g, &b, &c);
    delay(100);


    if (c == 0) c = 1;

    // Normalize using clear channel, apply calibration
    red = (((float)r / c) * 256.0 / cal3_R) * 1000;
    green = (((float)g / c) * 256.0 / cal3_G) * 1000;
    blue = (((float)b / c) * 256.0 / cal3_B) * 1000;
    delay(500);
  }

  Serial.print("R: ");
  Serial.print((int)red);
  Serial.print("  G: ");
  Serial.print((int)green);
  Serial.print("  B: ");
  Serial.println((int)blue);

  if (red > green && red > blue) return "RED";
  if (green > red && green > blue) return "GREEN";
  if (blue > red && blue > green) return "BLUE";

  return "UNKNOWN";
}

//-----------------------------

void displayValue(char x) {
  if (x == 'r') {
    displaySegment(0, 1, 1, 1, 0, 0, 1);
  } else if (x == 'g') {
    displaySegment(LOW, HIGH, LOW, LOW, LOW, LOW, HIGH);
  } else if (x == '0') {
    displaySegment(LOW, LOW, LOW, LOW, LOW, LOW, HIGH);
  } else if (x == '1') {
    displaySegment(HIGH, LOW, LOW, HIGH, HIGH, HIGH, HIGH);
  } else if (x == '2') {
    displaySegment(LOW, LOW, HIGH, LOW, LOW, HIGH, LOW);
  } else if (x == '3') {
    displaySegment(LOW, LOW, LOW, LOW, HIGH, HIGH, LOW);
  }
}

// Helper function to set all 7 segments at once
// Remember: LOW = ON, HIGH = OFF
void displaySegment(int a, int b, int c, int d, int e, int f, int g) {
  digitalWrite(segA, a);
  digitalWrite(segB, b);
  digitalWrite(segC, c);
  digitalWrite(segD, d);
  digitalWrite(segE, e);
  digitalWrite(segF, f);
  digitalWrite(segG, g);
}

void clearDisplay() {
  displaySegment(HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH);
}

void task1() {
  juncType junc = junction();

  if (junc == DEAD_END) {
    GoDistance(5);
    if (isFrontBlack()) {

      if (FirstBox && SecondBox) {
        delay(500);
        rotateMotors(180);
        // hard coding for putting the boxes
        bool DonePlacing = false;
        while (!DonePlacing) {
          junc = junction();
          if (junc == DEAD_END) {
            GoDistance(5);
            bool lineDetected = isFrontBlack();
            //most likely me part ek wenne na
            if (lineDetected && task1done) {
              DonePlacing = false;
              // start of task 2
            } else {
              if (SecondBox) {
                delay(100);
                GoDistance(-(armLength + 10));
                place_second_box();
                GoDistance(armLength);
                SecondBox = false;
              } else {
                delay(100);
                GoDistance(-(armLength + 10));
                place_first_box();
                FirstBox = false;
                task1done = true;
                GoDistance(armLength);
                DonePlacing = true;
              }
              delay(500);
              rotateMotors(180);
            }
          } else if (junc == T_JUNC) {
            if (DoneCross) {
              if (SecondBoxColour == "RED") {
                SecondBlue = true;
                delay(500);
                rotateMotors(-90);
              } else {
                delay(500);
                rotateMotors(90);
              }
            } else {
              delay(500);
              rotateMotors(90);
            }
          } else if (junc == CROSS) {
            delay(500);
            rotateMotors(-90);
            DoneCross = true;
          } else if (junc == LEFT_BEND) {
            delay(500);
            rotateMotors(-90);
          } else if (junc == LEFT_BRANCH) {
            if (DoneCross) {
              read_line();
            } else {
              delay(500);
              rotateMotors(-90);
              DoneCross = true;
            }
          } else if (junc == RIGHT_BRANCH) {
            if (DoneCross) {
              read_line();
            } else {
              delay(500);
              rotateMotors(90);
            }
          } else {
            read_line();
          }
        }
      } else if (task1done) {
        delay(500);
        // add the part what to do after finishing task 1
        task2();
      } else {
        GoBack = true;
        delay(500);
        rotateMotors(180);
      }
    } else {
      delay(500);
      rotateMotors(180);
    }

  } else if (junc == T_JUNC || junc == RIGHT_BRANCH || junc == RIGHT_BEND) {
    delay(500);
    rotateMotors(90);

  } else if (junc == CROSS) {
    bool isSquare = isFrontBlackSquare();
    if (isSquare == true) {
      delay(500);
      rotateMotors(180);
    } else if (task1done) {
      delay(500);
      rotateMotors(-90);
    } else {
      delay(500);
      rotateMotors(90);
    }

  } else if (junc == LEFT_BRANCH) {
    if (GoBack) {
      delay(500);
      rotateMotors(75);
      GoBack = false;
    } else if (SecondBlue) {
      delay(500);
      rotateMotors(-90);
      SecondBlue = false;
    } else {
      read_line();
    }

  } else if (junc == LEFT_BEND) {
    delay(500);
    rotateMotors(-90);

  } else {
    read_line();
  }

  if (!((task1done) || (FirstBox && SecondBox))) {
    CheckForBox();
  }
}

//new functions for task 2


void CheckForTower() {
  float tower = readUltrasonic(echoF);

  if (tower <= 12) {
    GoDistance(2);
    delay(500);
    rotateMotors(180);
    if (isFrontBlack()==true){
      for (int i = 0; i < 5; i++){
        read_line();
      }
    }
    GoDistance(2);
    delay(500);
    GoDistance((-8-(tower)));
    delay(500);
    String TColour = detectColor3();
    TowerColours[TowerCount] = TColour;
    TowerCount += 1;
  }
}

void displayColorArray(const String colors[], int size, int delayTime = 2000) {
  //for testing
  displayValue('1');
  delay(2000);
  for (int i = 0; i < size; i++) {
    if (colors[i] == "RED") {
      displayValue('r');   // your existing function
    }
    else if (colors[i] == "GREEN") {
      displayValue('g');   // your existing function
    }
    else if (colors[i] == "BLUE") {
      displayValue('2');
    }else{
      displayValue('0');      // unknown value -> blank
    }

    delay(delayTime);
    clearDisplay();
    delay(1000);
  }
}

void task2(){
  juncType junc = junction();

  if (junc == DEAD_END) {
    GoDistance(5);
    bool lineDetected = isFrontBlack();
    //need to fix this part
    if ((!lineDetected)) {
      delay(500);
      rotateMotors(180);
    }
  } else if (junc == T_JUNC) {
    if (RightTurned) {
      delay(500);
      rotateMotors(90);
    } else {
      delay(500);
      rotateMotors(-90);
    }
  } else if (junc == RIGHT_BRANCH || junc == RIGHT_BEND) {
    GoDistance(5);
    delay(500);
    rotateMotors(90);
    RightTurned = true;
  } else if (junc == LEFT_BEND || junc == LEFT_BRANCH) {
    GoDistance(5);
    delay(500);
    rotateMotors(-90);
    RightTurned = false;
  } else if (isFrontBlackSquare()){
    GoDistance(2);
    displayColorArray(TowerColours,5,2000);
    delay(2000);
    //task2 done from here
    
  } else {
    read_line();
  }
  CheckForTower();
}