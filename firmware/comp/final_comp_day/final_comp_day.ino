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

#define dip1 22
#define dip2 41
#define dip3 43
#define dip4 45
#define dip5 47
#define dip6 49
#define dip7 51
#define dip8 53

int buttonCount = 0;



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

//variables to check if the tasks are done
bool TASK1 = false;
bool TASK2 = false;
bool TASK3 = false;
bool TASK4 = false;
bool TASK5 = false;

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
bool RightTurned = false;
String TowerColours[3] = { "GREEN", "GREEN", "GREEN" };
int TowerCount = 0;


//Variable defining for task 3
float kpWall = 0.25;
float kdWall = 20.0;

unsigned long prevTime_Wall = 0;
float lastErr_Wall = 0;

float disBetweenSonar_Wall = 20.0;

enum juncType_Wall { STRAIGHT_Wall,
                     DEAD_END_Wall,
                     RIGHT_BEND_Wall,
                     RIGHT_BRANCH_Wall,
                     LEFT_BEND_Wall,
                     LEFT_BRANCH_Wall,
                     T_JUNC_Wall };

bool leftTurned_Wall = false;
bool rightTurned_Wall = false;
String ColorBall = "UNKNOWN";
int ballcount = 0;
String BallColors[5] = { "GREEN", "GREEN", "GREEN", "GREEN", "GREEN" };

//Variable defining for task 4
String colourstsk1[3] = { "GREEN", "GREEN", "RED" };
String colourstsk2[5] = { "GREEN", "GREEN", "GREEN", "GREEN", "RED" };
String colours[5] = colourstsk2[5];
//-----------------------------------------------

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
int servo1_angle = 40;  // Initial angle for gripper
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


//-------------task4----------

// --- Define PCA9685 Channels ---
#define SERVO3_CHANNEL 14  // lid 1
#define SERVO4_CHANNEL 15  // lid 2
#define SERVO5_CHANNEL 12  // catapult

// --- Current Angle Tracking Variables ---
int servo3_angle = 132;  // Initial angle for lid 1
int servo4_angle = 110;  // Initial angle for lid 2
int servo5_angle = 50;

int num_from_tsk_2;
int num_from_tsk_3;

// --- ADDED GLOBAL VARIABLES ---
int globalQuotient = 0;
int globalRemainder = 0;

void setCatapultAngle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}

// Moves a servo smoothly from its current tracked angle to a target angle.
void moveCatapultToAngle(uint8_t channel, int targetAngle, int delayMs) {
  int* currentAnglePtr = nullptr;

  if (channel == SERVO3_CHANNEL) {
    currentAnglePtr = &servo3_angle;
  } else if (channel == SERVO4_CHANNEL) {
    currentAnglePtr = &servo4_angle;
  } else if (channel == SERVO5_CHANNEL) {
    currentAnglePtr = &servo5_angle;
  } else {
    return;
  }

  int currentAngle = *currentAnglePtr;
  int step = (targetAngle > currentAngle) ? 1 : -1;

  for (int angle = currentAngle; angle != targetAngle; angle += step) {
    setCatapultAngle(channel, angle);
    *currentAnglePtr = angle;
    delay(delayMs);
  }

  setCatapultAngle(channel, targetAngle);
  *currentAnglePtr = targetAngle;
}

void lid_1_open_close() {
  int openAngle = 132;
  int closeAngle = 165;
  delay(1000);
  moveCatapultToAngle(SERVO3_CHANNEL, openAngle, 10);
  delay(2000);
  moveCatapultToAngle(SERVO3_CHANNEL, closeAngle, 10);
  delay(1000);
}

void lid_2_open_close() {
  int openAngle = 50;
  int closeAngle = 110;

  delay(1000);
  moveCatapultToAngle(SERVO4_CHANNEL, openAngle, 10);
  delay(2000);
  moveCatapultToAngle(SERVO4_CHANNEL, closeAngle, 10);
  delay(1000);
}

void catapult_fire_reset() {
  int fireAngle = 135;
  int resetAngle = 50;

  moveCatapultToAngle(SERVO5_CHANNEL, fireAngle, 0);
  delay(1000);
  moveCatapultToAngle(SERVO5_CHANNEL, resetAngle, 20);
}

void ball_throw() {
  grab();
  lid_1_open_close();
  delay(1000);
  lid_2_open_close();
  delay(1000);
  catapult_fire_reset();
  delay(1000);
}

char mapNumberToLetter(int value) {
  switch (value) {
    case 0: return 'A';
    case 1: return 'B';
    case 2: return 'C';
    case 3: return 'D';
    default: return '?';  // Invalid value
  }
}
char storage;
void throw_storage() {
  for (int i = 0; i < 5; i++) {

    if (colours[i] == "GREEN") {
      storage = mapNumberToLetter(globalQuotient);
      if (storage == 'A') {
        rotateMotors(-45);
        ball_throw();
        rotateMotors(45);
      } else if (storage == 'B') {
        rotateMotors(45);
        ball_throw();
        rotateMotors(-45);
      } else if (storage == 'C') {
        rotateMotors(-135);
        ball_throw();
        rotateMotors(135);
      } else if (storage == 'D') {
        rotateMotors(135);
        ball_throw();
        rotateMotors(-135);
      }
    } else if (colours[i] == "RED") {
      storage = mapNumberToLetter(globalRemainder);
      if (storage == 'A') {
        rotateMotors(-45);
        ball_throw();
        rotateMotors(45);
      } else if (storage == 'B') {
        rotateMotors(45);
        ball_throw();
        rotateMotors(-45);
      } else if (storage == 'C') {
        rotateMotors(-135);
        ball_throw();
        rotateMotors(135);
      } else if (storage == 'D') {
        rotateMotors(135);
        ball_throw();
        rotateMotors(-135);
      }
    }
  }
}


void init_dip() {
  pinMode(dip1, INPUT_PULLUP);
  pinMode(dip2, INPUT_PULLUP);
  pinMode(dip3, INPUT_PULLUP);
  pinMode(dip4, INPUT_PULLUP);
  pinMode(dip6, INPUT_PULLUP);
}

void setup() {
  init_dip();
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

  //for task 4
  setCatapultAngle(SERVO3_CHANNEL, servo3_angle);
  setCatapultAngle(SERVO4_CHANNEL, servo4_angle);
  setCatapultAngle(SERVO5_CHANNEL, servo5_angle);

  num_from_tsk_2 = convertColourArray(colourstsk1, 3);
  num_from_tsk_3 = countGreenToBinaryInt(colourstsk2, 5);

  task4cal();


  poseDownBox();
  delay(500);
  grab();
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
  poseUpStorage();
  delay(10000);
  GoDistance(10);
}

void loop() {
  // bool val1 = digitalRead(dip1) == 0;
  // bool val2 = digitalRead(dip1) == 0;
  // bool val3 = digitalRead(dip1) == 0;
  // bool val4 = digitalRead(dip1) == 0;
  // bool val5 = digitalRead(dip6) == 0;

  // if (val1) {
  //   TASK1 = true;
  // }

  // if (val2) {
  //   TASK1 = true;
  //   TASK2 = true;
  // }

  // if (val3) {
  //   TASK1 = true;
  //   TASK2 = true;
  //   TASK3 = true;
  // }

  // if (val4) {
  //   TASK1 = true;
  //   TASK2 = true;
  //   TASK3 = true;
  //   TASK4 = true;
  // }

  // if (val5) {
  //   bool TASK1 = false;
  //   bool TASK2 = false;
  //   bool TASK3 = false;
  //   bool TASK4 = false;
  //   bool TASK5 = false;
  // }

  if (!TASK1) {
    task1();
  } else if (TASK1 && !TASK2) {
    task2();
  } else if (TASK1 && TASK2 && !TASK3) {
    task3();
  } else if (TASK1 && TASK2 && TASK3 && !TASK4) {
    task4();
  } else if (TASK1 && TASK2 && TASK3 && TASK4 && !TASK5) {
    task5();
    rotateMotors(270);
    delay(1000);
  } else if (TASK1 && TASK2 && TASK3 && TASK4 && TASK5) {
    rotateMotors(360);
    delay(1000);
  } else {
    rotateMotors(720);
    delay(1000);
  }
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
void runMotor(Motor& M) {

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
  moveServoToAngle(SERVO2_CHANNEL, 90, 10);
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
  //for testing
  if (FirstBoxColour == "RED") {
    clearDisplay();
    displayValue('2');
    delay(3000);
    clearDisplay();
  } else if (FirstBoxColour == "BLUE") {
    clearDisplay();
    displayValue('3');
    delay(3000);
    clearDisplay();
  } else {
    clearDisplay();
    displayValue('0');
    delay(3000);
    clearDisplay();
  }
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
  poseDownBox();
  delay(1000);
  grab();
  delay(100);
  //detecting the colour
  SecondBoxColour = detectColor2();
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
  } else {
    clearDisplay();
    displayValue('0');
    delay(3000);
    clearDisplay();
  }
  grab();
  delay(1000);
  poseUpStraight2();
  delay(1000);
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
  ColorBall = detectColor3();
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
      GoDistance(obs - armLength + 1);
      delay(200);
      grab_second_box();
      if ((SecondBoxColour != "UNKNOWN") || (box2try > 2)) {
        SecondBox = true;
      }
      GoDistance(armLength - obs);

    } else {
      GoDistance(2);
      delay(300);
      obs = readUltrasonic(echoF);
      GoDistance(obs - armLength + 1);
      delay(200);
      grab_first_box();
      if ((FirstBoxColour != "UNKNOWN") || (box1try > 2)) {
        FirstBox = true;
      }
      GoDistance(armLength - obs);
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

  else {
    return "GREEN";
  }
}


// ---------------------- SENSOR 2 COLOR DETECTION ----------------------
String detectColor2() {  //front sensor for box
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

  if (red > blue) {
    return "RED";
  } else {
    return "BLUE";
  }
}

String detectColor3() {  //front sensor for ball
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

  if (red > green) {
    return "RED";
  } else {
    return "GREEN";
  }
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
                SecondBox = false;
              } else {
                delay(100);
                GoDistance(-(armLength + 10));
                place_first_box();
                FirstBox = false;
                task1done = true;
                DonePlacing = true;
              }
              delay(500);
              rotateMotors(180);
              GoDistance(-10);
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
        TASK1 = true;
        // add the part what to do after finishing task 1

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
    if (isFrontBlack() == true) {
      for (int i = 0; i < 5; i++) {
        read_line();
      }
    }
    GoDistance(2);
    delay(500);
    GoDistance((-8 - (tower)));
    delay(500);
    String TColour = detectColor1();
    if (TowerCount < 3) {
      TowerColours[TowerCount] = TColour;
    }
    TowerCount += 1;
  }
}

void displayColorArray(const String colors[], int size, int delayTime = 2000) {
  //for testing
  displayValue('1');
  delay(2000);
  for (int i = 0; i < size; i++) {
    if (colors[i] == "RED") {
      displayValue('2');  // your existing function
    } else {
      displayValue('3');  // your existing function
    }
    delay(delayTime);
    clearDisplay();
    delay(1000);
  }
}

void task2() {
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
  } else if (isFrontBlackSquare()) {
    GoDistance(2);
    displayColorArray(TowerColours, 5, 2000);
    delay(2000);
    GoDistance(5);
    TASK2 = true;
    //task2 done from here

  } else {
    read_line();
  }
  CheckForTower();
}

void readWall(float left, float right) {
  unsigned long now_Wall = millis();
  float dt_Wall = (now_Wall - prevTime_Wall);
  if (dt_Wall < 1) dt_Wall = 1;  // Prevent divide-by-zero

  prevTime_Wall = now_Wall;

  // Error: positive means robot too close to LEFT wall
  float pos_Wall = (left - right);
  float err_Wall = map(pos_Wall, -50, 50, -1000, 1000);

  // PD output
  float diff_Wall = kpWall * err_Wall + kdWall * (err_Wall - lastErr_Wall) / dt_Wall;
  lastErr_Wall = err_Wall;

  int baseSpeed_Wall = 20;  // You can adjust this

  int leftMotor_Wall = baseSpeed_Wall - diff_Wall;
  int rightMotor_Wall = baseSpeed_Wall + diff_Wall;

  drive(leftMotor_Wall, rightMotor_Wall);
}

juncType_Wall junction_Wall(float L, float R, float F) {
  if (L < 20.0 && R < 20.0) {
    if (F < 20.0) {
      return DEAD_END_Wall;
    }
    return STRAIGHT_Wall;
  } else if ((L > 20.0 && R < 20.0) || (L < 20.0 && R > 20.0) || (L > 20.0 && R > 20.0)) {
    GoDistance(21);
    delay(1000);
    L = readUltrasonic(echoL);
    delay(5);
    R = readUltrasonic(echoR);
    delay(5);
    F = readUltrasonic(echoF);
    delay(5);

    if (L > 20.0 && R < 20.0) {
      if (F < 30.0) {
        return LEFT_BEND_Wall;
      }
      return LEFT_BRANCH_Wall;
    } else if (L < 20.0 && R > 20.0) {
      if (F < 30.0) {
        return RIGHT_BEND_Wall;
      }
      return RIGHT_BRANCH_Wall;
    } else if (L > 20.0 && R > 20.0) {
      return T_JUNC_Wall;
    }
  }
}

void task3() {
  float distL = readUltrasonic(echoL);
  delay(5);
  float distR = readUltrasonic(echoR);
  delay(5);
  float distF = readUltrasonic(echoF);
  delay(5);
  if ((distL > 20) && (distR > 20) && (distF > 20)) {
    GoDistance(5);
    bool checkfinishedwall = isFrontBlackSquare();
    if (!checkfinishedwall) {
      TASK3 = true;
    }
  }
  juncType_Wall j_Wall = junction_Wall(distL, distR, distF);
  if (j_Wall == STRAIGHT_Wall) {
    readWall(distL, distR);
  } else if (j_Wall == DEAD_END_Wall) {
    CheckForBall();
    delay(1000);
    GoDistance(8);
    rotateMotors(180);
    GoDistance(10);
  } else if (j_Wall == LEFT_BEND_Wall || j_Wall == LEFT_BRANCH_Wall) {
    leftTurned_Wall = true;
    rotateMotors(-90);
    delay(1000);
    GoDistance(15);
  } else if (j_Wall == RIGHT_BEND_Wall || j_Wall == RIGHT_BRANCH_Wall) {
    rightTurned_Wall = true;
    rotateMotors(90);
    delay(1000);
    GoDistance(15);
  } else if (j_Wall == T_JUNC_Wall) {
    if (rightTurned_Wall) {
      rotateMotors(90);
      delay(1000);
      GoDistance(15);
      rightTurned_Wall = false;
    } else if (leftTurned_Wall) {
      rotateMotors(-90);
      delay(1000);
      GoDistance(15);
      leftTurned_Wall = false;
    }
  }
}


void CheckForBall() {
  GoDistance(2);
  float balldis = readUltrasonic(echoF);
  delay(500);
  GoDistance(balldis - armLength);
  grab_ball();
  if (ballcount < 5) {
    BallColors[ballcount] = ColorBall;
  }
  ballcount += 1;
}

int convertColourArray(String arr[], int len) {
  int value = 0;

  for (int i = 0; i < len; i++) {
    value *= 10;

    if (arr[i] == "RED") {
      value += 0;
    } else if (arr[i] == "GREEN") {
      value += 1;
    }
  }

  return value;
}

int countGreenToBinaryInt(String arr[], int len) {
  int greenCount = 0;

  for (int i = 0; i < len; i++) {
    if (arr[i] == "GREEN") {
      greenCount++;
    }
  }

  int binValue = 0;
  int place = 1;
  int val = greenCount;

  if (val == 0) return 0;

  while (val > 0) {
    int bit = val % 2;
    binValue += bit * place;
    place *= 10;
    val /= 2;
  }

  return binValue;
}

void compute_my_ass(int num1, int num2, int* quotient, int* remainder) {
  int matrix[2][3];

  matrix[0][0] = num1 / 100;
  matrix[0][1] = (num1 / 10) % 10;
  matrix[0][2] = num1 % 10;

  matrix[1][0] = num2 / 100;
  matrix[1][1] = (num2 / 10) % 10;
  matrix[1][2] = num2 % 10;

  int transpose[3][2];
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 3; j++) {
      transpose[j][i] = matrix[i][j];
    }
  }

  int result[2][2] = { 0 };
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      for (int k = 0; k < 3; k++) {
        result[i][j] += matrix[i][k] * transpose[k][j];
      }
    }
  }

  int sum = 0;
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      sum += result[i][j];
    }
  }

  *quotient = sum / 4;
  *remainder = sum % 4;
}

void get_box_to_shoot(int num1, int num2) {
  int number1 = num1;
  int number2 = num2;

  int quotient, remainder;
  compute_my_ass(number1, number2, &quotient, &remainder);

  // --- STORE RESULTS IN GLOBAL VARIABLES ---
  globalQuotient = quotient;
  globalRemainder = remainder;
}

void task4cal() {
  get_box_to_shoot(num_from_tsk_2, num_from_tsk_3);
}

void task4() {
  juncType junc = junction();

  if (junc == DEAD_END) {
    GoDistance(5);
    delay(500);
    rotateMotors(180);


  } else if (junc == T_JUNC || junc == RIGHT_BRANCH || junc == RIGHT_BEND) {
    delay(500);
    rotateMotors(90);

  } else if (junc == CROSS) {
    bool isSquare = isFrontBlackSquare();
    if (isSquare == true) {
      GoDistance(13);
      colours[5] = BallColors[5];
      colourstsk1[3] = TowerColours[3];
      colourstsk2[5] = BallColors[5];
      throw_storage();
      TASK4 = true;
      //add the task 4 throwing

    } else {
      delay(500);
      rotateMotors(90);
    }

  } else if (junc == LEFT_BRANCH) {
    read_line();
  }

  else if (junc == LEFT_BEND) {
    delay(500);
    rotateMotors(-90);
  } else {
    read_line();
  }
}

void task5(){
  juncType junc = junction();

  if (junc == DEAD_END) {
    GoDistance(5);
    delay(500);
    rotateMotors(180);


  } else if (junc == T_JUNC || junc == RIGHT_BRANCH || junc == RIGHT_BEND) {
    delay(500);
    rotateMotors(90);

  } else if (junc == CROSS) {
    bool isSquare = isFrontBlackSquare();
    if (isSquare == true) {
      TASK5 = true;
      //add the task 4 throwing

    } else {
      delay(500);
      rotateMotors(90);
    }

  } else if (junc == LEFT_BRANCH) {
    read_line();
  }

  else if (junc == LEFT_BEND) {
    delay(500);
    rotateMotors(-90);
  } else {
    read_line();
  }
}