#include <QTRSensors.h>

#define ir_rm 53
#define ir_r 51
#define ir_m 49
#define ir_l 47
#define ir_lm 45

#define leftIR 52
#define rightIR 50

#define mr1 6
#define mr2 7
#define ml1 5
#define ml2 4
#define enl 12
#define enr 11

#define mrEncA 3
#define mrEncB 2
#define mlEncA 19
#define mlEncB 18

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


#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create the PCA9685 driver object
// Default I2C address is 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- Servo Configuration ---
// These values are for a 50 Hz PWM frequency.
// You may need to tune these for
// your specific servos.
#define SERVOMIN 120   // Min pulse length count (out of 4096)
#define SERVOMAX 600   // Max pulse length count (out of 4096)
#define SERVO_FREQ 50  // Standard 50 Hz for analog servos

// Define which channel each servo is on
#define SERVO1_CHANNEL 4
#define SERVO2_CHANNEL 7

void setServoAngle(uint8_t channel, int angle) {
  // Map the 0-180 degree angle to the servo's pulse range
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);

  // Send the pulse command
  // The '0' means the pulse starts at tick 0
  pwm.setPWM(channel, 0, pulse);
}

// --- Variables for Way 1 (Sweep) ---
int servo1_angle = 50;
int servo2_angle = 120;  // Initial angle



// --- Servo Movement Functions ---
// --------------------------------------------------------
// HC-SR04 PIN SETUP
// --------------------------------------------------------
#define TRIG_PIN 8
#define ECHO_PIN 9

// --------------------------------------------------------
// INITIALIZE SENSOR
// Call once in setup(): InitUltrasonic();
// --------------------------------------------------------
void InitUltrasonic() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}


// --------------------------------------------------------
// READ HC-SR04 DISTANCE (cm)
// --------------------------------------------------------
float ReadDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // 30ms timeout
  if (duration == 0) return 999;                   // No reading → assume no object

  float distance = duration * 0.0343 / 2.0;
  return distance;
}

// --------------------------------------------------------
// MAIN FUNCTION YOU REQUESTED
// Detect box → stop → go back → grab box
// --------------------------------------------------------


void setup() {
  init_IR();
  init_motors();
// Initialize the PCA9685
  pwm.begin();
// Set the PWM frequency
  pwm.setPWMFreq(SERVO_FREQ);
  InitUltrasonic();
  calibrateBot();
}


void loop() {
  juncType junc = junction();
  if (junc == T_JUNC) {
    stop();
    rotateMotors(90);
    rotateMotors(90);
  } else if (junc == CROSS) {
    stop();
    rotateMotors(90);
    rotateMotors(90);
    rotateMotors(90);
    rotateMotors(90);
  } else if (junc == RIGHT_BEND) {
    stop();
    rotateMotors(90);
  } else if (junc == RIGHT_BRANCH) {
    stop();
    rotateMotors(-90);
    rotateMotors(-90);
    rotateMotors(-90);
  } else if (junc == LEFT_BEND) {
    stop();
    rotateMotors(-90);
  } else if (junc == LEFT_BRANCH) {
    stop();
    rotateMotors(90);
    rotateMotors(90);
    rotateMotors(90);
  }else if (junc == DEAD_END) {
    GoDistance(8);
    stop();
    rotateMotors(90);
    rotateMotors(90);
  } else {
    read_line();
  }
  read_line();
  CheckForBox();
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

  pinMode(ir_rm, INPUT);
  pinMode(ir_r, INPUT);
  pinMode(ir_m, INPUT);
  pinMode(ir_l, INPUT);
  pinMode(ir_lm, INPUT);

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

  float WheelTurnDeg = BotTurnDeg * 2 * (22.0 / 7.0) * 9 / 21;
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
// STOPS THE ROBOT
// =============================================================
void stop() {
  delay(500);
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


// --- Servo Movement Functions ---

void grab() {
  // Move servo1 (gripper) to 90 degrees
  for (int angle = servo1_angle; angle <= 90; angle++) {
    setServoAngle(SERVO1_CHANNEL, angle);
    servo1_angle = angle;
    delay(20);
  }
}

void release() {
  // Move servo1 (gripper) down to 30 degrees
  for (int angle = servo1_angle; angle >= 30; angle--) {
    setServoAngle(SERVO1_CHANNEL, angle);
    servo1_angle = angle;
    delay(20);
  }
}

void up_straight() {
  // Move servo2 (arm) to 150 degrees
  int target_angle = 150;

  if (servo2_angle > target_angle) {
    // Current angle is > 150, so DECREMENT (--)
    for (int angle = servo2_angle; angle >= target_angle; angle--) {
      setServoAngle(SERVO2_CHANNEL, angle);
      servo2_angle = angle;
      delay(20);
    }
  } else if (servo2_angle < target_angle) {
    // Current angle is < 150, so INCREMENT (++)
    for (int angle = servo2_angle; angle <= target_angle; angle++) {
      setServoAngle(SERVO2_CHANNEL, angle);
      servo2_angle = angle;
      delay(20);
    }
  }
  // If servo2_angle == 150, the function does nothing (correct)
}

void up_storage() {
  // Move servo2 down to 20 degrees
  for (int angle = servo2_angle; angle >= 20; angle--) {
    setServoAngle(SERVO2_CHANNEL, angle);
    servo2_angle = angle;
    delay(20);
  }
}

void down_box() {
  // Move servo2 up to 180 degrees
  for (int angle = servo2_angle; angle <= 180; angle++) {
    setServoAngle(SERVO2_CHANNEL, angle);
    servo2_angle = angle;
    delay(20);
  }
}

void down_ball() {
  // Move servo2 down to 0 degrees
  for (int angle = servo2_angle; angle >= 0; angle--) {
    setServoAngle(SERVO2_CHANNEL, angle);
    servo2_angle = angle;
    delay(10);
  }
}

//for first box
void grab_first_box() {
  delay(2000);
  up_straight();
  delay(1000);
  down_box();
  delay(1000);
  grab();
  up_straight();
  delay(1000);
  up_storage();
  delay(1000);
}

void CheckForBox() {
  float obs = ReadDistanceCM();

  if (obs <= 7) {
    // --- Object detected 5cm ahead ---
    GoDistance(2);
    stop();  // Your function to stop motors

    GoDistance(obs - 13);  // Move BACK 10 cm (you can change this)
    delay(200);

    grab_first_box();  // Arm goes down and grabs the box
  }
}
