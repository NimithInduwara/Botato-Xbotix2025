/* ===============================================================
   Dual Motor: 25JGA370 Planetary Gear Motors
   Gear Ratio: 34:1
   Encoder: 12 PPR → 48 CPR (Quadrature)
   Output Shaft CPR = 48 * 34 = 1632
   Angle/Count = 360 / 1632 = 0.220588 deg
   Arduino MEGA version
   =============================================================== */

const float anglePerCount = 360.0 / (48.0 * 34.0);

// ---------- PID Gains ----------
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
  int PWM = map(pwmVal, 0, 255, 150, 255);

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
// CALLABLE FUNCTION — THIS IS WHAT YOU WILL USE
// =============================================================
void GoDistance(float BotGoDis) {

  float WheelGoDeg=BotGoDis*180/((22/7)*8.7);
  // set targets
  motorA.targetDeg = WheelGoDeg;
  motorB.targetDeg = -WheelGoDeg;

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


void setup() {
  Serial.begin(115200);

  // Motor A pins
  motorA.IN1 = 7;
  motorA.IN2 = 6;
  motorA.EN  = 12;
  motorA.encA = 2;
  motorA.encB = 3;

  // Motor B pins
  motorB.IN1 = 4;
  motorB.IN2 = 5;
  motorB.EN  = 11;
  motorB.encA = 18;  
  motorB.encB = 19;

  pinMode(motorA.IN1, OUTPUT);
  pinMode(motorA.IN2, OUTPUT);
  pinMode(motorA.EN, OUTPUT);

  pinMode(motorB.IN1, OUTPUT);
  pinMode(motorB.IN2, OUTPUT);
  pinMode(motorB.EN, OUTPUT);

  pinMode(motorA.encA, INPUT_PULLUP);
  pinMode(motorA.encB, INPUT_PULLUP);
  pinMode(motorB.encA, INPUT_PULLUP);
  pinMode(motorB.encB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(motorA.encA), updateMotorA_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorA.encB), updateMotorA_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorB.encA), updateMotorB_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorB.encB), updateMotorB_B, CHANGE);
}


void loop() {

  GoDistance(5); 

  delay(2000);

  GoDistance(10);  

  delay(2000);
  
  GoDistance(0); 

  delay(2000);
}

