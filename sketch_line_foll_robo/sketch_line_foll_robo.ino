/*
  High-Speed 5-Sensor IR Line Follower
  Arduino Uno + L298N Motor Driver

  Sensors:
  A0 A1 A2 A3 A4 = left edge, left, center, right, right edge

  Motor driver:
  ENA=5, ENB=6, IN1=7, IN2=8, IN3=9, IN4=10

  Control style:
  - Threshold detection from analogRead()
  - Weighted proportional steering, no full PID
  - Dedicated sharp/90-degree turn handling
  - Lost-line recovery using last known direction
*/

// ---------- SENSOR SETTINGS ----------
const byte SENSOR_COUNT = 5;
const byte sensorPins[SENSOR_COUNT] = {A0, A1, A2, A3, A4};
const int sensorWeights[SENSOR_COUNT] = {-2, -1, 0, 1, 2};

int sensorRaw[SENSOR_COUNT];
bool sensorActive[SENSOR_COUNT];

int threshold = 500;          // Tune using Serial Monitor.
int strongMargin = 150;       // Edge must beat threshold by this much for 90-degree detection.
bool lineIsDark = true;       // true: black line gives lower analog value.

// ---------- L298N MOTOR PINS ----------
const byte ENA = 5;
const byte ENB = 6;
const byte IN1 = 7;
const byte IN2 = 8;
const byte IN3 = 9;
const byte IN4 = 10;

/*
  Direction flags:
  Positive motor speed means physical forward.

  Test these true/false combinations with the robot lifted off the ground:
  1. Set both false, upload, and command straight movement.
  2. If both wheels spin backward, set both true.
  3. If only the left wheel spins backward, flip only leftMotorReversed.
  4. If only the right wheel spins backward, flip only rightMotorReversed.

  Do not change IN1-IN4 wiring in code once these flags make positive speed
  move the robot physically forward.
*/
bool leftMotorReversed = true;
bool rightMotorReversed = true;

// ---------- HIGH-SPEED TUNING ----------
const int maxSpeed = 255;

int baseSpeed = 255;          // Full speed only when centered on the line.
int curveBaseSpeed = 170;     // Slower speed for curves and waves.
int minForwardSpeed = 80;     // Keeps inner wheel moving on normal curves.
int pivotSpeed = 190;         // Sharp turns and 90-degree turns.
int recoverySpeed = 150;      // Reserved for recovery after testing.

float Kp = 55.0;              // Proportional steering only.
float correctionFilter = 0.45; // 0.0 = very smooth, 1.0 = instant response.
float deadband = 0.15;        // Prevents small sensor noise from twitching motors.

// ---------- TURN STATE ----------
enum RobotState {
  FOLLOW_LINE,
  TURN_LEFT_90,
  TURN_RIGHT_90,
  LOST_LINE
};

RobotState robotState = FOLLOW_LINE;

float lineError = 0.0;
float lastError = 0.0;
float smoothedCorrection = 0.0;
int lastDirection = 0;        // -1 left, 0 straight, 1 right.
unsigned long turnStartTime = 0;
const unsigned long minTurnTimeMs = 90;     // Avoids exiting a 90-degree turn too early.
const unsigned long maxTurnTimeMs = 900;    // Escape hatch if the center sensor never finds line.

// ---------- DEBUG ----------
unsigned long lastDebugTime = 0;
const unsigned long debugIntervalMs = 100;
const char *currentAction = "STOP";
int lastLeftCommand = 0;
int lastRightCommand = 0;

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();

  Serial.println("=== HIGH-SPEED 5 SENSOR LINE FOLLOWER ===");
  Serial.println("Positive speed = physical forward. Use motor reverse flags to fix direction.");
  Serial.print("Left reversed: ");
  Serial.print(leftMotorReversed ? "YES" : "NO");
  Serial.print(" | Right reversed: ");
  Serial.println(rightMotorReversed ? "YES" : "NO");
}

void loop() {
  readSensors();
  runRobot();
  printDebug();
}

// ---------- SENSOR READING ----------
void readSensors() {
  for (byte i = 0; i < SENSOR_COUNT; i++) {
    sensorRaw[i] = analogRead(sensorPins[i]);
    sensorActive[i] = lineIsDark ? sensorRaw[i] < threshold : sensorRaw[i] > threshold;
  }
}

bool isStrongLine(byte index) {
  if (lineIsDark) {
    return sensorRaw[index] < (threshold - strongMargin);
  }
  return sensorRaw[index] > (threshold + strongMargin);
}

byte activeSensorCount() {
  byte count = 0;
  for (byte i = 0; i < SENSOR_COUNT; i++) {
    if (sensorActive[i]) {
      count++;
    }
  }
  return count;
}

float calculateLineError() {
  int weightedSum = 0;
  int activeCount = 0;

  for (byte i = 0; i < SENSOR_COUNT; i++) {
    if (sensorActive[i]) {
      weightedSum += sensorWeights[i];
      activeCount++;
    }
  }

  if (activeCount == 0) {
    return lastError;
  }

  return (float)weightedSum / activeCount;
}

// ---------- MAIN CONTROL ----------
void runRobot() {
  switch (robotState) {
    case FOLLOW_LINE:
      followLine();
      break;

    case TURN_LEFT_90:
      rotateUntilCenter(-1);
      break;

    case TURN_RIGHT_90:
      rotateUntilCenter(1);
      break;

    case LOST_LINE:
      recoverLostLine();
      break;
  }
}

void followLine() {
  byte activeCount = activeSensorCount();

  if (activeCount == 0) {
    stopMotors();
    robotState = FOLLOW_LINE;
    currentAction = "NO LINE STOP";
    smoothedCorrection = 0.0;
    return;
  }

  if (activeCount == SENSOR_COUNT) {
    stopMotors();
    robotState = FOLLOW_LINE;
    currentAction = "ALL SENSORS ACTIVE - STOP";
    smoothedCorrection = 0.0;
    return;
  }

  if (detectLeft90()) {
    startNinetyTurn(-1);
    return;
  }

  if (detectRight90()) {
    startNinetyTurn(1);
    return;
  }

  lineError = calculateLineError();

  if (isStrongLine(0) && !sensorActive[2]) {
    lastError = -2.0;
    lastDirection = -1;
    setMotorSpeeds(-(pivotSpeed / 2), pivotSpeed);
    currentAction = "AGGRESSIVE LEFT";
    return;
  }

  if (isStrongLine(4) && !sensorActive[2]) {
    lastError = 2.0;
    lastDirection = 1;
    setMotorSpeeds(pivotSpeed, -(pivotSpeed / 2));
    currentAction = "AGGRESSIVE RIGHT";
    return;
  }

  if (lineError > -deadband && lineError < deadband) {
    lineError = 0.0;
  }

  lastError = lineError;
  if (lineError < -deadband) {
    lastDirection = -1;
  } else if (lineError > deadband) {
    lastDirection = 1;
  }

  float rawCorrection = Kp * lineError;
  smoothedCorrection += correctionFilter * (rawCorrection - smoothedCorrection);

  int driveSpeed = lineError == 0.0 ? baseSpeed : curveBaseSpeed;
  int speedLimit = lineError == 0.0 ? maxSpeed : pivotSpeed;
  int leftSpeed = driveSpeed + smoothedCorrection;
  int rightSpeed = driveSpeed - smoothedCorrection;

  leftSpeed = constrain(leftSpeed, minForwardSpeed, speedLimit);
  rightSpeed = constrain(rightSpeed, minForwardSpeed, speedLimit);

  setMotorSpeeds(leftSpeed, rightSpeed);

  if (lineError == 0.0) {
    currentAction = "FAST STRAIGHT";
  } else if (lineError < 0) {
    currentAction = "SMOOTH LEFT";
  } else {
    currentAction = "SMOOTH RIGHT";
  }
}

bool detectLeft90() {
  bool leftEdgeStrong = isStrongLine(0);
  bool leftPair = sensorActive[0] && sensorActive[1];
  bool rightClear = !sensorActive[3] && !sensorActive[4];
  return leftEdgeStrong && leftPair && rightClear && !sensorActive[2];
}

bool detectRight90() {
  bool rightEdgeStrong = isStrongLine(4);
  bool rightPair = sensorActive[3] && sensorActive[4];
  bool leftClear = !sensorActive[0] && !sensorActive[1];
  return rightEdgeStrong && rightPair && leftClear && !sensorActive[2];
}

void startNinetyTurn(int direction) {
  turnStartTime = millis();
  lastDirection = direction;
  smoothedCorrection = 0.0;

  if (direction < 0) {
    robotState = TURN_LEFT_90;
    pivotLeft();
  } else {
    robotState = TURN_RIGHT_90;
    pivotRight();
  }
}

void rotateUntilCenter(int direction) {
  unsigned long elapsed = millis() - turnStartTime;

  byte activeCount = activeSensorCount();
  if (activeCount == 0) {
    stopMotors();
    robotState = FOLLOW_LINE;
    currentAction = "NO LINE STOP";
    smoothedCorrection = 0.0;
    return;
  }

  if (activeCount == SENSOR_COUNT) {
    stopMotors();
    robotState = FOLLOW_LINE;
    currentAction = "ALL SENSORS ACTIVE - STOP";
    smoothedCorrection = 0.0;
    return;
  }

  if (direction < 0) {
    pivotLeft();
  } else {
    pivotRight();
  }

  if (elapsed > minTurnTimeMs && sensorActive[2]) {
    robotState = FOLLOW_LINE;
    lastError = 0.0;
    smoothedCorrection = 0.0;
    moveForwardFast();
    return;
  }

  if (elapsed > maxTurnTimeMs) {
    robotState = LOST_LINE;
  }
}

void recoverLostLine() {
  byte activeCount = activeSensorCount();

  if (activeCount == 0) {
    stopMotors();
    robotState = FOLLOW_LINE;
    currentAction = "NO LINE STOP";
    smoothedCorrection = 0.0;
    return;
  }

  if (activeCount == SENSOR_COUNT) {
    stopMotors();
    robotState = FOLLOW_LINE;
    currentAction = "ALL SENSORS ACTIVE - STOP";
    smoothedCorrection = 0.0;
    return;
  }

  if (activeCount > 0) {
    robotState = FOLLOW_LINE;
    followLine();
    return;
  }
}

// ---------- MOTOR MOVEMENT HELPERS ----------
void moveForwardFast() {
  setMotorSpeeds(baseSpeed, baseSpeed);
  currentAction = "FAST STRAIGHT";
}

void pivotLeft() {
  setMotorSpeeds(-pivotSpeed, pivotSpeed);
  currentAction = "PIVOT LEFT 90";
}

void pivotRight() {
  setMotorSpeeds(pivotSpeed, -pivotSpeed);
  currentAction = "PIVOT RIGHT 90";
}

void stopMotors() {
  lastLeftCommand = 0;
  lastRightCommand = 0;
  currentAction = "STOP";

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ---------- L298N MOTOR DIRECTION HANDLING ----------
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);
}

void setLeftMotor(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -maxSpeed, maxSpeed);
  lastLeftCommand = motorSpeed;

  if (motorSpeed == 0) {
    analogWrite(ENA, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    return;
  }

  bool physicalForward = motorSpeed > 0;
  bool driverForward = leftMotorReversed ? !physicalForward : physicalForward;

  digitalWrite(IN1, driverForward ? HIGH : LOW);
  digitalWrite(IN2, driverForward ? LOW : HIGH);
  analogWrite(ENA, abs(motorSpeed));
}

void setRightMotor(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -maxSpeed, maxSpeed);
  lastRightCommand = motorSpeed;

  if (motorSpeed == 0) {
    analogWrite(ENB, 0);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    return;
  }

  bool physicalForward = motorSpeed > 0;
  bool driverForward = rightMotorReversed ? !physicalForward : physicalForward;

  digitalWrite(IN3, driverForward ? HIGH : LOW);
  digitalWrite(IN4, driverForward ? LOW : HIGH);
  analogWrite(ENB, abs(motorSpeed));
}

// ---------- SERIAL DEBUG ----------
void printDebug() {
  if (millis() - lastDebugTime < debugIntervalMs) {
    return;
  }
  lastDebugTime = millis();

  Serial.print("RAW:");
  for (byte i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(' ');
    Serial.print(sensorRaw[i]);
  }

  Serial.print(" | BIN:");
  for (byte i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(sensorActive[i] ? '1' : '0');
  }

  Serial.print(" | ERR:");
  Serial.print(lineError);

  Serial.print(" | STATE:");
  Serial.print(stateName());

  Serial.print(" | ACTION:");
  Serial.print(currentAction);

  Serial.print(" | L:");
  Serial.print(lastLeftCommand);
  Serial.print(leftMotorReversed ? " inv" : " norm");

  Serial.print(" | R:");
  Serial.print(lastRightCommand);
  Serial.println(rightMotorReversed ? " inv" : " norm");
}

const char *stateName() {
  switch (robotState) {
    case FOLLOW_LINE:
      return "FOLLOW";
    case TURN_LEFT_90:
      return "LEFT90";
    case TURN_RIGHT_90:
      return "RIGHT90";
    case LOST_LINE:
      return "LOST";
  }
  return "?";
}
