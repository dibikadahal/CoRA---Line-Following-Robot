/*
  5-Sensor IR Line Follower for Arduino Uno + L298N

  Hardware:
  - IR sensors: A0, A1, A2, A3, A4
  - L298N: ENA=5, ENB=6, IN1=7, IN2=8, IN3=9, IN4=10
  - Start button: pin 2 to GND, using INPUT_PULLUP

  Start with CONTROL_MODE = SIMPLE_MODE.
  After the robot follows the line reliably, change it to PID_MODE and tune Kp/Kd.
*/

// ---------- CONTROL MODE ----------
enum ControlMode {
  SIMPLE_MODE,
  PID_MODE
};

ControlMode CONTROL_MODE = SIMPLE_MODE;  // Change to PID_MODE after testing simple mode.

// ---------- SENSOR SETTINGS ----------
const byte SENSOR_COUNT = 5;
const byte sensorPins[SENSOR_COUNT] = {A0, A1, A2, A3, A4};
const int sensorWeights[SENSOR_COUNT] = {-2, -1, 0, 1, 2};

int sensorRaw[SENSOR_COUNT];
bool sensorActive[SENSOR_COUNT];

int threshold = 500;        // Adjust after checking Serial Monitor values.
bool lineIsDark = true;     // true: black line gives lower reading. false: black line gives higher reading.

// ---------- MOTOR PINS ----------
const byte ENA = 5;
const byte ENB = 6;
const byte IN1 = 7;
const byte IN2 = 8;
const byte IN3 = 9;
const byte IN4 = 10;

// Flip either toggle if that side moves backward when it should move forward.
bool leftMotorReversed = false;
bool rightMotorReversed = false;

// ---------- BUTTON ----------
const byte buttonPin = 2;
bool started = false;

// ---------- SPEED SETTINGS ----------
int baseSpeed = 110;        // Editable driving speed. Keep PID output constrained to 0-180.
int turnSpeed = 105;        // Used by simple mode and line-lost recovery.
const int maxSpeed = 180;

// ---------- PID SETTINGS ----------
float Kp = 35.0;
float Ki = 0.0;             // Start at 0. Add only if the robot has steady offset.
float Kd = 18.0;

float error = 0.0;
float lastError = 0.0;
float integral = 0.0;

// ---------- DEBUG SETTINGS ----------
unsigned long lastDebugTime = 0;
const unsigned long debugIntervalMs = 120;

void setup() {
  Serial.begin(9600);

  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();

  Serial.println("=== 5 SENSOR LINE FOLLOWER READY ===");
  Serial.println("Press the tact switch to start.");
  Serial.println("Mode: SIMPLE_MODE first. Change CONTROL_MODE to PID_MODE after basic testing.");
}

void loop() {
  waitForStartButton();

  if (!started) {
    stopMotors();
    return;
  }

  readSensors();
  printDebug();

  if (CONTROL_MODE == SIMPLE_MODE) {
    runSimpleLineFollower();
  } else {
    runPIDLineFollower();
  }
}

// ---------- START BUTTON ----------
void waitForStartButton() {
  if (started) {
    return;
  }

  if (digitalRead(buttonPin) == LOW) {
    delay(30);  // Simple debounce.
    if (digitalRead(buttonPin) == LOW) {
      started = true;
      integral = 0.0;
      lastError = 0.0;
      Serial.println(">>> ROBOT STARTED <<<");

      while (digitalRead(buttonPin) == LOW) {
        delay(5);
      }
    }
  }
}

// ---------- SENSOR READING ----------
void readSensors() {
  for (byte i = 0; i < SENSOR_COUNT; i++) {
    sensorRaw[i] = analogRead(sensorPins[i]);

    if (lineIsDark) {
      sensorActive[i] = sensorRaw[i] < threshold;
    } else {
      sensorActive[i] = sensorRaw[i] > threshold;
    }
  }
}

bool anySensorActive() {
  for (byte i = 0; i < SENSOR_COUNT; i++) {
    if (sensorActive[i]) {
      return true;
    }
  }
  return false;
}

// ---------- SIMPLE NON-PID LINE FOLLOWER ----------
void runSimpleLineFollower() {
  bool farLeft = sensorActive[0];
  bool left = sensorActive[1];
  bool center = sensorActive[2];
  bool right = sensorActive[3];
  bool farRight = sensorActive[4];

  if (center) {
    moveForward();
    lastError = 0;
  } else if (farLeft || left) {
    turnLeft();
    lastError = -1;
  } else if (right || farRight) {
    turnRight();
    lastError = 1;
  } else {
    recoverLine();
  }
}

// ---------- PID LINE FOLLOWER ----------
void runPIDLineFollower() {
  int activeCount = 0;
  int weightedSum = 0;

  for (byte i = 0; i < SENSOR_COUNT; i++) {
    if (sensorActive[i]) {
      weightedSum += sensorWeights[i];
      activeCount++;
    }
  }

  if (activeCount == 0) {
    recoverLine();
    return;
  }

  error = (float)weightedSum / activeCount;

  integral += error;
  integral = constrain(integral, -25.0, 25.0);

  float derivative = error - lastError;
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  int leftSpeed = constrain(baseSpeed + correction, 0, maxSpeed);
  int rightSpeed = constrain(baseSpeed - correction, 0, maxSpeed);

  setMotorSpeeds(leftSpeed, rightSpeed);
  lastError = error;
}

void recoverLine() {
  if (lastError < 0) {
    turnLeft();
  } else if (lastError > 0) {
    turnRight();
  } else {
    stopMotors();
  }
}

// ---------- MOTOR FUNCTIONS ----------
void moveForward() {
  setMotorSpeeds(baseSpeed, baseSpeed);
}

void turnLeft() {
  setLeftMotor(-turnSpeed);
  setRightMotor(turnSpeed);
}

void turnRight() {
  setLeftMotor(turnSpeed);
  setRightMotor(-turnSpeed);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);
}

void setLeftMotor(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -maxSpeed, maxSpeed);
  bool forward = motorSpeed >= 0;

  if (leftMotorReversed) {
    forward = !forward;
  }

  digitalWrite(IN1, forward ? HIGH : LOW);
  digitalWrite(IN2, forward ? LOW : HIGH);
  analogWrite(ENA, abs(motorSpeed));
}

void setRightMotor(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -maxSpeed, maxSpeed);
  bool forward = motorSpeed >= 0;

  if (rightMotorReversed) {
    forward = !forward;
  }

  digitalWrite(IN3, forward ? HIGH : LOW);
  digitalWrite(IN4, forward ? LOW : HIGH);
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
  Serial.print(error);

  Serial.print(" | MODE:");
  Serial.println(CONTROL_MODE == SIMPLE_MODE ? "SIMPLE" : "PID");
}
