// 5 IR Sensor Line Follower - L293D
// ENA=5, ENB=3, IN1=7, IN2=8, IN3=9, IN4=12
// Sensors: A0, A1, A2, A3, A4 (used as digital pins 14,15,16,17,18)

// --- Motor Pins ---
const int ENA = 5;
const int ENB = 3;
const int IN1 = 7;
const int IN2 = 8;
const int IN3 = 9;
const int IN4 = 12;

// --- Sensor Pins (A0-A4 = digital 14-18) ---
const int S1 = A0; // far left
const int S2 = A1; // left
const int S3 = A2; // center
const int S4 = A3; // right
const int S5 = A4; // far right

// --- Speed ---
const int SPEED = 200;

int lastDir = 0; // -1 = left, 0 = straight, 1 = right

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);

  stopMotors();
  delay(1000);
}

void loop() {
  bool s1 = digitalRead(S1);
  bool s2 = digitalRead(S2);
  bool s3 = digitalRead(S3);
  bool s4 = digitalRead(S4);
  bool s5 = digitalRead(S5);

  Serial.print("BIN: ");
  Serial.print(s1); Serial.print(s2); Serial.print(s3);
  Serial.print(s4); Serial.print(s5); Serial.print("  ");

  // No line detected = recover
  if (!s1 && !s2 && !s3 && !s4 && !s5) {
    if (lastDir == -1) {
      turnLeft();
    } else if (lastDir == 1) {
      turnRight();
    } else {
      stopMotors();
    }
    Serial.println("LOST / RECOVER");

  // All sensors = intersection, go forward
  } else if (s1 && s2 && s3 && s4 && s5) {
    goForward();
    lastDir = 0;
    Serial.println("INTERSECTION FORWARD");

  // Line on left = turn left
  } else if (s1 || s2) {
    turnLeft();
    lastDir = -1;
    Serial.println("TURN LEFT");

  // Line on right = turn right
  } else if (s4 || s5) {
    turnRight();
    lastDir = 1;
    Serial.println("TURN RIGHT");

  // Center or default = go forward
  } else {
    goForward();
    lastDir = 0;
    Serial.println("FORWARD");
  }
}

void goForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void turnLeft() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}