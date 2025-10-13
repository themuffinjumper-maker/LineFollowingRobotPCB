// ====== PID Line Following Robot with Button-Controlled Calibration ======
// 5 TCRT5000 analog sensors, 2 PWM motors, button + 2 LEDs

// --- Pins ---
const int LEFT_MOTOR_PWM = 5;
const int RIGHT_MOTOR_PWM = 6;

const int sensorPins[5] = {A0, A1, A2, A3, A4};
const int BUTTON_PIN = 7;
const int LED_CAL = 8;    // lights during calibration
const int LED_RUN = 9;    // lights during line following

// --- PID constants (tune for your robot) ---
float Kp = 25.0;
float Ki = 0.0;
float Kd = 8.0;

// --- PID variables ---
float error = 0, lastError = 0;
float integral = 0;
float derivative = 0;
float output = 0;

// --- Motor control ---
int baseSpeed = 120;
int maxSpeed  = 255;

// --- Calibration levels ---
int whiteLevel = 0;
int blackLevel = 1023;

// --- State management ---
enum RobotState { IDLE, CALIBRATION, RUNNING };
RobotState state = IDLE;

void setup() {
  Serial.begin(9600);

  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_CAL, OUTPUT);
  pinMode(LED_RUN, OUTPUT);

  // Default: both LEDs off
  digitalWrite(LED_CAL, LOW);
  digitalWrite(LED_RUN, LOW);

  Serial.println("Waiting for button to start calibration...");
}

void loop() {
  switch (state) {
    case IDLE:
      idleState();
      break;

    case CALIBRATION:
      calibrationState();
      break;

    case RUNNING:
      runningState();
      break;
  }
}

// ====== STATE: IDLE ======
void idleState() {
  digitalWrite(LED_CAL, LOW);
  digitalWrite(LED_RUN, LOW);

  if (buttonPressed()) {
    state = CALIBRATION;
    delay(200);
  }
}

// ====== STATE: CALIBRATION ======
void calibrationState() {
  digitalWrite(LED_CAL, HIGH);
  digitalWrite(LED_RUN, LOW);

  Serial.println("Calibrating sensors...");
  calibrateSensors();
  Serial.print("White level: "); Serial.println(whiteLevel);
  Serial.print("Black level: "); Serial.println(blackLevel);

  digitalWrite(LED_CAL, LOW);
  Serial.println("Calibration done. Press button again to start running.");

  // Wait for second press
  while (!buttonPressed()) {
    delay(10);
  }

  state = RUNNING;
  delay(200);
}

// ====== STATE: RUNNING ======
void runningState() {
  digitalWrite(LED_RUN, HIGH);
  digitalWrite(LED_CAL, LOW);

  int sensorValues[5];
  float weights[5] = {-2, -1, 0, 1, 2};

  // Read sensors
  for (int i = 0; i < 5; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }

  // Normalize (0 = black, 1 = white)
  float normalized[5];
  for (int i = 0; i < 5; i++) {
    normalized[i] = constrain((float)(sensorValues[i] - blackLevel) / (whiteLevel - blackLevel), 0, 1);
  }

  // Weighted position calculation
  float weightedSum = 0;
  float total = 0;
  for (int i = 0; i < 5; i++) {
    float lineSignal = 1.0 - normalized[i]; // black = 1
    weightedSum += weights[i] * lineSignal;
    total += lineSignal;
  }

  float position = 0;
  if (total != 0)
    position = weightedSum / total;

  // PID control
  error = position;
  integral += error;
  derivative = error - lastError;
  output = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  // Motor speed adjustment
  int leftSpeed  = constrain(baseSpeed + output, 0, maxSpeed);
  int rightSpeed = constrain(baseSpeed - output, 0, maxSpeed);

  analogWrite(LEFT_MOTOR_PWM, leftSpeed);
  analogWrite(RIGHT_MOTOR_PWM, rightSpeed);

  // Debug
  Serial.print("Pos: "); Serial.print(position);
  Serial.print("  Err: "); Serial.print(error);
  Serial.print("  L: "); Serial.print(leftSpeed);
  Serial.print("  R: "); Serial.println(rightSpeed);

  delay(10);
}

// ====== HELPER: Sensor Calibration ======
void calibrateSensors() {
  whiteLevel = 0;
  blackLevel = 1023;
  unsigned long startTime = millis();
  unsigned long calibrationTime = 3000; // 3 seconds

  while (millis() - startTime < calibrationTime) {
    for (int i = 0; i < 5; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < blackLevel) blackLevel = val;
      if (val > whiteLevel) whiteLevel = val;
    }
    delay(5);
  }
}

// ====== HELPER: Button Press Detection with Debounce ======
bool buttonPressed() {
  static unsigned long lastPress = 0;
  static bool prevState = HIGH;
  bool current = digitalRead(BUTTON_PIN);

  if (prevState == HIGH && current == LOW && millis() - lastPress > 250) {
    lastPress = millis();
    prevState = LOW;
    return true;
  } else if (current == HIGH) {
    prevState = HIGH;
  }
  return false;
}
