// ====== PID Line Following Robot with Button-Controlled States ======
// 5 TCRT5000 sensors, 2 PWM motors, 1 active-HIGH button, 2 LEDs

// --- Pins ---
const int LEFT_MOTOR_PWM = 5;
const int RIGHT_MOTOR_PWM = 6;

const int sensorPins[5] = {A0, A1, A2, A3, A4};
const int BUTTON_PIN = 7;   // Active HIGH
const int LED_CAL = 8;      // Calibration LED
const int LED_RUN = 9;      // Running LED

// --- PID constants (tune for your robot) ---
float Kp = 25.0;
float Ki = 0.0;
float Kd = 8.0;

// --- PID variables ---
float error = 0, lastError = 0;
float integral = 0;
float derivative = 0;
float output = 0;

// --- Integral setup (for later) ---
unsigned long lastTime = 0;
float integralMax = 100.0; // anti-windup clamp

// --- Motor control ---
int baseSpeed = 120;
int maxSpeed  = 255;

// --- Calibration levels ---
int whiteLevel = 0;
int blackLevel = 1023;

// --- State management ---
enum RobotState { 
  IDLE, 
  CALIBRATION, 
  RUNNING 
};

RobotState state = IDLE;

void setup() {
  Serial.begin(9600);

  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);


  pinMode(BUTTON_PIN, INPUT);  // Active HIGH
  pinMode(LED_CAL, OUTPUT);
  pinMode(LED_RUN, OUTPUT);

  // Default: LEDs OFF
  digitalWrite(LED_CAL, LOW);
  digitalWrite(LED_RUN, LOW);
  lastTime = millis();


  Serial.println("Waiting for button press to start calibration...");
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
  stopMotors();

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

  // Wait for next button press
  while (!buttonPressed()) delay(10);

  state = RUNNING;
  delay(200);
}

// ====== STATE: RUNNING ======
void runningState() {
  digitalWrite(LED_RUN, HIGH);
  digitalWrite(LED_CAL, LOW);

  // --- PID Line Following Loop ---
  int sensorValues[5];
  float weights[5] = {-2, -1, 0, 1, 2};

  for (int i = 0; i < 5; i++)
    sensorValues[i] = analogRead(sensorPins[i]);

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
  if (total != 0) {
    position = weightedSum / total;
  }
  static bool firstRun = true;
  unsigned long now = millis();
  if (firstRun) {
    lastTime = now;
    firstRun = false;
  }
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0; // seconds
  if (dt <= 0) dt = 0.001; // safety
  lastTime = now;

  if (total == 0) {
    integral = 0;
  } else {
    // integrate with dt
    integral += error * dt;

    // optional: only integrate when error small to avoid accumulating while correcting big mistakes
    // if (abs(error) > 1.0) integral = 0;

    // anti-windup clamp
    if (integral > integralMax) integral = integralMax;
    if (integral < -integralMax) integral = -integralMax;
  }

  // derivative (time-based)
  float derivative = (error - lastError) / dt;

  // PID output
  output = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;
  // --- PID control ---
  output = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  // --- Motor control ---
  int leftSpeed  = constrain(baseSpeed + output, 0, maxSpeed);
  int rightSpeed = constrain(baseSpeed - output, 0, maxSpeed);

  analogWrite(LEFT_MOTOR_PWM, leftSpeed);
  analogWrite(RIGHT_MOTOR_PWM, rightSpeed);

  // Debug info
  Serial.print("Pos: "); Serial.print(position);
  Serial.print("  Err: "); Serial.print(error);
  Serial.print("  L: "); Serial.print(leftSpeed);
  Serial.print("  R: "); Serial.println(rightSpeed);

  // === Stop on button press ===
  if (buttonPressed()) {
    Serial.println("Stopping robot...");
    stopMotors();
    digitalWrite(LED_RUN, LOW);
    state = IDLE;
    delay(500);
  }

  delay(10);
}

// ====== HELPER: Calibration ======
void calibrateSensors() {
  // --- Reset Values ---
  integral = 0;
  lastError = 0;
  
  whiteLevel = 0;
  blackLevel = 1023;
  unsigned long startTime = millis();
  unsigned long calibrationTime = 3000; // 3s

  while (millis() - startTime < calibrationTime) {
    for (int i = 0; i < 5; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < blackLevel) blackLevel = val;
      if (val > whiteLevel) whiteLevel = val;
    }
    delay(5);
  }
}

// ====== HELPER: Button Press Detection (Active HIGH, debounced) ======
bool buttonPressed() {
  static unsigned long lastPress = 0;
  static bool prevState = LOW;
  bool current = digitalRead(BUTTON_PIN);

  if (prevState == LOW && current == HIGH && millis() - lastPress > 250) {
    lastPress = millis();
    prevState = HIGH;
    return true;
  } else if (current == LOW) {
    prevState = LOW;
  }
  return false;
}

// ====== HELPER: Stop Motors ======
void stopMotors() {
  analogWrite(LEFT_MOTOR_PWM, 0);
  analogWrite(RIGHT_MOTOR_PWM, 0);
}
