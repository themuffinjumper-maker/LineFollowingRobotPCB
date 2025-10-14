// ============================
// Line Following Robot (PID)
// ============================
// 5x TCRT5000 sensors (analog inputs)
// 2x PWM motor outputs
// 1x Active HIGH button input
// 2x LED outputs for state indication

// ----------------------------
// Pin Configuration
// ----------------------------
const uint8_t sensorPins[5] = {A0, A1, A2, A3, A4};
const uint8_t leftMotorPWM = 6;
const uint8_t rightMotorPWM = 5;
const uint8_t buttonPin = 7;
const uint8_t ledCalibrate = 9;
const uint8_t ledRunning = 8;

// ----------------------------
// PID Constants
// ----------------------------
uint8_t Kp = 10;
//uint8_t Ki = 0;    // start with 0, tune later if needed
uint8_t Kd = 0;

// ----------------------------
// Variables
// ----------------------------
uint8_t sensorValues[5];
uint16_t sensorMin[5];
uint16_t sensorMax[5];

int8_t lastError = 0;
//int16_t integral = 0;

// Robot speed limits
const uint8_t baseSpeed = 49;    // adjust to your motor strength
const uint8_t maxSpeed = 51;

// State management
enum State {
  IDLE, 
  CALIBRATING, 
  RUNNING
};
State robotState = IDLE;

// ----------------------------
// Setup
// ----------------------------
void setup() {
  Serial.begin(9600);

  pinMode(leftMotorPWM, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(ledCalibrate, OUTPUT);
  pinMode(ledRunning, OUTPUT);

  // Initialize sensor calibration ranges
  for (int i = 0; i < 5; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  analogWrite(leftMotorPWM, 0);
  analogWrite(rightMotorPWM, 0);
}

// ----------------------------
// Loop
// ----------------------------
void loop() {
  switch (robotState) {
    case IDLE:
      idleState();
      break;
    case CALIBRATING:
      calibrateState();
      break;
    case RUNNING:
      runningState();
      break;
  }
}

// ----------------------------
// IDLE STATE
// ----------------------------
void idleState() {
  digitalWrite(ledCalibrate, LOW);
  digitalWrite(ledRunning, LOW);
  analogWrite(leftMotorPWM, 0);
  analogWrite(rightMotorPWM, 0);

  // Wait for button press to start calibration
  if (digitalRead(buttonPin) == HIGH) {
    while (digitalRead(buttonPin) == HIGH) {
      continue;
    }
    delay(200); // debounce
    robotState = CALIBRATING;
  }
}

// ----------------------------
// CALIBRATION STATE
// ----------------------------
void calibrateState() {
  digitalWrite(ledCalibrate, HIGH);


  while (digitalRead(buttonPin) == LOW) { // Calibrate till button press
    for (uint8_t i = 0; i < 5; i++) {
      int16_t value = analogRead(sensorPins[i]);
      if ((value < sensorMin[i]) && (value != 0)) sensorMin[i] = value;
      if (value > sensorMax[i]) sensorMax[i] = value;
    }
    delay(200);
  }
  delay(200);
  

  digitalWrite(ledCalibrate, LOW);

  // Wait for button press to start running
  while (digitalRead(buttonPin) == LOW) {
    continue;
  }
  while (digitalRead(buttonPin) == HIGH) {
    continue;
  }
  delay(200);
  robotState = RUNNING;
  digitalWrite(ledRunning, HIGH);
}

// ----------------------------
// RUNNING STATE
// ----------------------------
void runningState() {
  delay(50);
  readSensors();
  uint8_t position = calculateLinePosition();  // 0 to 4 (center = 2)
  position = constrain(position, 0, 4);
  int8_t error = 2 - position;
  //integral += error;
  //int8_t derivative = (error - lastError);
  //lastError = error;


  int8_t correction = (Kp * error); //Ki * integral + +  Kd * derivative

  if (correction > baseSpeed) {
    correction = baseSpeed;
  }

  if ((-correction) > baseSpeed) {
    correction = -baseSpeed;
  }

  uint8_t leftSpeed = baseSpeed + correction;
  uint8_t rightSpeed = baseSpeed - (correction);

if (leftSpeed>maxSpeed) {
  leftSpeed = maxSpeed;
}
if (rightSpeed>maxSpeed) {
  rightSpeed = maxSpeed;
}
  Serial.printf("Left motor: %d, Right motor: %d", leftSpeed, rightSpeed);
  for (int i = 0; i < 5; i++) {
    Serial.printf(", Sensor %d: %d", i, sensorValues[i]);
  }
  Serial.println();
  Serial.flush();


  analogWrite(leftMotorPWM, leftSpeed);
  analogWrite(rightMotorPWM, rightSpeed);

  // Stop if button pressed again
  if (digitalRead(buttonPin) == HIGH) {
    while (digitalRead(buttonPin) == HIGH) {
      continue;
    }
    delay(200);
    robotState = IDLE;
    digitalWrite(ledRunning, LOW);
  }
}

// ----------------------------
// Read sensors (normalized 0–10)
// ----------------------------
void readSensors() {
  for (uint8_t i = 0; i < 5; i++) {
    uint16_t value = analogRead(sensorPins[i]);
    uint8_t minV = ((sensorMin[i] - 10));
    uint16_t maxV = ((sensorMax[i] + 10));
    uint8_t value_mapped = map(value, minV, maxV, 10, 0); // dark line → higher value
    value_mapped = constrain(value_mapped, 0, 10);
    sensorValues[i] = value_mapped;
  }
}

// ----------------------------
// Compute weighted average position
// ----------------------------
uint8_t calculateLinePosition() {
  uint8_t sumValues = 0;
  uint8_t sumWeighted = 0;
  for (uint8_t i = 0; i < 5; i++) {
    sumValues += sensorValues[i];
    sumWeighted += sensorValues[i] * i;
  }
  if (sumValues == 0) return 2; // default center
  return ((sumWeighted) / sumValues);
}