// ====== PID Line Following Robot with Auto Calibration ======
// 5 TCRT5000 analog sensors, 2 PWM motors

// Motor pins
const int LEFT_MOTOR_PWM = 5;
const int RIGHT_MOTOR_PWM = 6;


// Sensor pins
const int sensorPins[5] = {A0, A1, A2, A3, A4};

// PID constants (tune for your robot)
float Kp = 25.0;
float Ki = 0.0;
float Kd = 8.0;

// Variables for PID
float error = 0, lastError = 0;
float integral = 0;
float derivative = 0;
float output = 0;

// Motor speed limits
int baseSpeed = 120;
int maxSpeed  = 255;

// Calibration values
int whiteLevel = 0;
int blackLevel = 1023;

void setup() {
  Serial.begin(9600);

  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);


  Serial.println("=== Calibrating sensors ===");
  calibrateSensors();
  Serial.println("Calibration done!");
  Serial.print("White level: "); Serial.println(whiteLevel);
  Serial.print("Black level: "); Serial.println(blackLevel);
  delay(1000);
}

void loop() {
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

  // Compute line position (weighted average)
  float weightedSum = 0;
  float total = 0;
  for (int i = 0; i < 5; i++) {
    float lineSignal = 1.0 - normalized[i]; // black = 1, white = 0
    weightedSum += weights[i] * lineSignal;
    total += lineSignal;
  }

  float position = 0;
  if (total != 0)
    position = weightedSum / total;

  // PID calculations
  error = position; // target is 0 (center)
  integral += error;
  derivative = error - lastError;
  output = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  // Adjust motor speeds
  int leftSpeed  = constrain(baseSpeed + output, 0, maxSpeed);
  int rightSpeed = constrain(baseSpeed - output, 0, maxSpeed);

  analogWrite(LEFT_MOTOR_PWM, leftSpeed);
  analogWrite(RIGHT_MOTOR_PWM, rightSpeed);

  // Debug info
  Serial.print("Pos: "); Serial.print(position);
  Serial.print("  Error: "); Serial.print(error);
  Serial.print("  L: "); Serial.print(leftSpeed);
  Serial.print("  R: "); Serial.println(rightSpeed);

  delay(10);
}

// === Calibration Function ===
void calibrateSensors() {
  unsigned long startTime = millis();
  unsigned long calibrationTime = 3000; // 3 seconds

  while (millis() - startTime < calibrationTime) {
    for (int i = 0; i < 5; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < blackLevel) blackLevel = val;
      if (val > whiteLevel) whiteLevel = val;
    }

    // Optional: spin motors slightly during calibration
    analogWrite(LEFT_MOTOR_PWM, 100);
    analogWrite(RIGHT_MOTOR_PWM, 150);
  }

  // Stop motors after calibration
  analogWrite(LEFT_MOTOR_PWM, 0);
  analogWrite(RIGHT_MOTOR_PWM, 0);
}
