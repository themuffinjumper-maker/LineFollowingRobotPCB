const uint8_t sensorPins[5] = {A0, A1, A2, A3, A4};
const uint8_t leftMotorPWM = 6;
const uint8_t rightMotorPWM = 5;
const uint8_t buttonPin = 7;
const uint8_t ledCalibrate = 9;
const uint8_t ledRunning = 8;



uint16_t sensorMin[5];
uint16_t sensorMax[5];

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

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(buttonPin) == HIGH) {
    while (digitalRead(buttonPin) == HIGH) {
      continue;
    }
    delay(200); // debounce  
    calibrateState();
  }
}

void calibrateState() {
  digitalWrite(ledCalibrate, HIGH);
  Serial.println("Calibrating... Move sensors over line and background");

  while (digitalRead(buttonPin) == LOW) { // Calibrate till button press
    for (uint8_t i = 0; i < 5; i++) {
      int16_t value = analogRead(sensorPins[i]);
      if ((value < sensorMin[i]) && (value != 0)) sensorMin[i] = value;
      if (value > sensorMax[i]) sensorMax[i] = value;
    }
    delay(200);
  }
  delay(200);

  if (digitalRead(buttonPin) == HIGH) {
    while (digitalRead(buttonPin) == HIGH) {
      continue;
    }
    delay(200); // debounce  
    calibrateState();
  }
  Serial.print("White level: "); 
  for (uint8_t i = 0; i < 5; i++) {
    Serial.println(i+1);
    Serial.println(sensorMax[i]);
  }
  
  Serial.print("Black level: "); 
  for (uint8_t i = 0; i < 5; i++) {
    Serial.println(i+1);
    Serial.println(sensorMin[i]);
  }

  digitalWrite(ledCalibrate, LOW);
  Serial.println("Calibration done.");
}