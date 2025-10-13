void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(7, INPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(A4, INPUT);
  pinMode(A3, INPUT);
  pinMode(A2, INPUT);
  pinMode(A1, INPUT);
  pinMode(A0, INPUT);
}


void loop() {
  // put your main code here, to run repeatedly:

  bool value_mot = false;
  if (digitalRead(7)) {
    while (digitalRead(7)) {
      continue;
    }
    value_mot = !(value_mot);
    digitalWrite(8, !(digitalRead(8)));
  }
  float pin_5 = analogRead(A4);
  Serial.println("Pin 5: ");
  Serial.println(pin_5);
  float pin_4 = analogRead(A3);
  Serial.println("Pin 4: ");
  Serial.println(pin_4);
  float pin_3 = analogRead(A2);
  Serial.println("Pin 3: ");
  Serial.println(pin_3);
  float pin_2 = analogRead(A1);
  Serial.println("Pin 2: ");
  Serial.println(pin_2);
  float pin_1 = analogRead(A0);
  Serial.println("Pin 1: ");
  Serial.println(pin_1);
  delay(500);
  if (value_mot) {
    analogWrite(6, 30);
    analogWrite(5, 30);
  } else {
    analogWrite(6, 0);
    analogWrite(5, 0);
  }
}
