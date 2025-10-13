void setup() {
  // put your setup code here, to run once:
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
  static bool value_mot = false;
  if (digitalRead(7)) {
    while (digitalRead(7)) {
      continue;
    }
    value_mot = !(value_mot);
    digitalWrite(8, !(digitalRead(8)));
  }
  if (value_mot) {
    analogWrite(6, 1);
    analogWrite(5, 50);
  } else {
    analogWrite(6, 0);
    analogWrite(5, 0);
  }

}
