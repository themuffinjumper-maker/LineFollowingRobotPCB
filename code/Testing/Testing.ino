void setup() {
  // put your setup code here, to run once:
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(7, INPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(7)) {
    digitalWrite(8, !digitalRead(8));
    while(digitalRead(7)) {
      continue;
    }
  }
}
