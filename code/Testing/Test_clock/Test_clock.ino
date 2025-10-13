void setup() {
  // put your setup code here, to run once:
  pinMode(9, OUTPUT);

  Serial.begin(9600);


}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(9, !digitalRead(9));

  Serial.println("Check Check");
  Serial.println("We Love it");

  delay(10000);

  if (Serial.available()) {
    String got = Serial.readStringUntil('\n');
    Serial.print("Got:");
    Serial.print(got);
  }
}
