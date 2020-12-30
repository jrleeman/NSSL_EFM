void setup()
{
  pinMode(0, OUTPUT);
}

void loop() {
  digitalWrite(0, HIGH);
  delayMicroseconds(333); //333
  digitalWrite(0, LOW);
  delayMicroseconds(333);
}
