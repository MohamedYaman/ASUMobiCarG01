void setup() {
  Serial.begin(9600);
  pinMode(7, OUTPUT);
}

void loop() {
  int sensorValue = analogRead(A0);
  if (sensorValue > 50)
{
  digitalWrite(7, HIGH);
  Serial.println("Rumble on");
delay(1000);
}
else
{
  digitalWrite(7, LOW);
  Serial.println("Rumble off");
delay(100);
}
}
