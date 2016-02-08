bool flag = false;

void setup() {
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
}

void loop() {
  if (flag) {
    digitalWrite(0,HIGH);
    digitalWrite(1, LOW);
    flag = false;
  }
  else {
    digitalWrite(0, LOW);
    digitalWrite(1, HIGH);
    flag = true;
  }
  delay(1000);
}
