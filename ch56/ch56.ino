int ch5;
int ch6;

void setup() {
 pinMode(10, INPUT);
 pinMode(9, INPUT);

 Serial.begin(9600);

}

void loop() {
  ch5 = pulseIn(10, HIGH);
  ch6 = pulseIn(9, HIGH);

  Serial.print("ch5: ");
  Serial.print(ch5);
  Serial.print(" ch6: ");
  Serial.print(ch6);
  Serial.println();
  delay(100);
}
