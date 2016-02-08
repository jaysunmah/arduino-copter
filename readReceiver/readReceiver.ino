int ch1;
int ch2;
int ch3;
int ch4;
int tempVal;

void setup() {
  // put your setup code here, to run once:
  pinMode(8,INPUT);
  pinMode(9,INPUT);
  pinMode(10,INPUT);
  pinMode(11,INPUT);

  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  ch1 = pulseIn(8, HIGH); //1802 - 1146
  tempVal = ch1;
//  ch1 = map(tempVal, 1146, 1840, 1000, 2000); 
  ch2 = pulseIn(9, HIGH);
  ch3 = pulseIn(10, HIGH);
  tempVal = ch3;
//  ch3 = map(tempVal, 1146, 1802, 1000, 2000);
  ch4 = pulseIn(11, HIGH);

  Serial.print("Ch1: ");
  Serial.print(ch1);
  Serial.print(" Ch2: ");
  Serial.print(ch2);
  Serial.print(" Ch3: ");
  Serial.print(ch3);
  Serial.print(" Ch4: ");
  Serial.print(ch4);
  Serial.println();
  delay(250);

}
