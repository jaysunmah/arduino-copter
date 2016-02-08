int ch1 = 8;
int ch2 = 9;
int ch3 = 10;
int ch4 = 11;
int ch5 = 12;
int ch6 = 14;
int val_ch1,val_ch2,val_ch3,val_ch4,val_ch5,val_ch6;
unsigned long current_time;

void setup() {
  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);
  pinMode(ch3, INPUT);
  pinMode(ch4, INPUT);
  pinMode(ch5, INPUT);
  pinMode(ch6, INPUT);

  Serial.begin(9600);
  current_time = micros();
}

void loop() {
  Serial.print(micros() - current_time);
  current_time = micros();
  print_signals();

}

void print_signals() {
  val_ch1 = pulseIn(ch1, HIGH);

  
  Serial.print("ch1: ");
  Serial.print(val_ch1);
  Serial.print(" ch2: ");
  Serial.print(pulseIn(ch2, HIGH, 50000));
  Serial.print(" ch3: ");
  Serial.print(pulseIn(ch3, HIGH,50000));
  Serial.print(" ch4: ");
  Serial.print(pulseIn(ch4, HIGH, 50000));
  Serial.print(" ch5: ");
  Serial.print(pulseIn(ch5, HIGH, 50000));
//  Serial.print(" ch6: ");
//  Serial.print(pulseIn(ch6, HIGH, 50000));  
  Serial.println();
}

