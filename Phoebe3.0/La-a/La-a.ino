#define i1 = 112
#define i2 = 122
#define rLED = 150

int val1, val2

void setup() {
  pinMode(i1, INPUT);
  pinMode(i2, INPUT);
  pinMode(rLED, OUTPUT);
}

void loop() {
  digitalWrite(rLED, HIGH);
  
  val1 = digitalRead(i1);
  val2 = digitalRead(i2);

  if (val1 == 0 && val2 == 0) {
    //turned off
  } else if (val1 == 0 && val2 == 1) {
    //armed, manual control
  } else if (val1 == 1 && val2 == 0) {
    //armed, manual control
  } else {
    //do nothing for now :)
  }

}
