//Declaring Variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
float receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
float counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, start;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long zero_timer, timer_1, timer_2, timer_3, timer_4, current_time;
float throttle, pitch, roll, yaw;

int ch5, ch6;
float hoverOutput, hoverError, hoverP, hoverI, hoverD, total, prevError;
float maxHoverOutput = 1800;
float minHoverOutput = 1200;
int arm = 0;
int hoverSetPoint = 50;

float hoverKp = 2.5; //2.5 is this downward force??
float hoverKi = 0.003; //0.003 //try 0.009
float hoverKd = 170; // 120

float deltaT, hoverPrevTime, hoverCurrentTime, currentHoverKi;
int justStartedHover = 0;
int listenToReceiver = 1;

int startTime, currentTime, pressedDown;
long duration, distance, prevDistance, avgDistance, sumDistance;


#define trigPin 14
#define echoPin 15
#define i1 2 //status indicator 1
#define i2 3 //status indicator 2

void setup() {
  Serial.begin(9600);

  pinMode(i1, OUTPUT);
  pinMode(i2, OUTPUT);

  pinMode(12, INPUT);  //Channel 5 #2 modes(1000 2000) ^ = off V = on
  pinMode(13, INPUT); //Channel 6 #3 modes (1000 1500 2000) ^ = off V = on

  DDRD |= B11110000;                                 //Configure digital poort 4, 5, 6 and 7 as output

  //Arduino Uno pins default to inputs, so they don't need to be explicitly declared as inputs

  PCICR |= (1 << PCIE0);                             // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);                           // set PCINT0 (digital input 8) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT1);                           // set PCINT1 (digital input 9)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT2);                           // set PCINT2 (digital input 10)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT3);                           // set PCINT3 (digital input 11)to trigger an interrupt on state change

  start = 0;
  zero_timer = micros();                             //Set the zero_timer for the first loop.

}

void loop() {

  //sets controls such that jitters are ignored
  if (receiver_input_channel_1 > 1470 && receiver_input_channel_1 < 1530) { //roll is at rest
    roll = 1500; //1000 <-- ---> 2000
  } else {
    roll = receiver_input_channel_1;
  }
  if (receiver_input_channel_2 > 1470 && receiver_input_channel_2 < 1530) { //pitch is at rest
    pitch = 1500; //1000 ^ V 2000
  } else {
    pitch = receiver_input_channel_2;
  }
  if (receiver_input_channel_4 > 1470 && receiver_input_channel_4 < 1530) { //yaw is at rest
    yaw = 1496; //2000<---- ----> 1000
//      yaw = 1500;
  } else {
    yaw = receiver_input_channel_4;
  }

//  if (arm == 0) {
//    digitalWrite(i1, LOW);
//    digitalWrite(i2, LOW);
//    //insert code here to turn OFF all of the LEDs
//
//    if ((receiver_input_channel_3 < 1100) && (receiver_input_channel_4 < 1100)) { //thr off and yaw right
//      if (pressedDown == 0) {
//        startTime = millis();
//        currentTime = millis();
//        pressedDown = 1;
//      } else {
//        pressedDown = 1;
//        currentTime = millis();
//      }
//    } else {
//      pressedDown = 0;
//    }
//
//    if (pressedDown == 1) {
//      roll = 1950;
//      if (currentTime - startTime > 1200) {
//        arm = 1;
//        startTime = 0;
//        currentTime = 0;
//      }
//    }
//  } else {
//    digitalWrite(i1, HIGH); // activated, 01 (armed / manual)
//    digitalWrite(i1, LOW); 
//    //insert code here to turn ON all of the inner LEDs
//    if ((receiver_input_channel_3 < 1100) && (receiver_input_channel_4 > 1800)) {
//      if (pressedDown == 0) {
//        startTime = millis();
//        currentTime = millis();
//        pressedDown = 1;
//      } else {
//        pressedDown = 1;
//        currentTime = millis();
//      }
//    } else {
//      pressedDown = 0;
//    }
//    if (pressedDown == 1) {
//      if (currentTime - startTime > 1200) {
//        arm = 0;
//        startTime = 0;
//        currentTime = 0;
//      }
//    }
//  }

  throttle = receiver_input_channel_3;
 

  ch6 = pulseIn(12, HIGH, 35000);
  ch5 = pulseIn(13, HIGH, 35000);
  printRollPitchYaw();
  
//  if (arm == 1) {
//    if (ch5 > 1500) { //10, Automatic (blinking lights)
//      digitalWrite(2, LOW);
//      digitalWrite(3, HIGH);
//    } else {
//    digitalWrite(2, HIGH); // 01 armed, but not automatic 
//    digitalWrite(3, LOW);
//    }
//  }



  while (zero_timer + 4000 > micros());                       //Start the pulse after 4000 micro seconds.
  zero_timer = micros();                                      //Reset the zero timer.
  PORTD |= B11110000;
  timer_channel_1 = roll + zero_timer;                        //Calculate the time when digital port 8 is set low.
  timer_channel_2 = pitch + zero_timer;                       //Calculate the time when digital port 9 is set low.
  timer_channel_3 = throttle + zero_timer;                    //Calculate the time when digital port 10 is set low.
  timer_channel_4 = yaw + zero_timer;                         //Calculate the time when digital port 11 is set low.

  while (PORTD >= 16) {                                       //Execute the loop until digital port 8 til 11 is low.
    esc_loop_timer = micros();                                //Check the current time.
    if (timer_channel_1 <= esc_loop_timer)PORTD &= B11101111; //When the delay time is expired, digital port 8 is set low.
    if (timer_channel_2 <= esc_loop_timer)PORTD &= B11011111; //When the delay time is expired, digital port 9 is set low.
    if (timer_channel_3 <= esc_loop_timer)PORTD &= B10111111; //When the delay time is expired, digital port 10 is set low.
    if (timer_channel_4 <= esc_loop_timer)PORTD &= B01111111; //When the delay time is expired, digital port 11 is set low.
  }


}

ISR(PCINT0_vect) {
  current_time = micros();
  //Channel 1=========================================
  if (PINB & B00000001) {                                      //Is input 8 high?
    if (last_channel_1 == 0) {                                 //Input 8 changed from 0 to 1
      last_channel_1 = 1;                                      //Remember current input state
      timer_1 = current_time;                                  //Set timer_1 to current_time
    }
  }
  else if (last_channel_1 == 1) {                              //Input 8 is not high and changed from 1 to 0
    last_channel_1 = 0;                                        //Remember current input state
    receiver_input_channel_1 = current_time - timer_1;         //Channel 1 is current_time - timer_1
  }
  //Channel 2=========================================
  if (PINB & B00000010 ) {                                     //Is input 9 high?
    if (last_channel_2 == 0) {                                 //Input 9 changed from 0 to 1
      last_channel_2 = 1;                                      //Remember current input state
      timer_2 = current_time;                                  //Set timer_2 to current_time
    }
  }
  else if (last_channel_2 == 1) {                              //Input 9 is not high and changed from 1 to 0
    last_channel_2 = 0;                                        //Remember current input state
    receiver_input_channel_2 = current_time - timer_2;         //Channel 2 is current_time - timer_2
  }

  //Channel 3=========================================
  if (PINB & B00000100 ) {                                     //Is input 10 high?
    if (last_channel_3 == 0) {                                 //Input 10 changed from 0 to 1
      last_channel_3 = 1;                                      //Remember current input state
      timer_3 = current_time;                                  //Set timer_3 to current_time
    }
  }
  else if (last_channel_3 == 1) {                              //Input 10 is not high and changed from 1 to 0
    last_channel_3 = 0;                                        //Remember current input state
    receiver_input_channel_3 = current_time - timer_3;         //Channel 3 is current_time - timer_3
  }
  //Channel 4=========================================
  if (PINB & B00001000 ) {                                     //Is input 11 high?
    if (last_channel_4 == 0) {                                 //Input 11 changed from 0 to 1
      last_channel_4 = 1;                                      //Remember current input state
      timer_4 = current_time;                                  //Set timer_4 to current_time
    }
  }
  else if (last_channel_4 == 1) {                              //Input 11 is not high and changed from 1 to 0
    last_channel_4 = 0;                                        //Remember current input state
    receiver_input_channel_4 = current_time - timer_4;         //Channel 4 is current_time - timer_4
  }
}

//note to self: possibly change hoverOutput to start at 1400?
void calculateHover() {
  // DeltaT is about 19-20 ms
  hoverError = hoverSetPoint - distance;
  hoverP = hoverError;
  hoverI = hoverI + hoverError * 19;
  hoverD = (hoverError - prevError) / 19;

  prevError = hoverError;
  hoverPrevTime = hoverCurrentTime;
  
  total = hoverP * hoverKp + hoverI * hoverKi + hoverD * hoverKd;

  if (total < 0) {
    total = 0;
  }
  else if (total > 500) {
    total = 500;
  }
  hoverOutput = 1300 + total;

  printPID(); //uncomment this for debugging PID

}

//updates distance variable, change the duration pulseIn
void updateDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 9000);
  
  if (duration == 0) { //timed out
     distance = prevDistance;
     Serial.println("fuck");
     digitalWrite(14, HIGH);
  } else {
    distance = (duration / 2) / 29.1;
    digitalWrite(14, LOW);
  }  
  
  if ((abs(distance - prevDistance) > 15 && prevDistance != 0)|| distance > 100) {
    distance = prevDistance;
  }
  prevDistance = distance;
}

//call these for debugging purposes

void printPID() {
  Serial.print("P: ");
  Serial.print(hoverP * hoverKp);
  Serial.print("  I: ");
  Serial.print(hoverI * hoverKi);
  Serial.print("  D: ");
  Serial.print(hoverD * hoverKd);
  Serial.print("  Total:");
  Serial.print(total);
  Serial.print("  Distance: ");
  Serial.print(distance);
  Serial.print("  Output: ");
  Serial.print(hoverOutput);
  Serial.println();

}


void printRollPitchYaw() {
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(" Pitch: ");
  Serial.print(pitch);
  Serial.print(" Thr: ");
  Serial.print(throttle);
  Serial.print(" Yaw: ");
  Serial.print(yaw);
  Serial.print(" Ch5: ");
  Serial.print(ch5);
  Serial.print(" Ch6: ");
  Serial.print(ch6);
  Serial.println();
}

