//Declaring Variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, start;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long zero_timer, timer_1, timer_2, timer_3, timer_4, current_time;

int ch5;
int modified = 0;

void setup() {
  Serial.begin(9600);

  pinMode(3, OUTPUT);
  pinMode(12, INPUT); //Channel 5

  DDRD |= B11110000;                                 //Configure digital poort 4, 5, 6 and 7 as output
  //  DDRB |= B00010000;                                 //Configure digital poort 12 as output
  //Arduino Uno pins default to inputs, so they don't need to be explicitly declared as inputs

  PCICR |= (1 << PCIE0);                             // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);                           // set PCINT0 (digital input 8) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT1);                           // set PCINT1 (digital input 9)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT2);                           // set PCINT2 (digital input 10)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT3);                           // set PCINT3 (digital input 11)to trigger an interrupt on state change

  //  ///Wait until the receiver is active and the throtle is set to the lower position.
  //  while(receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400){
  //
  //    start ++;                                        //While waiting increment start whith every loop.
  //    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
  //    PORTD |= B11110000;                              //Set digital poort 4, 5, 6 and 7 high.
  //    delayMicroseconds(1000);                         //Wait 1000us (We can use delayMicroseconds because the receiver interrupt routine is not used).
  //    PORTD &= B00001111;                              //Set digital poort 4, 5, 6 and 7 low.
  //    delay(3);                                        //Wait 3 milliseconds before the next loop.
  //    if(start == 125){                                //Every 125 loops (500ms).
  //      Serial.print("this went here");
  //      Serial.print(receiver_input_channel_3);
  //      Serial.println();
  //      digitalWrite(12, !digitalRead(12));            //Change the led status.
  //      start = 0;                                     //Start again at 0.
  //    }
  //  }
  start = 0;
  //  digitalWrite(12, LOW);                             //Turn off the led.
  zero_timer = micros();                             //Set the zero_timer for the first loop.

}

void loop() {
  while (zero_timer + 4000 > micros());                      //Start the pulse after 4000 micro seconds.
  //  print_signals();
  zero_timer = micros();                                     //Reset the zero timer.
  PORTD |= B11110000;
  timer_channel_1 = receiver_input_channel_1 + zero_timer;   //Calculate the time when digital port 8 is set low.
  timer_channel_2 = receiver_input_channel_2 + zero_timer;   //Calculate the time when digital port 9 is set low.
  timer_channel_3 = receiver_input_channel_3 + zero_timer;   //Calculate the time when digital port 10 is set low.
  timer_channel_4 = receiver_input_channel_4 + zero_timer;   //Calculate the time when digital port 11 is set low.

  while (PORTD >= 16) {                                      //Execute the loop until digital port 8 til 11 is low.
    esc_loop_timer = micros();                               //Check the current time.
    if (timer_channel_1 <= esc_loop_timer)PORTD &= B11101111; //When the delay time is expired, digital port 8 is set low.
    if (timer_channel_2 <= esc_loop_timer)PORTD &= B11011111; //When the delay time is expired, digital port 9 is set low.
    if (timer_channel_3 <= esc_loop_timer)PORTD &= B10111111; //When the delay time is expired, digital port 10 is set low.
    if (timer_channel_4 <= esc_loop_timer)PORTD &= B01111111; //When the delay time is expired, digital port 11 is set low.
  }
  ch5 = pulseIn(12, HIGH, 25000);
  //  Serial.println(ch5);
  if (ch5 > 1400) { //Self Level ON
    if (modified == 0) {
      analogWrite(3, 200);
      Serial.print(ch5);
      modified = 1;
    }
  }
  else {
    if (modified == 1) {
      analogWrite(3, 125);
      modified = 0;
    }

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
    //    tempVar = receiver_input_channel_1;
    //    receiver_input_channel_1 = map(tempVar, 1152, 1880, 1000, 2000);
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

    //    tempVar = receiver_input_channel_2;
    //    receiver_input_channel_2 = map(tempVar, 1148, 1880, 2000, 1000);
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
    //  1152 - 1808
    //    tempVar = receiver_input_channel_3;
    //    receiver_input_channel_3 = map(tempVar, 1152, 1808, 1000, 2000);
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
    //    tempVar = receiver_input_channel_4;
    //    receiver_input_channel_4 = map(tempVar, 1116, 1896, 1000, 2000);
  }
}

void print_signals() {
  Serial.print("Roll:");
  if (receiver_input_channel_1 - 1480 < 0)Serial.print("<<<");
  else if (receiver_input_channel_1 - 1520 > 0)Serial.print(">>>");
  else Serial.print("--+--");
  Serial.print(receiver_input_channel_1);

  Serial.print(" Nick:");
  if (receiver_input_channel_2 - 1480 < 0)Serial.print("VVV");
  else if (receiver_input_channel_2 - 1520 > 0)Serial.print("^^^");
  else Serial.print("--+--");
  Serial.print(receiver_input_channel_2);

  Serial.print(" Gas:");
  if (receiver_input_channel_3 - 1480 < 0)Serial.print("^^^");
  else if (receiver_input_channel_3 - 1520 > 0)Serial.print("VVV");
  else Serial.print("--+--");
  Serial.print(receiver_input_channel_3);

  Serial.print(" Yaw:");
  if (receiver_input_channel_4 - 1480 < 0)Serial.print("<<<");
  else if (receiver_input_channel_4 - 1520 > 0)Serial.print(">>>");
  else Serial.print("--+--");
  Serial.print(receiver_input_channel_4);

  Serial.println();
}
