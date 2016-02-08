///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.5
///////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20.h>

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_L3GD20 gyro;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int sensitivity = 10;
int maxDTheta = 20;

float pid_p_gain_roll = 0.3;               //Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0.01;              //Gain setting for the roll I-controller (0.3)
float pid_d_gain_roll = 0.5;                //Gain setting for the roll D-controller (15) my max was 28
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4, tempVar;
int throttle, battery_voltage;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
int cal_int, start;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
byte highByte, lowByte;
int ch5;
float x, y, z;
float x_cal, y_cal, z_cal;

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

float pid_i_mem_hover, pid_hover_setpoint, hover_input, pid_output_hover, pid_last_hover_d_error;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup() {

  pinMode(12, INPUT); //Channel 5
  pinMode(14, OUTPUT); //LEDS
  pinMode(15, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  digitalWrite(14, HIGH);
  Serial.begin(9600);
  Serial.println("Accelerometer Test"); Serial.println("");

  /* Initialise the sensor */
  if (!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1);
  }
  /* Display some basic information on this sensor */
  displaySensorDetails();
  Serial.begin(9600);

  DDRD |= B11110000;                                           //Configure digital poort 4, 5, 6 and 7 as output.
  // Try to initialise and warn if we couldn't detect the chip
  if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
    //  if (!gyro.begin(gyro.L3DS20_RANGE_500DPS))
    //if (!gyro.begin(gyro.L3DS20_RANGE_2000DPS))
  {
    Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
    while (1);
  }

  delay(250);                                                  //Give the gyro time to start.

  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).

  Serial.print("Starting calibration...");
  for (cal_int = 0; cal_int < 2000 ; cal_int ++) {             //Take 2000 readings for calibration.
    genData();                                                 //Read the gyro output.
    gyro_roll_cal += gyro_roll;                                //Ad roll value to gyro_roll_cal.
    gyro_pitch_cal += gyro_pitch;                              //Ad pitch value to gyro_pitch_cal.
    gyro_yaw_cal += gyro_yaw;                                  //Ad yaw value to gyro_yaw_cal.

    x_cal += x;
    y_cal += y;
    z_cal += z;

    if (cal_int % 100 == 0)Serial.print(".");                  //Print a dot every 100 readings

    if (cal_int == 700) {
      digitalWrite(15, HIGH);
      digitalWrite(14, LOW);
    }
    else if (cal_int == 1400) {
      digitalWrite(16, HIGH);
      digitalWrite(15, LOW);
    }
    else if (cal_int == 1900) {
      digitalWrite(17, HIGH);
      digitalWrite(16, LOW);
    }

    PORTD |= B11110000;                                        //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                   //Wait 1000us.
    PORTD &= B00001111;                                        //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                  //Wait 3 milliseconds before the next loop.
  }

  Serial.println(" done!");                          //2000 measures are done!
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  gyro_roll_cal /= 2000;                                       //Divide the roll total by 2000.
  gyro_pitch_cal /= 2000;                                      //Divide the pitch total by 2000.
  gyro_yaw_cal /= 2000;                                        //Divide the yaw total by 2000.

  x_cal /= 2000;
  y_cal /= 2000;
  z_cal /= 2000;


  PCICR |= (1 << PCIE0);                                       //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                     //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                     //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                     //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                     //Set PCINT3 (digital input 11)to trigger an interrupt on state change.

  //Wait until the receiver is active and the throtle is set to the lower position.
  while (receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400) {
    start ++;                                        //While waiting increment start whith every loop.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
    PORTD |= B11110000;                              //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                         //Wait 1000us (We can use delayMicroseconds because the receiver interrupt routine is not used).
    PORTD &= B00001111;                              //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                        //Wait 3 milliseconds before the next loop.
  }
  start = 0;                                                   //Set start back to 0.
  digitalWrite(17, LOW);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  //Let's get the current gyro data and scale it to degrees per second for the pid calculations.
  genData();



  gyro_roll_input = gyro_roll;
  gyro_pitch_input = gyro_pitch;
  gyro_yaw_input = gyro_yaw;
  
  x = x * sensitivity;
  y = y * -1 * sensitivity;
  if(x > maxDTheta)x = maxDTheta;
  else if(x < maxDTheta * -1)x = maxDTheta * -1;

  Serial.print("X: "); Serial.print(x * -1); Serial.print("  ");
  Serial.print("Y: "); Serial.print(y * -1); Serial.print("  ");
  Serial.print("Z: "); Serial.print(z); Serial.print("  "); Serial.print("m/s^2 ");

  Serial.print(" Pitch: "); Serial.print(gyro_pitch);   Serial.print(" ");
  Serial.print("Roll: "); Serial.print(gyro_roll);   Serial.print(" ");
  Serial.print("Yaw: "); Serial.println(gyro_yaw); Serial.print(" "); Serial.println();

  //For starting the motors: throttle low and yaw left (step 1).
  if (receiver_input_channel_3 < 1070 && receiver_input_channel_4 < 1070)start = 1;
  
  //When yaw stick is back in the center position start the motors (step 2).
  if (start == 1 && receiver_input_channel_3 < 1070 && receiver_input_channel_4 > 1450) {
    start = 2;
    //Reset the pid controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  //Stopping the motors: throttle low and yaw right.
  if (start == 2 && receiver_input_channel_3 < 1070 && receiver_input_channel_4 > 1950)start = 0;

  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_1 > 1516) {
    pid_roll_setpoint = (receiver_input_channel_1 - 1516) / 6.0;
    digitalWrite(14, HIGH);
    //    Serial.print(receiver_input_channel_1);
    //    Serial.print(" 1.1");
    //    Serial.println();
  }
  else if (receiver_input_channel_1 < 1476) {
    pid_roll_setpoint = (receiver_input_channel_1 - 1476) / 6.0;
    digitalWrite(14, HIGH);
    //    Serial.print(receiver_input_channel_1);
    //    Serial.print(" 1.2");
    //    Serial.println();
  }
  else {
    digitalWrite(14, LOW);
  }

  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_2 > 1516) {
    pid_pitch_setpoint = (receiver_input_channel_2 - 1516) / 6.0;
    digitalWrite(15, HIGH);
    //    Serial.print(receiver_input_channel_2);
    //    Serial.print(" 2.1");
    //    Serial.println();
  }
  else if (receiver_input_channel_2 < 1476) {
    pid_pitch_setpoint = (receiver_input_channel_2 - 1476) / 6.0;
    digitalWrite(15, HIGH);
    //    Serial.print(receiver_input_channel_2);
    //    Serial.print(" 2.2");
    //    Serial.println();
  }
  else {
    digitalWrite(15, LOW);
    pid_pitch_setpoint = x / 3.0;
  }

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_3 > 1070) { //Do not yaw when turning off the motors.
    if (receiver_input_channel_4 > 1516) {
      pid_yaw_setpoint = (receiver_input_channel_4 - 1516) / 6.0;
      digitalWrite(16, HIGH);
    }
    else if (receiver_input_channel_4 < 1476) {
      pid_yaw_setpoint = (receiver_input_channel_4 - 1476) / 6.0;
      digitalWrite(16, HIGH);
    }
    else {
      digitalWrite(16, LOW);
    }
  }




  //PID inputs are known. So we can calculate the pid output.
  calculate_pid();

  throttle = receiver_input_channel_3;                                      //We need the throttle signal as a base signal.

  ch5 = pulseIn(12, HIGH, 25000);
  //  Serial.println(ch5);
  if (ch5 > 1400) { //Self Level ON
    digitalWrite(17, HIGH);
  }
  else {
    digitalWrite(17, LOW);
  }


  Serial.println(pid_output_yaw);
  
  if (start == 2) {                                                         //The motors are started.
    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

    if (esc_1 < 1200) esc_1 = 1100;                                         //Keep the motors running.
    if (esc_2 < 1200) esc_2 = 1100;                                         //Keep the motors running.
    if (esc_3 < 1200) esc_3 = 1100;                                         //Keep the motors running.
    if (esc_4 < 1200) esc_4 = 1100;                                         //Keep the motors running.

    if (esc_1 > 2000)esc_1 = 2000;                                          //Limit the esc-1 pulse to 2000us.
    if (esc_2 > 2000)esc_2 = 2000;                                          //Limit the esc-2 pulse to 2000us.
    if (esc_3 > 2000)esc_3 = 2000;                                          //Limit the esc-3 pulse to 2000us.
    if (esc_4 > 2000)esc_4 = 2000;                                          //Limit the esc-4 pulse to 2000us.
  }

  else {
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }

  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while (micros() - loop_timer < 4000);                                     //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.

  while (PORTD >= 16) {                                                     //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if (timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;               //Set digital output 4 to low if the time is expired.
    if (timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;               //Set digital output 5 to low if the time is expired.
    if (timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;               //Set digital output 6 to low if the time is expired.
    if (timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;               //Set digital output 7 to low if the time is expired.
  }
//    Serial.print(esc_1);
//    Serial.print(" ");
//    Serial.print(esc_2);
//    Serial.print(" ");
//    Serial.print(esc_3);
//    Serial.print(" ");
//    Serial.print(esc_4);
//    Serial.println();

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This routine is called every time input 8, 9, 10 or 11 changed state
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
    //    receiver_input_channel_1 = map(tempVar, 1072, 1956, 1300, 1700);

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
    //    receiver_input_channel_2 = map(tempVar, 1120, 1896, 1300, 1700);

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
    tempVar = receiver_input_channel_3;
    receiver_input_channel_3 = map(tempVar, 1072, 1860, 1000, 2000);
    if (receiver_input_channel_3 < 1000) {
      receiver_input_channel_3 = 1000;
    }
    else if (receiver_input_channel_3 > 2000) {
      receiver_input_channel_3 = 2000;
    }
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for reading the gyro
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void genData() {
  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);
  x = event.acceleration.x;
  y = event.acceleration.y;
  z = event.acceleration.z;

  gyro.read();
  gyro_roll = (int)gyro.data.x;
  gyro_pitch = (int)gyro.data.y * -1;
  gyro_yaw = (int)gyro.data.z * -1;

  if (cal_int == 2000) {
    gyro_pitch -= gyro_pitch_cal;
    gyro_roll -= gyro_roll_cal;
    gyro_yaw -= gyro_yaw_cal;

    gyro_pitch = (int)gyro_pitch;
    gyro_roll = (int)gyro_roll;
    gyro_yaw = (int)gyro_yaw;

    x -= x_cal;
    y -= y_cal;
    z -= z_cal;
    if (abs(x) < 0.08)x = 0;
    if (abs(y) < 0.08)y = 0;
    if (abs(z) < 0.08)z = 0;
  }

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The PID controllers are explained in part 5 of the YMFC-3D video session:
//www.youtube.com/watch?v=JBvnB0279-Q

void calculate_pid() {
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}


