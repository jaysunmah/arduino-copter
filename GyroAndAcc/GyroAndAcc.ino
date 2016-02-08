#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20.h>

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_L3GD20 gyro;

float gyro_pitch, gyro_roll, gyro_yaw;
float x, y, z;
float gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
float x_cal, y_cal, z_cal;
int cal_int;

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

void setup(void)
{
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

  // Try to initialise and warn if we couldn't detect the chip
  if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
//  if (!gyro.begin(gyro.L3DS20_RANGE_500DPS))
    //if (!gyro.begin(gyro.L3DS20_RANGE_2000DPS))
  {
    Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
    while (1);
  }
  Serial.print("Starting calibration...");
  for (cal_int = 0; cal_int < 2000 ; cal_int ++){              //Take 2000 readings for calibration.
    genData();                                                 //Read the gyro output.
    gyro_roll_cal += gyro_roll;                                //Ad roll value to gyro_roll_cal.
    gyro_pitch_cal += gyro_pitch;                              //Ad pitch value to gyro_pitch_cal.
    gyro_yaw_cal += gyro_yaw;                                  //Ad yaw value to gyro_yaw_cal.
    
    x_cal += x;
    y_cal += y;
    z_cal += z;
    
    if(cal_int%100 == 0)Serial.print(".");                     //Print a dot every 100 readings
    
    if(cal_int == 700) {
      digitalWrite(15, HIGH);
      digitalWrite(14, LOW);
    }
    else if(cal_int == 1400) {
      digitalWrite(16, HIGH);
      digitalWrite(15, LOW);
    }
    else if(cal_int == 1900) {
      digitalWrite(17, HIGH);
      digitalWrite(16, LOW);
    }
    
    PORTD |= B11110000;                                        //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                   //Wait 1000us.
    PORTD &= B00001111;                                        //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                  //Wait 3 milliseconds before the next loop.
  }
  
  Serial.println(" done!");                                     //2000 measures are done!
  gyro_roll_cal /= 2000;                                       //Divide the roll total by 2000.
  gyro_pitch_cal /= 2000;                                      //Divide the pitch total by 2000.
  gyro_yaw_cal /= 2000;                                        //Divide the yaw total by 2000.  

  x_cal /= 2000;
  y_cal /= 2000;
  z_cal /= 2000;

}

void loop(void)
{
  genData();
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: "); Serial.print(x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(z); Serial.print("  "); Serial.print("m/s^2 ");

  Serial.print(" X1: "); Serial.print(gyro_pitch);   Serial.print(" ");
  Serial.print("Y1: "); Serial.print(gyro_roll);   Serial.print(" ");
  Serial.print("Z1: "); Serial.println(gyro_yaw); Serial.print(" "); Serial.println();

  delay(100);
}

void genData() {
  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);
  x = event.acceleration.x;
  y = event.acceleration.y;
  z = event.acceleration.z;

  gyro.read();
  gyro_pitch = (int)gyro.data.y;
  gyro_roll = (int)gyro.data.x;
  gyro_yaw = (int)gyro.data.z;

  if(cal_int == 2000) {
    gyro_pitch -= gyro_pitch_cal;
    gyro_roll -= gyro_roll_cal;
    gyro_yaw -= gyro_yaw_cal;

    gyro_pitch = (int)gyro_pitch * -1;
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

