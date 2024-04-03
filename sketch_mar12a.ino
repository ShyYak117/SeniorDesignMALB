/*
    MPU6050 Triple Axis Gyroscope & Accelerometer. Pitch & Roll & Yaw Gyroscope Example.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-i-akcelerometr-mpu6050.html
    GIT: https://github.com/jarzebski/Arduino-MPU6050
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>
#include <MPU6050.h>
#include <MPU6050.h>
#include <ResponsiveAnalogRead.h>


#define RPWM 5
#define LPWM 6
#define STROKE_LENGTH      100    // PLEASE UPDATE THIS VALUE (in millimeter)
#define POTENTIOMETER_MAX  962   // PLEASE UPDATE THIS VALUE
#define POTENTIOMETER_MIN  34   // PLEASE UPDATE THIS VALUE
MPU6050 mpu;
const int FBA = A0;
ResponsiveAnalogRead analog(FBA, true);
// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

int command_location = 120;                                   // Commanded position
int actual_location = 0;                                      // Current position
int deadband =  3;  
int pot_read = 0;


void setup() 
{
  Serial.begin(115200);

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  // pinMode(FBA, INPUT);
  analogWrite(RPWM,0);
  analogWrite(LPWM,0);
}

void loop()
{
  analog.update();
  pot_read = analog.getValue();                  //read the actual position from the actuator potentiometer
  actual_location = map(pot_read, POTENTIOMETER_MIN, POTENTIOMETER_MAX, 0, STROKE_LENGTH);  //map(value, fromLow, fromHigh, toLow, toHigh)
  timer = millis();

  // Read normalized values
  // Vector norm = mpu.readNormalizeGyro();
  // Calculate Pitch, Roll and Yaw
  // pitch = pitch + norm.YAxis * timeStep;
  // roll = norm.YAxis * timeStep;
  // yaw = norm.YAxis;

  angle();
  // Output raw
  Serial.print(" Pitch = ");
  Serial.print(pitch);
  Serial.print(" length = ");
  Serial.println(actual_location);
  // Serial.print(" normyaxis*timestep = ");
  // Serial.print(roll);  
  // Serial.print(" normyaxis = ");
  // Serial.print(yaw);
  // Serial.print(" pos = ");
  // Serial.println(actual_location);

  // Wait to full timeStep period
  delay((timeStep*1000) - (millis() - timer));
  
  // analog.update();
  // pot_read = analog.getValue();                  //read the actual position from the actuator potentiometer
  // actual_location = map(pot_read, POTENTIOMETER_MIN, POTENTIOMETER_MAX, 0, STROKE_LENGTH);  //map(value, fromLow, fromHigh, toLow, toHigh)
  control();
    
  if ((roll == 0) && (yaw == 0) && (pitch != 0) && (pitch <= 10) && (pitch >= -10)) {
    pitch = 0;
  }

  if (pitch >= 30){
  pos(50);
  delay(2000); 
  pos(0); 
  pitch = 0;
  }
  // delay(10);

}

// void stuff(int angle){
  
// }

void pos(int command){
  control();
  if ((actual_location <= command + deadband) && (actual_location >= command - deadband)){      //if actual is close to commanded position, do nothing
      analogWrite(RPWM, 0);
      analogWrite(LPWM, 0);
    }

    if (actual_location > command + deadband){         //if too far out, retract
      analogWrite(RPWM, 255);
      analogWrite(LPWM, 0);
    }

    if (actual_location < command - deadband){         //if too far in, extend
      analogWrite(RPWM, 0);
      analogWrite(LPWM, 255);
    }
  angle();
  delay(50);
}

void control()
{
  analog.update();
  pot_read = analog.getValue();                  //read the actual position from the actuator potentiometer
  actual_location = map(pot_read, POTENTIOMETER_MIN, POTENTIOMETER_MAX, 0, STROKE_LENGTH);  //map(value, fromLow, fromHigh, toLow, toHigh)

}

void angle(){
  Vector norm = mpu.readNormalizeGyro();
  pitch = pitch + norm.YAxis * timeStep;
  roll = norm.YAxis * timeStep;
  yaw = norm.YAxis;
}