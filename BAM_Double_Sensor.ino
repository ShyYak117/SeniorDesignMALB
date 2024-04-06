#include <Wire.h>
#include <MPU6050.h>
#include <ResponsiveAnalogRead.h>

#define RPWM 5
#define LPWM 6
#define STROKE_LENGTH      100    // PLEASE UPDATE THIS VALUE (in millimeter) [4in actuator has max 100mm length]
#define POTENTIOMETER_MAX  962   // PLEASE UPDATE THIS VALUE [from maxing out the 4in]
#define POTENTIOMETER_MIN  34   // PLEASE UPDATE THIS VALUE [from bottoming out the 4in]

MPU6050 mpu; //initializng the 2 mpus to use with library
MPU6050 mpu1;

#define first 0x69
#define second 0x68

// Potentiometer Readings
const int FBA1 = A0;
const int FBA2 = A1;
ResponsiveAnalogRead analogOne(FBA1, true);
ResponsiveAnalogRead analogTwo(FBA2, true);

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Angle stuff
float pitch = 0;
float pitch1 = 0;
float pitch2 = 0;

// Angle stuff for second sensor
float bitch = 0;
float bitch1 = 0;
float bitch2 = 0;

int actual_location = 0;                   // Current position
int deadband =  3;                         // Tolerance for position
int pos1 = 0;                          // Position feedback value
int pos2 = 0;

void setup() {
  // put your setup code here, to run once:

  // Begin running mpu6050 library
  mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G, first);
  mpu1.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G, second);
  
  mpu.calibrateGyro();   // Calibrate gyroscope. The calibration must be at rest.
  mpu1.calibrateGyro();  // Calibrate gyroscope. The calibration must be at rest.

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
  mpu1.setThreshold(3);
}

void loop() {
  // put your main code here, to run repeatedly:

  timer = millis(); //gets the time for the gyro to use for angle detection

  //actuator readings
  angle1(); // thigh angle reading
  angle2(); // calf angle reading
  control(); // actuator length

  // Output raw
  Serial.print("Hip Angle = ");
  Serial.print(pitch);
  Serial.print(" Calf Angle = ");
  Serial.print(bitch);
  Serial.print(" length = ");
  Serial.println(actual_location);


  // Wait to full timeStep period
  delay((timeStep*1000) - (millis() - timer));
  
    
  if ((pitch1 == 0) && (pitch2 == 0) && (pitch != 0) && (pitch <= 10) && (pitch >= -10)) {  // zeroing the angle if it falls within the standing angle but gets fucked
    pitch = 0;
  }

  // initiate the moving sequence once the leg is raised
  if (pitch >= 30){ 
  pos(0); // retract leg
  delay(2000); 
  pos(50); //reposition leg
  angle1();
  }

//   // FOR FUTURE USE OF 2 SENSOR TESTING
//   if (pitch >= 30){
//     for(bitch = 0; (bitch < 50) && (actual_location > 0);){  //arbitrary angle for now that the leg has to pull back to
//     analogWrite(RPWM, 255);
//     analogWrite(LPWM, 0);
//     angle2();
//     control();
//     }
//     pos(50); // revert back to normal
//     angle1();
//   }
}
void pos(int command){ // position controller
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
  angle1();
  delay(50);
}

void control() // position update in mm
{
  analogOne.update();
  pos1 = analogOne.getValue();                  //read the actual position from the actuator potentiometer
  actual_location = map(pos1, POTENTIOMETER_MIN, POTENTIOMETER_MAX, 0, STROKE_LENGTH);  //map(value, fromLow, fromHigh, toLow, toHigh)

}

void angle1(){
  Vector norm = mpu.readNormalizeGyro();
  pitch = pitch + norm.YAxis * timeStep;
  pitch1 = norm.YAxis * timeStep;
  pitch2 = norm.YAxis;
}

void angle2(){
  Vector norm1 = mpu1.readNormalizeGyro();
  bitch = bitch + norm1.YAxis * timeStep;
  bitch1 = norm1.YAxis * timeStep;
  bitch2 = norm1.YAxis;
}