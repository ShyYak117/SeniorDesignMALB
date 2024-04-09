#include <Wire.h>
#include <MPU6050.h>
#include <ResponsiveAnalogRead.h>


#define MPU1_ADDR 0x68 // Address of IMU with AD0 pin connected to LOW ie GND
#define MPU2_ADDR 0x69 // Address of IMU with AD0 pin connected to HIGH ie 5V
MPU6050 mpu1;
MPU6050 mpu2;

#define RPWM 5
#define LPWM 6
#define STROKE_LENGTH      100    // PLEASE UPDATE THIS VALUE (in millimeter) [4in actuator has max 100mm length]
#define POTENTIOMETER_MAX  962   // PLEASE UPDATE THIS VALUE [from maxing out the 4in]
#define POTENTIOMETER_MIN  34   // PLEASE UPDATE THIS VALUE [from bottoming out the 4in]
int actual_location = 0;                   // Current position
int deadband =  3;                         // Tolerance for position
int pos1 = 0;                          // Position feedback value

// Potentiometer Readings
const int FBA1 = A0;
// const int FBA2 = A1;
ResponsiveAnalogRead analogOne(FBA1, true);

void setup() 
{
  Serial.begin(115200);

  if (!mpu1.begin(MPU1_ADDR)) {
        Serial.println("Failed to find MPU6050 sensor 1");
        while (1);
    }
    Serial.println("MPU6050 sensor 1 found!");

    // Initialize the second MPU6050 sensor
    if (!mpu2.begin(MPU2_ADDR)) {
        Serial.println("Failed to find MPU6050 sensor 2");
        while (1);
    }
    Serial.println("MPU6050 sensor 2 found!");
}

void loop()
{
  // Read normalized values 
  Vector normAccel1 = mpu1.readNormalizeAccel();
  Vector normAccel2 = mpu2.readNormalizeAccel();

  // Calculate Pitch & Roll
  int hip = -((atan2(normAccel1.YAxis, normAccel1.ZAxis)*180.0)/M_PI) + 90; //subtract 90 since the sensor is at 90 degrees at vertical
  int roll2 = -((atan2(normAccel2.YAxis, normAccel2.ZAxis)*180.0)/M_PI) + 90;
  int calf = hip - roll2; // calf angle in relation to the thigh. need to adjust if the angle of the sensor is the opposite dir
  

  // Output
  Serial.print(" hip = ");
  Serial.print(hip);
  Serial.print(" calf = ");
  Serial.print(calf);
  Serial.print(" length = ");
  Serial.print(actual_location);
  Serial.println();
  
  // initiate the moving sequence once the leg is raised
  if ((hip >= 30) && (calf < 65)){ 
    pos(0); // retract leg
  }
  else{
    pos(50); //reposition leg

  }

  delay(10);
  estop();
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
  delay(50);
}

void estop(){ // position controller
  control();
    if (actual_location > 53){         //if too far out, retract
      analogWrite(RPWM, 255);
      analogWrite(LPWM, 0);
    }
}

void control() // position update in mm
{
  analogOne.update();
  pos1 = analogOne.getValue();                  //read the actual position from the actuator potentiometer
  actual_location = map(pos1, POTENTIOMETER_MIN, POTENTIOMETER_MAX, 0, STROKE_LENGTH);  //map(value, fromLow, fromHigh, toLow, toHigh)
}