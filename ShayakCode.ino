#include <Wire.h>
#include <Adafruit_MPU6050.h>

#define MPU1_ADDR 0x68 // Address of IMU with AD0 pin connected to LOW ie GND
#define MPU2_ADDR 0x69 // Address of IMU with AD0 pin connected to HIGH ie 5V

Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;

#define y1g 9.75 // value of 1g at y axis
#define z1g 9.6  // value of 1g at z axis
#define x1g 10.0 //value of 1g at x axis



// Calculate angle of IMU
// angle = acos(g_axis / 1g_acc)
// g_axis is the measured acceleration
// 1g_acc is the value of 1g for that axis (may vary)

void angle_measure(float acc_measured, float acc_1g, double* angle) {
  float ratio;
  ratio = acc_measured / acc_1g;
  *angle = (180 / M_PI) * acos(ratio);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10); // wait for serial port to connect
    }

    Wire.begin();

    // Initialize the first MPU6050 sensor
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

void loop() {
    // Read data from the first sensor
    float mpu1_ax,mpu1_ay,mpu1_az;
    float mpu2_ax,mpu2_ay,mpu2_az;
    double AngleThigh_GlobalZ,AngleCalf_GlobalZ; // Global Z is the direction along which 1g acts (Earth gravity)
    sensors_event_t a, g, temp;
    mpu1.getEvent(&a, &g, &temp); // method getEvent for object mpu1 to get accel,gyro and temperature readings
    mpu1_ax = a.acceleration.x;
    mpu1_ay = a.acceleration.y;
    mpu1_az = a.acceleration.z;
    /*
    Serial.print("Sensor 1: ");
    Serial.print("AccelX: "); Serial.print(a.acceleration.x);
    Serial.print(", AccelY: "); Serial.print(a.acceleration.y);
    Serial.print(", AccelZ: "); Serial.print(a.acceleration.z);
    Serial.println();
    */
    // Read data from the second sensor
    
    mpu2.getEvent(&a, &g, &temp);
    mpu2_ax = a.acceleration.x;
    mpu2_ay = a.acceleration.y;
    mpu2_az = a.acceleration.z;
    /*
    Serial.print("Sensor 2: ");
    Serial.print("AccelX: "); Serial.print(a.acceleration.x);
    Serial.print(", AccelY: "); Serial.print(a.acceleration.y);
    Serial.print(", AccelZ: "); Serial.print(a.acceleration.z);
    Serial.println();
    */
    Serial.println("");
    angle_measure(mpu1_az,z1g,&AngleThigh_GlobalZ);
    Serial.print("Thigh angle: ");
    Serial.print(AngleThigh_GlobalZ-19);
    Serial.println(" deg");

    Serial.println("");
    angle_measure(mpu2_az,z1g,&AngleCalf_GlobalZ);
    Serial.print("Calf angle: ");
    Serial.print(AngleCalf_GlobalZ);
    Serial.println(" deg");
    delay(500); // Adjust delay as needed
}

