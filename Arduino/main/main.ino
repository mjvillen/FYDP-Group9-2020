#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"

// GLOBAL VARIABLES
double xPos = 0, yPos = 0, zPos = 0, xVel = 0, yVel = 0, zVel = 0, xAcc = 0, yAcc = 0, zAcc = 0, headingVel = 0, xAcc2 = 0, yAcc2 = 0, zAcc2 = 0;
uint16_t printCount = 0; //counter to avoid printing every 10MS sample

// CONSTANTS
const uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; // how often to read data from the board
const double DELTA_T =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
// const double DEG_2_RAD = 0.01745329251; // trig functions require radians, BNO055 outputs degrees

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address, &Wire
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BNO055 bno2 = Adafruit_BNO055(55, 0x29, &Wire1);

void setup(void) {
  Serial.begin(115200);
  if (!bno.begin()) {
    Serial.print("No BNO055 detected");
    while (1);
  }

  if (!bno2.begin()) {
    Serial.print("No BNO055 2 detected");
    while (1);
  }

  bno.calibrate();
  bno2.calibrate();
}

void loop(void) {
  unsigned long tStart = micros();

  sensors_event_t orientationData, linearAccelData, accelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Quaternion quat = bno.getQuat();
  imu::Quaternion quatConj = quat.conjugate();
  imu::Vector<3> Accel = quat.rotateVector(imu::Vector<3> (accelData.acceleration.x, accelData.acceleration.y, accelData.acceleration.z));

  sensors_event_t orientationData2, linearAccelData2, accelData2;
  bno2.getEvent(&orientationData2, Adafruit_BNO055::VECTOR_EULER);
  bno2.getEvent(&accelData2, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Quaternion quat2 = bno2.getQuat();
  imu::Quaternion quatConj2 = quat2.conjugate();
  imu::Vector<3> Accel2 = quat2.rotateVector(imu::Vector<3> (accelData2.acceleration.x, accelData2.acceleration.y, accelData2.acceleration.z));

  xAcc = Accel.x();
  yAcc = Accel.y();
  zAcc = Accel.z() - 9.81;

  xAcc2 = Accel2.x();
  yAcc2 = Accel2.y();
  zAcc2 = Accel2.z() - 9.81;

//  if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
    //enough iterations have passed that we can print the latest data
    Serial.print(xAcc, 5);
    Serial.print(",");
    Serial.print(yAcc, 5);
    Serial.print(",");
    Serial.print(zAcc, 5);
    
    Serial.print(",");
    Serial.print(xAcc2, 5);
    Serial.print(",");
    Serial.print(yAcc2, 5);
    Serial.print(",");
    Serial.println(zAcc2, 5);

//    printCount = 0;
//  }
//  else {
//    printCount = printCount + 1;
//  }

  while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000)) {
    //poll until the next sample is ready
  }
}
