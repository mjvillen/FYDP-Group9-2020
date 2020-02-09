#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"

// GLOBAL VARIABLES
double xPos = 0, yPos = 0, zPos = 0, xVel = 0, yVel = 0, zVel = 0, xAcc = 0, yAcc = 0, zAcc = 0, headingVel = 0;
double avgX = 0, avgY = 0, avgZ = 0;
uint16_t printCount = 0; //counter to avoid printing every 10MS sample

// CONSTANTS
const uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
const uint16_t PRINT_DELAY_MS = 500; // how often to print the data
const double deltaT =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
const double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address, &Wire
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup(void) {
  Serial.begin(9600);
  if (!bno.begin()) {
    Serial.print("No BNO055 detected");
    while (1);
  }

  bno.calibrate();
  bno.setExtCrystalUse(true);

  delay(5000);

  calibrateAccelerations();

  delay(1000);
}

void loop(void) {
  unsigned long tStart = micros();
  sensors_event_t orientationData, linearAccelData, accelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  // This is the quaternion math way. It should be similar to the linear acceleration so leaving in comments for now
  // bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  // imu::Quaternion quat = bno.getQuat();
  // imu::Quaternion quatConj = quat.conjugate();
  // imu::Vector<3> Accel = quat.rotateVector(imu::Vector<3> (accelData.acceleration.x, accelData.acceleration.y, accelData.acceleration.z));

  xAcc = (abs(linearAccelData.acceleration.x - avgX) > 0.2) ? linearAccelData.acceleration.x - avgX : 0;
  yAcc = (abs(linearAccelData.acceleration.y - avgY) > 0.2) ? linearAccelData.acceleration.y - avgY : 0;
  zAcc = (abs(linearAccelData.acceleration.z - avgZ) > 0.2) ? linearAccelData.acceleration.z - avgZ : 0;

  xVel = xVel + xAcc * deltaT;
  yVel = yVel + yAcc * deltaT;
  zVel = zVel + zAcc * deltaT;
  xVel = (abs(xVel) > 0.1) ? xVel : 0;
  yVel = (abs(yVel) > 0.1) ? yVel : 0;
  zVel = (abs(zVel) > 0.1) ? zVel : 0;

  xPos = xPos + (xVel * deltaT);
  yPos = yPos + (yVel * deltaT);
  zPos = zPos + (zVel * deltaT);

  // velocity of sensor in the direction it's facing
  // headingVel = (0.5 * deltaT * deltaT) * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);

  // if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
  //   //enough iterations have passed that we can print the latest data
    Serial.print("NEW POS: ");
    Serial.print(xPos, 5);
    Serial.print(" , ");
    Serial.print(yPos, 5);
    Serial.print(" , ");
    Serial.println(zPos, 5);

  //   printCount = 0;
  // }
  // else {
  //   printCount = printCount + 1;
  // }


  while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000)) {
    //poll until the next sample is ready
  }
}

void calibrateAccelerations(void) {
  double xSum = 0, ySum = 0, zSum = 0;
  uint16_t count = 0;
  sensors_event_t linearAccelData;

  while (count < 5000) {
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    xSum = xSum + linearAccelData.acceleration.x;
    ySum = ySum + linearAccelData.acceleration.y;
    zSum = zSum + linearAccelData.acceleration.z;

    count = count + 1;
  }

  avgX = xSum / 5000;
  avgY = ySum / 5000;
  avgZ = zSum / 5000;
}
