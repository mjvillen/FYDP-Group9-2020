// https://github.com/denyssene/SimpleKalmanFilter

#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "utilities/high_pass_filter.h"

// GLOBAL VARIABLES
double xPos = 0, yPos = 0, zPos = 0, xVel = 0, yVel = 0, zVel = 0, xAcc = 0, yAcc = 0, zAcc = 0, headingVel = 0;
double avgX = 0, avgY = 0, avgZ = 0;
uint16_t printCount = 0; //counter to avoid printing every 10MS sample

// CONSTANTS
const uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; // how often to read data from the board
const uint16_t PRINT_DELAY_MS = 500; // how often to print the data
const double DELTA_T =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
// const double DEG_2_RAD = 0.01745329251; // trig functions require radians, BNO055 outputs degrees

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address, &Wire
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

//                            cutoffFrequency/samplingFrequency
HighPassFilter highPassFilter_ax(0.1 / (1000/BNO055_SAMPLERATE_DELAY_MS));
HighPassFilter highPassFilter_ay(0.1 / (1000/BNO055_SAMPLERATE_DELAY_MS));
HighPassFilter highPassFilter_az(0.1 / (1000/BNO055_SAMPLERATE_DELAY_MS));
HighPassFilter highPassFilter_vx(0.1 / (1000/BNO055_SAMPLERATE_DELAY_MS));
HighPassFilter highPassFilter_vy(0.1 / (1000/BNO055_SAMPLERATE_DELAY_MS));
HighPassFilter highPassFilter_vz(0.1 / (1000/BNO055_SAMPLERATE_DELAY_MS));

FilterBuHp2 buttHigh1 = FilterBuHp2();
FilterBuHp2 buttHigh2 = FilterBuHp2();
FilterBuHp2 buttHigh3 = FilterBuHp2();

void setup(void) {
  Serial.begin(115200);
  if (!bno.begin()) {
    Serial.print("No BNO055 detected");
    while (1);
  }

  bno.calibrate();
  bno.setExtCrystalUse(true);

  // delay(2000);

  calibrateAccelerations();

  // delay(2000);
}

void loop(void) {
  unsigned long tStart = micros();
  sensors_event_t orientationData, linearAccelData, accelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Quaternion quat = bno.getQuat();
  imu::Quaternion quatConj = quat.conjugate();
  imu::Vector<3> Accel = quat.rotateVector(imu::Vector<3> (accelData.acceleration.x, accelData.acceleration.y, accelData.acceleration.z));

  xAcc = Accel.x();
  yAcc = Accel.y();
  zAcc = Accel.z() - 9.81;

  xVel = xVel + buttHigh1.step(xAcc * DELTA_T);
  yVel = yVel + buttHigh2.step(yAcc * DELTA_T);
  zVel = zVel + buttHigh3.step(zAcc * DELTA_T);

  xPos = xPos + (xVel * DELTA_T);
  yPos = yPos + (yVel * DELTA_T);
  zPos = zPos + (zVel * DELTA_T);

  // velocity of sensor in the direction it's facing
  // headingVel = (0.5 * DELTA_T * DELTA_T) * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);

  if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
    //enough iterations have passed that we can print the latest data
    Serial.print("NEW POS: ");
    Serial.print(xPos, 5);
    Serial.print(" , ");
    Serial.print(yPos, 5);
    Serial.print(" , ");
    Serial.println(zPos, 5);

    printCount = 0;
  }
  else {
    printCount = printCount + 1;
  }

  while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000)) {
    //poll until the next sample is ready
  }
}

void calibrateAccelerations(void) {
  double xSum = 0, ySum = 0, zSum = 0;
  uint16_t count = 0;
  sensors_event_t linearAccelData;

  while (count < 2000) {
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
