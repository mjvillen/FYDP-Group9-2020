// https://github.com/denyssene/SimpleKalmanFilter

#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "utilities/high_pass_filter.h"

// GLOBAL VARIABLES
double xPos = 0, yPos = 0, zPos = 0,
       xVel = 0, yVel = 0, zVel = 0,
       xAcc = 0, yAcc = 0, zAcc = 0;

// CONSTANTS
const uint16_t SAMPLE_RATE = 10; // [ms]
const double DELTA_T =  (double)(SAMPLE_RATE) / 1000.0; // [s]

// Instantiate BNO; id, address, &Wire
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Instantiate HighPass; cutoffFrequency/samplingFrequency (one filter per variable)
HighPassFilter highPassFilter(0.1 / (1000/SAMPLE_RATE));

// Instantiate Butterworth HighPass (one filter per variable)
FilterBuHp2 buttHigh1 = FilterBuHp2();
FilterBuHp2 buttHigh2 = FilterBuHp2();
FilterBuHp2 buttHigh3 = FilterBuHp2();

// case control variable - determines which case runs each loop
int state = 0;

void setup(void) {
  Serial.begin(115200);
}

void loop(void) {
  switch (state) {
    case 0:    // Power on and wait for callibration
      if (!bno.begin()) {
        Serial.print("No BNO055 detected");
        // throw a major error - recover?
        state = 4;
      }

      // indicate waiting for calibration

      // wait for button press then switch to state 1
      state = 1;
      break;

    case 1:    // Calibration Phase
      // indicate calibrating

      bno.calibrate();
      bno.setExtCrystalUse(true);

      // indicate calibration finished

      // wait for button press then switch to state 2
      state = 2;
      break;

    case 2:    // Inactive operation - the arm is currently tracking
      {
        // indicate active operation

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

        Serial.print("POS: ");
        Serial.print(xPos, 5);
        Serial.print(" , ");
        Serial.print(yPos, 5);
        Serial.print(" , ");
        Serial.println(zPos, 5);
        Serial.print("EULER: ");
        Serial.print(orientationData.orientation.pitch, 5);
        Serial.print(" , ");
        Serial.print(orientationData.orientation.heading, 5);
        Serial.print(" , ");
        Serial.println(orientationData.orientation.roll, 5);

        while ((micros() - tStart) < (SAMPLE_RATE * 1000)) {
          //poll until the next sample is ready
        }

        // check for button press to switch to state 3
        state = 3;
      }
      break;

    case 3:    // Active operation - the button is pressed and the arm will not move
      // indicate inactive operation

      // reset all position calculation variables
      xPos = 0, yPos = 0, zPos = 0,
      xVel = 0, yVel = 0, zVel = 0,
      xAcc = 0, yAcc = 0, zAcc = 0;

      // wait for button release then switch to state 2
      state = 2;
      break;

    case 4:    // Error state - something bad happened
      // Maybe split into minor / major?
      // Some sort of error recovery?

      // indicate error

      break;

  } // end state loop
} // end void loop
