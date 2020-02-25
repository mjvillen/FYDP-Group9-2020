// https://github.com/denyssene/SimpleKalmanFilter

#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"

// GLOBAL VARIABLES
uint16_t buttonPressCount = 0;
uint16_t buttonReleaseCount = 0;
enum states {
  activeOperation,
  inactiveOperation,
  waitingCalibration,
  calibrating,
  error
};
states state;  // case control variable
double xPos = 0, yPos = 0, zPos = 0;
double xOffset = 0, yOffset = 0, zOffset = 0;
bool zeroed = false;

// CONSTANTS
const uint16_t buttonPin = 2; // TODO: set button pin
const uint16_t redLightPin= 11; // TODO: set LED pin
const uint16_t greenLightPin = 12; // TODO: set LED pin
const uint16_t blueLightPin = 13; // TODO: set LED pin
const double L1 = 27; // TODO: Upper-arm Length
const double L2 = 23; // TODO: Forearm Length

// Instantiate BNO; id, address, &Wire
Adafruit_BNO055 bnoShoulder = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 bnoWrist = Adafruit_BNO055(55, 0x29);

imu::Vector<3> getPosition(double theta_0, double theta_1, double theta_2, double theta_3) {
  // TODO: fix this
  double x = 0;//(L1 + L2)*(cos(theta_0)*cos(theta_1)*cos(theta_3) - cos(theta_2)*sin(theta_0)*sin(theta_3) + cos(theta_0)*sin(theta_1)*sin(theta_2)*sin(theta_3)) + L1*sin(theta_3)*(cos(theta_2)*sin(theta_0) - cos(theta_0)*sin(theta_1)*sin(theta_2)) - (L1*exp(-theta_3*1i)*cos(theta_0)*cos(theta_1)*(exp(theta_3*1i) - 1)^2)/2;
  double y = 0;//(L1 + L2)*(cos(theta_1)*cos(theta_3)*sin(theta_0) + cos(theta_0)*cos(theta_2)*sin(theta_3) + sin(theta_0)*sin(theta_1)*sin(theta_2)*sin(theta_3)) - L1*sin(theta_3)*(cos(theta_0)*cos(theta_2) + sin(theta_0)*sin(theta_1)*sin(theta_2)) - (L1*exp(-theta_3*1i)*cos(theta_1)*sin(theta_0)*(exp(theta_3*1i) - 1)^2)/2;
  double z = 0;//-1 * (cos(theta_3)*sin(theta_1) - cos(theta_1)*sin(theta_2)*sin(theta_3))*(L1 + L2) - L1*cos(theta_1)*sin(theta_2)*sin(theta_3) + (L1*sin(theta_1)*(cos(theta_3) - sin(theta_3)*1i)*(cos(theta_3) - 1 + sin(theta_3)*1i)^2)/2;

  imu::Vector<3> ret = imu::Vector<3>(x, y, z);
  return ret;
}

void setup(void) {
  Serial.begin(115200);

  state = waitingCalibration;

  pinMode(buttonPin, INPUT);
  pinMode(redLightPin, OUTPUT);
  pinMode(greenLightPin, OUTPUT);
  pinMode(blueLightPin, OUTPUT);
}

void loop(void) {
  switch (state) {
    case waitingCalibration:    // Power on and wait for callibration
      {
        if (!bnoShoulder.begin()) {
          Serial.print("No Shoulder IMU detected");
          // TODO: throw a major error - recover?
          state = error;
        }

        if (!bnoWrist.begin()) {
          Serial.print("No Wrist IMU detected");
          // TODO: throw a major error - recover?
          state = error;
        }

        // indicate waiting for calibration
        RGBColor(0, 0, 255); // Blue

        // wait for button press then switch to calibrating
        buttonPressCount = 0;
        while (digitalRead(buttonPin) == HIGH) {
          buttonPressCount++;
        }
        if (buttonPressCount > 20) {
          state = calibrating;
        }
      }
      break;

    case calibrating:    // Calibration Phase
      {
        // indicate calibrating
        RGBColor(255, 255, 0); // Yellow

        // calibrate IMUs
        bnoShoulder.calibrate();
        bnoShoulder.setExtCrystalUse(true);

        bnoWrist.calibrate();
        bnoWrist.setExtCrystalUse(true);

        // indicate calibration finished
        RGBColor(0, 255, 255); // Cyan

        // wait for button press then switch to active operation
        buttonPressCount = 0;
        while (digitalRead(buttonPin) == HIGH) {
          buttonPressCount++;
        }
        if (buttonPressCount > 20) {
          xPos = 0, yPos = 0, zPos = 0;
          state = activeOperation;
          zeroed = false;
        }
      }
      break;

    case activeOperation:    // Active operation - the button is pressed and the arm is currently tracking
      {
        sensors_event_t orientationData;
        bnoShoulder.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        double pitch = orientationData.orientation.pitch;
        double yaw = orientationData.orientation.heading;
        double roll = orientationData.orientation.roll;

        // TODO: elbow reading goes here
        double elbow = 0;

        if (!zeroed) {
          imu::Vector<3> pos = getPosition(pitch, yaw, roll, elbow);
          xOffset = pos.x();
          yOffset = pos.y();
          zOffset = pos.z();

          zeroed = true;
        }

        // indicate active operation
        RGBColor(0, 255, 0); // Green

        imu::Vector<3> pos = getPosition(pitch, yaw, roll, elbow);
        double x = pos.x();
        double y = pos.y();
        double z = pos.z();

        xPos += x - xOffset;
        yPos += y - yOffset;
        zPos += z - zOffset;

        // wait for button release then switch to inactive operation
        buttonReleaseCount = 0;
        while (digitalRead(buttonPin) == LOW) {
          buttonReleaseCount++;
        }
        if (buttonReleaseCount > 20) {
          state = activeOperation;
        }
      }
      break;

    case inactiveOperation:    // Inactive operation - the button is not pressed and the arm will not move
      {
        // indicate inactive operation
        RGBColor(255, 255, 255); // White

        // TODO: reset all position calculation variables

        // check for button press to switch to active operation
        buttonPressCount = 0;
        bool buttonPressed = false;
        while (digitalRead(buttonPin) == HIGH && !buttonPressed) {
          buttonPressCount++;

          if (buttonPressCount > 20) {
            state = inactiveOperation;
            buttonPressed = true;
            zeroed = false;
          }
        }
      }
      break;

    case error:    // Error state - something bad happened
      {
        // TODO: Maybe split into minor / major?
        // TODO: Some sort of error recovery?

        // indicate error
        RGBColor(255, 0, 0); // Red
      }
      break;

  } // end state loop
} // end void loop

void RGBColor(uint8_t redValue, uint8_t greenValue, uint8_t blueValue) {
  analogWrite(redLightPin, redValue);
  analogWrite(greenLightPin, greenValue);
  analogWrite(blueLightPin, blueValue);
}
