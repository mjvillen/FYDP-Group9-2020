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

// CONSTANTS
const uint16_t buttonPin = 2; // TODO: set button pin
const uint16_t redLightPin= 11; // TODO: set LED pin
const uint16_t greenLightPin = 12; // TODO: set LED pin
const uint16_t blueLightPin = 13; // TODO: set LED pin

// Instantiate BNO; id, address, &Wire
Adafruit_BNO055 bnoShoulder = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 bnoWrist = Adafruit_BNO055(55, 0x29);


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
      break;

    case calibrating:    // Calibration Phase
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
        state = activeOperation;
      }
      break;

    case activeOperation:    // Active operation - the button is pressed and the arm is currently tracking
      {
        // indicate active operation
        RGBColor(0, 255, 0); // Green

        // TODO: update the readings
        bnoShoulder

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
      // indicate inactive operation
      RGBColor(255, 255, 255); // White

      // TODO: reset all position calculation variables

      // check for button press to switch to active operation
      buttonPressCount = 0;
      bool buttonPrressed = false;
      while (digitalRead(buttonPin) == HIGH && !buttonPrressed) {
        buttonPressCount++;

        if (buttonPressCount > 20) {
          state = inactiveOperation;
          buttonPrressed  = true;
        }
      }
     break;

    case error:    // Error state - something bad happened
      // TODO: Maybe split into minor / major?
      // TODO: Some sort of error recovery?

      // indicate error
      RGBColor(255, 0, 0); // Red

      break;

  } // end state loop
} // end void loop

void RGBColor(uint8_t redValue, uint8_t greenValue, uint8_t blueValue) {
  analogWrite(redLightPin, redValue);
  analogWrite(greenLightPin, greenValue);
  analogWrite(blueLightPin, blueValue);
}
