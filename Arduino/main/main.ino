// https://github.com/denyssene/SimpleKalmanFilter

#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"

// GLOBAL VARIABLES
uint16_t buttonPressCount = 0;
uint16_t buttonReleaseCount = 0;
unsigned long tStart = 0;
enum states {
  activeOperation,
  inactiveOperation,
  waitingCalibration,
  calibrating,
  error
};
states state;  // case control variable

// CONSTANTS
const uint16_t SAMPLE_RATE = 10; // [ms]
const uint16_t buttonPin = 2; // TODO: set button pin
const uint16_t redLightPin= 11; // TODO: set LED pin
const uint16_t greenLightPin = 12; // TODO: set LED pin
const uint16_t blueLightPin = 13; // TODO: set LED pin

// Instantiate BNO; id, address, &Wire
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

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
      if (!bno.begin()) {
        Serial.print("No BNO055 detected");
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

      // calibrate bno
      bno.calibrate();
      bno.setExtCrystalUse(true);

      // indicate calibration finished
      RGBColor(0, 255, 255); // Cyan

      // wait for button press then switch to inactive operation
      buttonPressCount = 0;
      while (digitalRead(buttonPin) == HIGH) {
        buttonPressCount++;
      }
      if (buttonPressCount > 20) {
        state = inactiveOperation;
      }
      break;

    case inactiveOperation:    // Inactive operation - the arm is currently tracking
      {
        // indicate active operation
        RGBColor(0, 255, 0); // Green

        // instantiate time current time
        tStart = micros();

        // update the readings
        bno.updateReadings();
        imu::Vector<3> position = bno.getPosition();
        imu::Vector<3> euler = bno.getEuler();

        while ((micros() - tStart) < (SAMPLE_RATE * 1000)) {
          // poll until the next sample is ready
        }

        // wait for button release then switch to active operation
        buttonReleaseCount = 0;
        while (digitalRead(buttonPin) == LOW) {
          buttonReleaseCount++;
        }
        if (buttonReleaseCount > 20) {
          state = inactiveOperation;
        }
      }
      break;

    case activeOperation:    // Active operation - the button is pressed and the arm will not move
      // indicate inactive operation
      RGBColor(255, 255, 255); // White

      // reset all position calculation variables
      bno.resetPosition();

      // check for button press to switch to inactive operation
      buttonPressCount = 0;
      bool buttonPrressed = false;
      while (digitalRead(buttonPin) == HIGH && !buttonPrressed) {
        buttonPressCount++;

        if (buttonPressCount > 20) {
          state = activeOperation;
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
