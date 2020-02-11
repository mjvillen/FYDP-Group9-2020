// https://github.com/denyssene/SimpleKalmanFilter

#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"

// GLOBAL VARIABLES
unsigned long tStart = 0;
int state = 0;  // case control variable

// CONSTANTS
const uint16_t SAMPLE_RATE = 10; // [ms]

// Instantiate BNO; id, address, &Wire
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup(void) {
  Serial.begin(115200);
}

void loop(void) {
  switch (state) {
    case 0:    // Power on and wait for callibration
      if (!bno.begin()) {
        Serial.print("No BNO055 detected");
        // TODO: throw a major error - recover?
        state = 4;
      }

      // TODO: indicate waiting for calibration

      // TODO: wait for button press then switch to state 1
      state = 1;
      break;

    case 1:    // Calibration Phase
      // TODO: indicate calibrating

      // calibrate bno
      bno.calibrate();
      bno.setExtCrystalUse(true);

      // TODO: indicate calibration finished

      // TODO: wait for button press then switch to state 2
      state = 2;
      break;

    case 2:    // Inactive operation - the arm is currently tracking
      {
        // TODO: indicate active operation

        // instantiate time current time
        tStart = micros();

        // update the readings
        bno.updateReadings();
        imu::Vector<3> position = bno.getPosition();
        imu::Vector<3> euler = bno.getEuler();

        while ((micros() - tStart) < (SAMPLE_RATE * 1000)) {
          // poll until the next sample is ready
        }

        // TODO: check for button press to switch to state 3
        state = 3;
      }
      break;

    case 3:    // Active operation - the button is pressed and the arm will not move
      // TODO: indicate inactive operation

      // reset all position calculation variables
      bno.resetPosition();

      // TODO: wait for button release then switch to state 2
      state = 2;
      break;

    case 4:    // Error state - something bad happened
      // TODO: Maybe split into minor / major?
      // TODO: Some sort of error recovery?

      // TODO: indicate error

      break;

  } // end state loop
} // end void loop
