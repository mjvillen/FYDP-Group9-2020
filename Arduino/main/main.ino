#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"

// GLOBAL VARIABLES
bool calibrated = false;
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
double xPos = 0, yPos = 0, zPos = 0;
double pitch = 0, yaw = 0, roll = 0;
double xOffset = 0, yOffset = 0, zOffset = 0, elbowOffset = 0;
bool zeroed = false;

// CONSTANTS
const uint16_t SAMPLE_RATE = 10; // [ms]
const uint16_t buttonPin = 7; // TODO: set button pin
const uint16_t redLightPin= 11; // TODO: set LED pin
const uint16_t greenLightPin = 12; // TODO: set LED pin
const uint16_t blueLightPin = 13; // TODO: set LED pin
const uint16_t buttonDebounce = 10;
const double L1 = 27; // TODO: Upper-arm Length
const double L2 = 23; // TODO: Forearm Length
const uint16_t CLOCK_PIN = 5; // encoder
const uint16_t DATA_PIN = 6; // encoder

// Instantiate BNO; id, address, &Wire
Adafruit_BNO055 bnoWrist1 = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BNO055 bnoWrist2 = Adafruit_BNO055(55, 0x29, &Wire);
Adafruit_BNO055 bnoWrist3 = Adafruit_BNO055(55, 0x28, &Wire1);
Adafruit_BNO055 bnoShoulder = Adafruit_BNO055(55, 0x29, &Wire1);

imu::Vector<3> getPosition(double theta_0, double theta_1, double theta_2, double theta_3) {
  double x = L1*sin(theta_3)*(sin(theta_0)*sin(theta_2) + cos(theta_0)*cos(theta_2)*sin(theta_1)) - (L1 + L2)*(sin(theta_0)*sin(theta_2)*sin(theta_3) - cos(theta_0)*cos(theta_1)*cos(theta_3) + cos(theta_0)*cos(theta_2)*sin(theta_1)*sin(theta_3)) - (L1*cos(theta_0)*cos(theta_1)*2*(cos(theta_3) - 1))/2;
  double y = (L1 + L2)*(cos(theta_0)*sin(theta_2)*sin(theta_3) + cos(theta_1)*cos(theta_3)*sin(theta_0) - cos(theta_2)*sin(theta_0)*sin(theta_1)*sin(theta_3)) - L1*sin(theta_3)*(cos(theta_0)*sin(theta_2) - cos(theta_2)*sin(theta_0)*sin(theta_1)) - (L1*cos(theta_1)*sin(theta_0)*2*(cos(theta_3) - 1))/2;
  double z = L1*cos(theta_1)*cos(theta_2)*sin(theta_3) - (cos(theta_3)*sin(theta_1) + cos(theta_1)*cos(theta_2)*sin(theta_3))*(L1 + L2) + (L1*sin(theta_1)*2*(cos(theta_3) - 1))/2;

  return imu::Vector<3>(x, y, z);
}

void RGBColor(uint8_t redValue, uint8_t greenValue, uint8_t blueValue) {
  analogWrite(redLightPin, redValue);
  analogWrite(greenLightPin, greenValue);
  analogWrite(blueLightPin, blueValue);
}

double getElbowAngle() {
  // Initial clock high/low to indicate prep for read
  digitalWrite(CLOCK_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(CLOCK_PIN, LOW);
  delayMicroseconds(1);

  // read 16 bits into stream
  byte stream[16] = {0.};
  for (int i = 0; i < 16; i++)
  {
    // indicate new byte
    digitalWrite(CLOCK_PIN, HIGH);
    digitalWrite(CLOCK_PIN, LOW);
    stream[i] = digitalRead(DATA_PIN);
  }

  // perform binary shift to pull raw value from binary stream
  double value = 0;
  for (int i = 0; i < 16; i++) {
    value += stream[i]*(1 << (15-i));
  }

  // return normalized value converted to degrees
  return ((value/65536) * 360);
}

void setup(void) {
  Serial.begin(115200);

  state = waitingCalibration;

  // LED pin setup
  pinMode(buttonPin, INPUT);
  pinMode(redLightPin, OUTPUT);
  pinMode(greenLightPin, OUTPUT);
  pinMode(blueLightPin, OUTPUT);

  // encoder pin setup
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, INPUT);
  digitalWrite(CLOCK_PIN, HIGH);
}

void loop(void) {
  switch (state) {
    case waitingCalibration:    // Power on and wait for callibration
      {
        if (!bnoShoulder.begin()) {
          Serial.print("No Shoulder BNO055 detected");
          // TODO: throw a major error - recover?
          state = error;
          break;
        }

        if (!bnoWrist1.begin()) {
          Serial.print("No Wrist BNO055 (1) detected");
          // TODO: throw a major error - recover?
          state = error;
          break;
        }

        if (!bnoWrist2.begin()) {
          Serial.print("No Wrist BNO055 (2) detected");
          // TODO: throw a major error - recover?
          state = error;
          break;
        }

        if (!bnoWrist3.begin()) {
          Serial.print("No Wrist BNO055 (3) detected");
          // TODO: throw a major error - recover?
          state = error;
          break;
        }

        // indicate waiting for calibration
        RGBColor(0, 0, 255); // Blue
        // Serial.println("Waiting for calibration");

        // wait for button press and release then switch to calibrating
        buttonPressCount = 0;
        while (digitalRead(buttonPin) == HIGH) {
          buttonPressCount++;
          if (buttonPressCount > buttonDebounce / 2) {
            state = calibrating;
          }
        }
      }
      break;

    case calibrating:    // Calibration Phase
      {
        if (!calibrated) {
          // indicate calibrating
          RGBColor(255, 255, 0); // Yellow
          // Serial.println("calibrating");

          // calibrate bno
          bnoShoulder.calibrate();
          bnoWrist1.calibrate();
          bnoWrist2.calibrate();
          bnoWrist3.calibrate();

          calibrated = true;
        }

        // indicate calibration finished
        RGBColor(0, 255, 255); // Cyan
        // Serial.println("calibration done");

        // wait for button press then switch to active operation
        buttonPressCount = 0;
        while (digitalRead(buttonPin) == HIGH) {
          buttonPressCount++;
          if (buttonPressCount > buttonDebounce) {
            xPos = 0, yPos = 0, zPos = 0; // Ensure that the global position has been zeroed
            state = activeOperation;

            // get Offsets
            bnoShoulder.setOffsets();
            bnoWrist1.setOffsets();
            bnoWrist2.setOffsets();
            bnoWrist3.setOffsets();
            elbowOffset = getElbowAngle();

            break;
          }
        }
      }
      break;

    case activeOperation:    // Active operation - the button is pressed and the arm is currently tracking
      {
        // indicate active operation
        RGBColor(0, 255, 0); // Green
        // Serial.println("Active operation");


        ///////////////////////////////////////////////
        ////////// Update Absolute Positions //////////
        ///////////////////////////////////////////////
        // instantiate time current time
        tStart = micros();

        // Get readings
        bnoWrist1.updateReadings();
        imu::Vector<3> position1 = bnoWrist1.getPosition();
        imu::Vector<3> euler1 = bnoWrist1.getEuler();
        imu::Vector<3> offsetAngles1 = bnoWrist1.getOffsetPitchYawRoll();

        bnoWrist2.updateReadings();
        imu::Vector<3> position2 = bnoWrist2.getPosition();
        imu::Vector<3> euler2 = bnoWrist2.getEuler();
        imu::Vector<3> offsetAngles2 = bnoWrist2.getOffsetPitchYawRoll();

        bnoWrist3.updateReadings();
        imu::Vector<3> position3 = bnoWrist3.getPosition();
        imu::Vector<3> euler3 = bnoWrist3.getEuler();
        imu::Vector<3> offsetAngles3 = bnoWrist3.getOffsetPitchYawRoll();

        while ((micros() - tStart) < (SAMPLE_RATE * 1000)) {
          // poll until the next sample is ready
          // TODO: Make sure this is running fast enough
        }


        ///////////////////////////////////////////////
        /////////// Update Elbow Positions ////////////
        ///////////////////////////////////////////////
        imu::Vector<3> offsetAngles4 = bnoShoulder.getOffsetPitchYawRoll();
        double pitch4 = offsetAngles4.x();
        double yaw4 = offsetAngles4.y();
        double roll4 = offsetAngles4.z();
        double elbow = getElbowAngle() - elbowOffset;

        // if we need to rezero the arm, get the new offset values and update zeroed bool
        if (!zeroed) {
          imu::Vector<3> position4 = getPosition(pitch4, yaw4, roll4, elbow);
          xOffset = position4.x();
          yOffset = position4.y();
          zOffset = position4.z();

          zeroed = true;
        }

        // get the position from current readings
        //  TODO: gets set twice initially
        imu::Vector<3> position4 = getPosition(pitch4, yaw4, roll4, elbow);

        // TODO: maybe delete - mostly here for debugs
        double x = position4.x();
        double y = position4.y();
        double z = position4.z();

        // update the global position based on the current while considering the offset for start position
        // TODO: how to deal wiith position
        xPos += x - xOffset;
        yPos += y - yOffset;
        zPos += z - zOffset;

        // update the global pitch, yaw roll
        // TODO: how to deal with angles
        pitch = offsetAngles4.x();
        yaw = offsetAngles4.y();
        roll = offsetAngles4.z();


        // wait for button release then switch to inactive operation
        buttonReleaseCount = 0;
        while (digitalRead(buttonPin) == LOW) {
          buttonReleaseCount++;
          if (buttonReleaseCount > buttonDebounce) {
            state = inactiveOperation;
            break;
          }
        }
      }
      break;

    case inactiveOperation:    // Inactive operation - the button is not pressed and the arm will not move
      {
        // indicate inactive operation
        RGBColor(255, 255, 255); // White
        // Serial.println("inacttive operation");

        // reset all position calculation variables
        bnoWrist1.resetPosition();
        bnoWrist2.resetPosition();
        bnoWrist3.resetPosition();

        // check for button press to switch to active operation
        buttonPressCount = 0;
        while (digitalRead(buttonPin) == HIGH ) {
          buttonPressCount++;
          if (buttonPressCount > buttonDebounce) {
            state = activeOperation;
            break;
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
