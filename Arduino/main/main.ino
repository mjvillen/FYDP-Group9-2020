#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"


///// GLOBAL VARIABLES /////
bool calibrated = false;
uint16_t buttonPressCount = 0;
uint16_t buttonReleaseCount = 0;
unsigned long tStart = 0;
enum states {
  activeOperation,
  inactiveOperation,
  awaitingCalibration,
  calibrating,
  error
};
states state;  // case control variable
double xPos = 0, yPos = 0, zPos = 0;
double pitch = 0, yaw = 0, roll = 0;
double xOffset = 0, yOffset = 0, zOffset = 0, elbowOffset = 0;
bool zeroed = false;


////// CONSTANTS /////
const uint16_t SAMPLE_RATE = 10; // [ms]
const uint16_t buttonPin = 7;
const uint16_t redLightPin= 11;
const uint16_t greenLightPin = 12;
const uint16_t blueLightPin = 13;
const uint16_t buttonDebounce = 10;
const double L1 = 29.0; // Upper-arm Length
const double L2 = 22.4; // Forearm Length
const uint16_t CLOCK_PIN = 5; // encoder
const uint16_t DATA_PIN = 6; // encoder


////// INSTANTIATE IMUS /////
Adafruit_BNO055 bnoShoulder = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BNO055 bnoWrist1 = Adafruit_BNO055(55, 0x28, &Wire1);
Adafruit_BNO055 bnoWrist2 = Adafruit_BNO055(55, 0x29, &Wire1);


////// FUNCTION DECLARATIONS /////
double getElbowAngle();
void RGBColor(uint8_t redValue, uint8_t greenValue, uint8_t blueValue);
imu::Vector<3> getHandPosition(double theta_0, double theta_1, double theta_2, double theta_3);
imu::Vector<3> getElbowPosition(double theta_0, double theta_1);
void printAngles(imu::Vector<3> eulerAngles, double elbow = -1);
void printPosition(imu::Vector<3> position);


///// SETUP /////
void setup(void) {
  Serial.begin(115200);

  state = awaitingCalibration;

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


///// MAIN LOOP /////
void loop(void) {
  switch (state) {
    case awaitingCalibration:    // Power on and wait for callibration
      {
        if (!bnoShoulder.begin()) {
          Serial.print("No Shoulder BNO055 detected");
          state = error;
          break;
        }

        if (!bnoWrist1.begin()) {
          Serial.print("No Wrist BNO055 (1) detected");
          state = error;
          break;
        }

        if (!bnoWrist2.begin()) {
          Serial.print("No Wrist BNO055 (2) detected");
          state = error;
          break;
        }

        // indicate waiting for calibration
        RGBColor(0, 0, 255); // Blue
        Serial.println("Waiting for calibration");

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
          Serial.println("calibrating");

          // calibrate bno
          bnoShoulder.calibrate();
          bnoWrist1.calibrate();
          bnoWrist2.calibrate();

          calibrated = true;
        }

        // indicate calibration finished
        RGBColor(0, 255, 255); // Cyan
        Serial.println("calibration done");

        // wait for button press then switch to active operation
        buttonPressCount = 0;
        while (digitalRead(buttonPin) == HIGH) {
          buttonPressCount++;
          if (buttonPressCount > buttonDebounce) {
            xPos = 0, yPos = 0, zPos = 0; // Ensure that the global position has been zeroed
            state = activeOperation;

            // get Offsets
            bnoShoulder.setAngleOffsets();
            bnoWrist1.setAngleOffsets();
            bnoWrist2.setAngleOffsets();
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
        Serial.println("Active operation");


        ///////////////////////////////////////////////
        ////////// Update Absolute Positions //////////
        ///////////////////////////////////////////////
        // instantiate time current time
        tStart = micros();

        // Get readings
        bnoWrist1.updateReadings();
        imu::Vector<3> position1 = bnoWrist1.getPosition();
        imu::Vector<3> offsetAngles1 = bnoWrist1.getOffsetAngles();

        bnoWrist2.updateReadings();
        imu::Vector<3> position2 = bnoWrist2.getPosition();
        imu::Vector<3> offsetAngles2 = bnoWrist2.getOffsetAngles();


        ///////////////////////////////////////////////
        /////////// Update Elbow Positions ////////////
        ///////////////////////////////////////////////
        imu::Vector<3> shoulderAngles = bnoShoulder.getOffsetAngles();
        double shoulderPitch = shoulderAngles[0];
        double shoulderYaw = shoulderAngles[1];
        double shoulderRoll = shoulderAngles[2];
        double elbow = getElbowAngle() - elbowOffset;

        printAngles(shoulderAngles, elbow);

        // get the position from current readings
        imu::Vector<3> kinematicsPosition = getHandPosition(shoulderPitch, shoulderYaw, shoulderRoll, elbow);

        // if we need to rezero the arm, get the new offset values and update zeroed bool
        if (!zeroed) {
          xOffset = kinematicsPosition[0];
          yOffset = kinematicsPosition[1];
          zOffset = kinematicsPosition[2];

          zeroed = true;
        }

        printPosition(kinematicsPosition);

        // update the global position based on the current while considering the offset for start position
        // TODO: how to deal with position
        xPos += kinematicsPosition[0] - xOffset;
        yPos += kinematicsPosition[1] - yOffset;
        zPos += kinematicsPosition[2] - zOffset;

        printPosition(imu::Vector<3>(xPos, yPos, zPos));

        // update the global pitch, yaw roll
        // TODO: how to deal with angles - Note: we may want to average, but will have issues on edges
        // (i.e. 180 vs. 0 -> average to 90 but we want either or, 180 vs -180 -> average to 0 but we want either or)
        // could do something like:
        // `pitch = abs((offsetAngles1[0] + offsetAngles2[0]) / 2 - offsetAngles1[0]) > 5 ? offsetAngles1[0] : (offsetAngles1[0] + offsetAngles2[0]) / 2;`
        pitch = offsetAngles1[0];
        yaw = offsetAngles1[1];
        roll = offsetAngles1[2];

        printAngles(imu::Vector<3> (pitch, yaw, roll));

        // wait for button release then switch to inactive operation
        buttonReleaseCount = 0;
        while (digitalRead(buttonPin) == LOW) {
          buttonReleaseCount++;
          if (buttonReleaseCount > buttonDebounce) {
            state = inactiveOperation;
            zeroed = false;
            break;
          }
        }

        // double idleCount = 0;
        while ((micros() - tStart) < (SAMPLE_RATE * 1000)) {
          // poll until the next sample is ready
          // TODO: Make sure this is running fast enough
          // idleCount++;
        }
        // Serial.println(idleCount);
      }
      break;

    case inactiveOperation:    // Inactive operation - the button is not pressed and the arm will not move
      {
        // indicate inactive operation
        RGBColor(255, 255, 255); // White
        Serial.println("inactive operation");

        // reset all position calculation variables
        bnoWrist1.resetPosition();
        bnoWrist2.resetPosition();

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


////// FUNCTION DECLARATIONS //////

// Axis representatiton: neutral or zeroed position is along the x axis (arm extended straight infront).
// All rotation is according to the right hand rule about the axis (thumb in direction of positive axis,
// fingers curl in direction of positive rotation).
//    y     x
//    |    /
//    |   /
//    |  /
//    | /
//    |/_ _ _ _ _ z


//                                    shoulderPitch   shoulderYaw     shoulderRoll    elbow
imu::Vector<3> getHandPosition(double theta_0, double theta_1, double theta_2, double theta_3) {
  double x = L1*sin(theta_3)*(sin(theta_0)*sin(theta_2) + cos(theta_0)*cos(theta_2)*sin(theta_1)) - (L1 + L2)*(sin(theta_0)*sin(theta_2)*sin(theta_3) - cos(theta_0)*cos(theta_1)*cos(theta_3) + cos(theta_0)*cos(theta_2)*sin(theta_1)*sin(theta_3)) - (L1*cos(theta_0)*cos(theta_1)*2*(cos(theta_3) - 1))/2;
  double y = (L1 + L2)*(cos(theta_0)*sin(theta_2)*sin(theta_3) + cos(theta_1)*cos(theta_3)*sin(theta_0) - cos(theta_2)*sin(theta_0)*sin(theta_1)*sin(theta_3)) - L1*sin(theta_3)*(cos(theta_0)*sin(theta_2) - cos(theta_2)*sin(theta_0)*sin(theta_1)) - (L1*cos(theta_1)*sin(theta_0)*2*(cos(theta_3) - 1))/2;
  double z = L1*cos(theta_1)*cos(theta_2)*sin(theta_3) - (cos(theta_3)*sin(theta_1) + cos(theta_1)*cos(theta_2)*sin(theta_3))*(L1 + L2) + (L1*sin(theta_1)*2*(cos(theta_3) - 1))/2;

  return imu::Vector<3>(x, y, z);
}

//                                     shoulderPitch   shoulderYaw
imu::Vector<3> getElbowPosition(double theta_0, double theta_1) {
  double x = L1*cos(theta_0)*cos(theta_1);
  double y = L1*cos(theta_1)*sin(theta_0);
  double z = -1*L1*sin(theta_1);

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

  // return normalized value converted to radians
  return ((value/65536) * 360 * DEG_TO_RAD);
}

void printAngles(imu::Vector<3> eulerAngles, double elbow) {
  Serial.print("Ang: ");
  Serial.print(eulerAngles[0] * RAD_TO_DEG, 5);
  Serial.print(" , ");
  Serial.print(eulerAngles[1] * RAD_TO_DEG, 5);
  Serial.print(" , ");
  Serial.print(eulerAngles[2] * RAD_TO_DEG, 5);
  if (elbow == -1) {
    Serial.println();
  } else {
    Serial.print(", ");
    Serial.println(elbow * RAD_TO_DEG, 5);
  }
}

void printPosition(imu::Vector<3> position) {
  Serial.print("Pos: ");
  Serial.print(position[0], 5);
  Serial.print(" , ");
  Serial.print(position[1], 5);
  Serial.print(" , ");
  Serial.println(position[2], 5);
}
