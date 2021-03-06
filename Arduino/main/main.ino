#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"


///// GLOBAL VARIABLES /////
bool calibrated = false;
bool imusOn = false;
bool gripperButtonState = false;
bool homeButtonState = false;
uint16_t buttonCounter = 0;
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
double currXPos = 0, currYPos = 0, currZPos = 0;
double xOffset = 0, yOffset = 0, zOffset = 0, elbowOffset = 0;
bool zeroed = false;
// imu::Quaternion quat = imu::Quaternion(1,0,0,0);


////// CONSTANTS /////
const uint16_t SAMPLE_RATE = 10; // [ms]
const uint16_t mainButtonPin = 10;
const uint16_t gripperButtonPin = 8;
const uint16_t homeButtonPin = 9;
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
// Adafruit_BNO055 bnoWrist = Adafruit_BNO055(55, 0x29, &Wire1);


////// FUNCTION DECLARATIONS /////
double getElbowAngle();
void RGBColor(uint8_t redValue, uint8_t greenValue, uint8_t blueValue);
imu::Vector<3> getPosition(imu::Quaternion shoulderQuat, double elbow);
void printPosition(imu::Vector<3> position);
void sendToPanda(states state, imu::Vector<3> handPosition, bool gripperButtonState, bool homeButtonState); // , imu::Quaternion wristQuat);


///// SETUP /////
void setup(void) {
  Serial.begin(9600);

  state = awaitingCalibration;

  // Button pin setup
  pinMode(mainButtonPin, INPUT);
  pinMode(gripperButtonPin, INPUT);
  pinMode(gripperButtonPin, INPUT);

  // LED pin setup
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
        if(!imusOn) {
          if (!bnoShoulder.begin()) {
            Serial.print("No Shoulder BNO055 detected");
            state = error;
            break;
          }
          bnoShoulder.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P0);
          bnoShoulder.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P0);

          // if (!bnoWrist.begin()) {
          //   Serial.print("No Wrist BNO055 (1) detected");
          //   state = error;
          //   break;
          // }
          // bnoWrist.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P3);
          // bnoWrist.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P3);

          imusOn = true;
        }

        // indicate waiting for calibration
        RGBColor(0, 0, 255); // Blue
        Serial.println("Waiting for calibration");

        // wait for button press and release then switch to calibrating
        buttonPressCount = 0;
        while (digitalRead(mainButtonPin) == HIGH) {
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
          // bnoWrist.calibrate();

          calibrated = true;
        }

        // indicate calibration finished
        RGBColor(0, 255, 255); // Cyan
        Serial.println("calibration done");

        // wait for button press then switch to active operation
        buttonPressCount = 0;
        while (digitalRead(mainButtonPin) == HIGH) {
          buttonPressCount++;
          if (buttonPressCount > buttonDebounce) {
            xPos = 0, yPos = 0, zPos = 0; // Ensure that the global position has been zeroed
            // quat = imu::Quaternion(1,0,0,0)
            state = activeOperation;

            // get Offsets
            bnoShoulder.setQuaternionOffsets();
            // bnoWrist.setQuaternionOffsets();
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
        ////////////// Check Panda State //////////////
        ///////////////////////////////////////////////
        if (Serial.available() > 0) {
          if (Serial.read() == 0) {
            state = error;
            Serial.println("Panda Error: Force Threshold Exceeded");
            break;
          }
        }


        ///////////////////////////////////////////////
        ////////// Update Absolute Positions //////////
        ///////////////////////////////////////////////
        // instantiate time current time
        // tStart = micros();

        // Get readings
        // imu::Quaternion wristQuat = bnoWrist.getOffsetQuat();

        ///////////////////////////////////////////////
        ////////////// Update Positions ///////////////
        ///////////////////////////////////////////////
        imu::Quaternion shoulderQuat = bnoShoulder.getOffsetQuat();
        double elbow = getElbowAngle() - elbowOffset;

        // get the position from current readings
        imu::Vector<3> handPosition = getPosition(shoulderQuat, elbow);

        // if we need to rezero the arm, get the new offset values and update zeroed bool
        if (!zeroed) {
          xOffset = handPosition[0];
          yOffset = handPosition[1];
          zOffset = handPosition[2];

          // quat = wristQuat;

          zeroed = true;
        }

        // update the global position based on the current while considering the offset for start position
        currXPos = xPos + handPosition[0] - xOffset;
        currYPos = yPos + handPosition[1] - yOffset;
        currZPos = zPos + handPosition[2] - zOffset;

        // wristQuat = quat * wristQuat;

        sendToPanda(state, imu::Vector<3>(currXPos, currYPos, currZPos), gripperButtonState, homeButtonState); // , wristQuat);

        ///////////////////////////////////////////////
        //////// Check Gripper Buttons / State ////////
        ///////////////////////////////////////////////
        buttonCounter++;
        if (buttonCounter >= buttonDebounce) {
          gripperButtonState = !digitalRead(gripperButtonPin);

          if (digitalRead(homeButtonPin) == LOW) {
            xPos = 0, yPos = 0, zPos = 0;
            xOffset = handPosition[0], yOffset = handPosition[1], zOffset = handPosition[2];
          }
          buttonCounter = 0;
        }

        // wait for button release then switch to inactive operation
        buttonReleaseCount = 0;
        while (digitalRead(mainButtonPin) == LOW) {
          buttonReleaseCount++;
          if (buttonReleaseCount > buttonDebounce) {
            state = inactiveOperation;
            zeroed = false;
            xPos = currXPos;
            yPos = currYPos;
            zPos = currZPos;
            break;
          }
        }

        // while ((micros() - tStart) < (SAMPLE_RATE * 1000)) {
          // poll until the next sample is ready
          // TODO: Slow down for Panda - Maybe swap to sample at full speed then print att slower speed?
        // }
      }
      break;

    case inactiveOperation:    // Inactive operation - the button is not pressed and the arm will not move
      {
        // indicate inactive operation
        RGBColor(255, 255, 255); // White
        Serial.println("inactive operation");

        // check for button press to switch to active operation
        buttonPressCount = 0;
        while (digitalRead(mainButtonPin) == HIGH ) {
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
        // indicate error
        RGBColor(255, 0, 0); // Red
        while (1) {} // do nothing until Arduino is reset
      }
      break;

  } // end state loop
} // end void loop


////// FUNCTION DECLARATIONS //////

// Axis representatiton: neutral or zeroed position is along the x axis (arm extended straight infront).
// All rotation is according to the right hand rule about the axis (thumb in direction of positive axis,
// fingers curl in direction of positive rotation).
//               z    x
//               |    /
//               |   /
//               |  /
//               | /
//    y_ _ _ _ _ |/


imu::Vector<3> getPosition(imu::Quaternion quat, double elbow) {
  imu::Vector<3> elbowVec = imu::Vector<3> (L1, 0, 0);
  elbowVec = quat.rotateVector(elbowVec);

  imu::Vector<3> handVec = imu::Vector<3> (L2*cos(-1*elbow), L2*sin(-1*elbow), 0);
  handVec = quat.rotateVector(handVec);

  return imu::Vector<3> ((elbowVec[0] + handVec[0]), (elbowVec[1] + handVec[1]), (elbowVec[2] + handVec[2]));
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

void printPosition(imu::Vector<3> position) {
  Serial.print("Pos: ");
  Serial.print(position[0], 5);
  Serial.print(",");
  Serial.print(position[1], 5);
  Serial.print(",");
  Serial.println(position[2], 5);
}

void sendToPanda(states state, imu::Vector<3> handPosition, bool gripperButtonState, bool homeButtonState) { //, imu::Quaternion wristQuat) {
  // Some Quaternion matht to first rotate the quaternion to the correct axis representation around the y axis by 90 degrees
  // We then rotate the quaternion by the inverse of the predefined Panda offsets.
  // wristQuat = imu::Quaternion(0.99512, -1*0.046895, 0.078191, -1*0.03763).inv() * wristQuat;

  Serial.print(handPosition[0]/100, 5);Serial.print(",");   // Hand X
  Serial.print(handPosition[1]/100, 5);Serial.print(",");   // Hand Y
  Serial.print(handPosition[2]/100, 5);Serial.print(",");   // Hand Z
  // Serial.print(wristQuat.w());Serial.print(",");            // Wrist Quat w
  // Serial.print(wristQuat.x());Serial.print(",");            // Wrist Quat x
  // Serial.print(wristQuat.y());Serial.print(",");            // Wrist Quat y
  // Serial.print(wristQuat.z());Serial.print(",");            // Wrist Quat z
  Serial.print(gripperButtonState);Serial.print(",");          // gripper button state - true -> pressed / false -> not pressed
  Serial.println(homeButtonState);                           // Open button state - true -> pressed / false -> not pressed
}
