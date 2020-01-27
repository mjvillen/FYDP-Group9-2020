#include "Imu.h"
#include "Wire.h"

LSM6DS3 SensorOne(0x6A);
LSM6DS3 SensorTwo(0x6B);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000); //relax...
  Serial.println("Processor came out of reset.\n");
  
  //Call .begin() to configure the IMUs
  if(SensorOne.begin() != 0) {
	  Serial.println("Problem starting the sensor at 0x6A.");
  } else {
	  Serial.println("Sensor at 0x6A started.");
  }

  if(SensorTwo.begin() != 0) {
	  Serial.println("Problem starting the sensor at 0x6B.");
  } else {
	  Serial.println("Sensor at 0x6B started.");
  }
}


void loop() {

  float measurements[3];

  SensorOne.getPitchYawRoll(measurements);

  Serial.print("Pitch: "); Serial.print(measurements[0]);
  Serial.print("\tYaw: "); Serial.print(measurements[1]);
  Serial.print("\tRoll: "); Serial.println(measurements[2]);
}
