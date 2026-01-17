// #include "compass.h"
#include "Wire.h"
#include "cmps-12.h"
#include "accelerometer.h"
#include "inter-i2c.h"


bool accelerometerReady = false;
bool compassReady = false;

uint16_t currentHeading = 0;
uint16_t lastHeading = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");

  // Start I²C bus
  Wire.begin();

  delay(2000);
  compassReady = compass_init();
  accelerometerReady = accelerometer_init();

  Serial.print("Sensors init: ");
  Serial.print("Compass: ");
  Serial.print(compassReady);
  Serial.print(" Gyro: ");
  Serial.println(accelerometerReady);
}


void loop() {
  // float xyz[3];
  currentHeading = readBearing();
  if (lastHeading != currentHeading) {
    Serial.print(" Heading: ");
    Serial.println(currentHeading / 10);
    lastHeading = currentHeading;
  }
  // if (mag.readXYZ(xyz)) {
  //   // Apply soft-iron correction
  //   xyz[0] *= SCALE_AVG / SCALE_X;
  //   xyz[1] *= SCALE_AVG / SCALE_Y;

  //   // Calculate heading with magnetic declination
  //   Serial.print("X:");
  //   Serial.print(xyz[0]);
  //   Serial.print(" Y:");
  //   Serial.print(xyz[1]);
  //   Serial.print(" Z:");
  //   Serial.print(xyz[2]);
  // }
  // if (accelerometerReady && getAccelerometerEuler()) {
  //   Serial.print("euler\t");
  //   Serial.print(euler[0]);
  //   Serial.print("\t");
  //   Serial.print(euler[1]);
  //   Serial.print("\t");
  //   Serial.println(euler[2]);
  //   Serial.print("eulerDeg\t");
  //   Serial.print(eulerDeg[0]);
  //   Serial.print("\t");
  //   Serial.print(eulerDeg[1]);
  //   Serial.print("\t");
  //   Serial.println(eulerDeg[2]);
  //   // Serial.print("areal\t");
  //   // Serial.print(aaReal.x);
  //   // Serial.print("\t");
  //   // Serial.print(aaReal.y);
  //   // Serial.print("\t");
  //   // Serial.println(aaReal.z);
  // } else {
  //   Serial.println("No reading...");
  // }
  uint8_t proximity = getProximityState();
  // if (unoQAvailable) {
  //   sendObstacles(currentHeading);
  //   getDirectionToDrive();
  // }
  if (proximity == 0xFF) {
    Serial.println("Timed out waiting for nano");
  } else if (proximity > 0) {
    if (checkFrontProximity(proximity)) {
      Serial.print("FRONT ");
      if (checkFrontRightProximity(proximity)) Serial.print("RIGHT ");
      if (checkFrontLeftProximity(proximity)) Serial.print("LEFT ");
    }
    if (checkRearProximity(proximity)) {
      Serial.print("REAR ");
      if (checkRearRightProximity(proximity)) Serial.print("RIGHT ");
      if (checkRearLeftProximity(proximity)) Serial.print("LEFT ");
    }
    Serial.println();
  }

  delay(100);
}
