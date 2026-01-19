#include <avr/wdt.h>
#include "Wire.h"

// #include "accelerometer.h"
#include "cmps-12.h"
#include "inter-i2c.h"
#include "robo-car.h"

bool leftGround() {
  return false;
}
#include "motor-driver.h"
#include "movement.h"

int16_t directionToDrive = 0;
int16_t currentDirection = 0;  //This is the direction to rotate to and drive forward in, in degrees. 0 is defined as straightahead when the device was booted

Robot_State currentState = INIT;
Drive_State currentDriveState = STOPPED;

void setup() {
  // Serial.begin(9600);
  delay(3000);

  motor_Init();
  Serial.println("Motor ready");

  Wire.begin();
  bool compassReady = compass_init();
  if (Serial) {
    Serial.print("Compass: ");
    Serial.println(compassReady);
  }
  waitUntilCalibrated();

  calibrateCompass();

  delay(1000);
  if (Serial) Serial.println("Pointing north");
  rotateTo(0);  //Start by pointing north
  delay(3000);
  // Serial.println("Rotating to 90");
  // rotateTo(90);
  // delay(1000);
  // rotateTo(180);
  // delay(1000);
  // rotateTo(270);
  // delay(1000);
  // rotateTo(360);
  // delay(1000);
  // rotateTo(45);
  // delay(1000);
  // rotateTo(135);
  // delay(1000);
  // rotateTo(-135);
  // delay(1000);
  // rotateTo(-45);
  // delay(1000);

  // Serial.println("Pointing north");
  // rotateTo(0);
  // delay(3000);
  if (Serial) Serial.println("U turn...");
  aboutTurn();
  delay(1000);
  if (Serial) Serial.println("U turn...");
  aboutTurn();
  delay(1000);
  if (Serial) Serial.println("U turn...");
  aboutTurn();
  delay(1000);
  if (Serial) Serial.println("U turn...");
  aboutTurn();
  delay(1000);
  rotateTo(0);  //Finish by pointing north
}

void loop() {
}
