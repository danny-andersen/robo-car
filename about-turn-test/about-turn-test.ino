#include <avr/wdt.h>
#include "accelerometer.h"
#include "inter-i2c.h"
#include "robo-car.h"

bool leftGround() {
  return false;
}
#include "motor-driver.h"
#include "distance-sensor.h"

uint16_t furthestDistance = 0;
uint16_t lastDistanceToObstacle = 10000;
uint16_t distances[NUMBER_OF_ANGLES_IN_SWEEP];  //Gives a step size of 1 deg
Arc arcs[MAX_NUMBER_OF_OBJECTS_IN_SWEEP];       // up to 20 arcs
uint8_t furthestObjectIndex = 0;

int16_t directionToDrive = 0;
int16_t currentDirectionDeg = 0;  //This is the direction to rotate to and drive forward in, in degrees. 0 is defined as straightahead when the device was booted

Robot_State currentState = INIT;
Drive_State currentDriveState = STOPPED;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  accelerometer_init();
  getAccelerometerEuler();
  currentDirectionDeg = eulerDeg[0];  //Before we work out which direction to turn, remember what straightahead is

  motor_Init();
  distanceSensorInit();

  // rotateTo(90);
  // delay(1000);
  // rotateTo(180);
  // delay(1000);
  // rotateTo(270);
  // delay(1000);
  // rotateTo(360);

  if (Serial) Serial.println("U turn...");
  aboutTurn();
  delay(1000);
  aboutTurn();
  delay(1000);
  aboutTurn();
  delay(1000);
  aboutTurn();

  }

void loop() {

}
