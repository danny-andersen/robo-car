#include <avr/wdt.h>
#include "accelerometer.h"
#include "robo-car.h"

bool leftGround() {
  return false;
}

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
  
  accelerometer_init();
  getAccelerometerEuler();
  currentDirectionDeg = eulerDeg[0];  //Before we work out which direction to turn, remember what straightahead is

  distanceSensorInit();

  currentState = sweepAndFindDirection();
}

void loop() {

}

Robot_State sweepAndFindDirection() {
  if (Serial) {
    Serial.print("Sweeping, straightahead = ");
    Serial.println(currentDirectionDeg);
  }
  sweep(distances);
  for (int i = 0; i<180; i++) {
    Serial.print(i);
    Serial.print(" : ");
    Serial.println(distances[i]);
  }
  // Array to hold arcs that represent similar distances, i.e. an object or obstacle in front
  for (int i = 0; i < MAX_NUMBER_OF_OBJECTS_IN_SWEEP; i++) {
    arcs[i].avgDistance = 0;
  }
  int numObjects = findObjectsInSweep(distances, NUMBER_OF_ANGLES_IN_SWEEP, arcs, MAX_NUMBER_OF_OBJECTS_IN_SWEEP);
  if (Serial) {
    Serial.println("Arcs found:");
    for (int j = 0; j < numObjects; j++) {
      if (arcs[j].avgDistance == 0) {
        //null entry
        continue;
      }
      Serial.print("Arc ");
      Serial.print(j);
      Serial.print(": start=");
      Serial.print(arcs[j].startIndex);
      Serial.print(", end=");
      Serial.print(arcs[j].endIndex);
      Serial.print(", width=");
      Serial.print(arcs[j].width);
      Serial.print(", center=");
      Serial.print(arcs[j].centreDirection);
      Serial.print(", avg=");
      Serial.println(arcs[j].avgDistance);
    }
  }
  SWEEP_STATUS sweepStatus = checkSurroundings(arcs, numObjects, &furthestObjectIndex);
  if (Serial) {
    Serial.print("Sweep status: ");
    Serial.println(sweepStatus);
  }
  if (sweepStatus == CLEAR_TO_DRIVE) {
    furthestDistance = arcs[furthestObjectIndex].avgDistance;
    int16_t servoDirection = arcs[furthestObjectIndex].centreDirection;  //This is the degree relative to where we are currently pointed, where 90 is straightahead
    int16_t relDirection = SERVO_CENTRE - servoDirection;            //Straight ahead is 0, +90 is full right, -90 is full left relative to current direction (yaw)
    //Convert to direction based on what the accelerometer things (where 0 is the original starting direction)
    //We need to convert that to a value relative to the accelerometer as it is our only constant point of reference
    directionToDrive = currentDirectionDeg + relDirection;
    currentState = ROTATING;
    if (Serial) {
      Serial.print("Distance to drive: ");
      Serial.print(furthestDistance);
      Serial.print(" in servo direction: ");
      Serial.print(servoDirection);
      Serial.print(" in Direction: ");
      Serial.print(directionToDrive);
      Serial.print(" Current direction: ");
      Serial.println(currentDirectionDeg);
    }
  } else if (sweepStatus == BLOCKED_AHEAD) {
    //Turn 180 and sweep
    currentState = UTURN_SWEEP;
  } else if (sweepStatus == CANNOT_TURN) {
    currentState = BACK_OUT;
  }
  return currentState;
}

