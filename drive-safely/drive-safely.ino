#include <avr/wdt.h>

#include "ground-tracking.h"
#include "compass.h"
#include "accelerometer.h"
#include "inter-i2c.h"
#include "motor-driver.h"
#include "robo-car.h"
#include "distance-sensor.h"


unsigned long driveTimer = 0;
int16_t directionToDrive = 0;
int16_t currentDirectionDeg = 0;  //This is the direction to rotate to and drive forward in, in degrees. 0 is defined as straightahead when the device was booted
uint16_t currentHeading = 0;      //Current compass heading pointed in
float currentDirectionRad = 0;    //This is the straightahead direction in Radians, used by the motor drive routine to keep dead ahead
Robot_State currentState = INIT;
Drive_State currentDriveState = STOPPED;

uint16_t furthestDistance = 0;
uint16_t lastDistanceToObstacle = 10000;
uint16_t distances[NUMBER_OF_ANGLES_IN_SWEEP];  //Gives a step size of 1 deg
Arc arcs[MAX_NUMBER_OF_OBJECTS_IN_SWEEP];       // up to 20 arcs
uint8_t furthestObjectIndex = 0;

bool accelerometerReady = false;
bool compassReady = false;
uint8_t proximitySensors = 0;

void setup() {
  // Serial.begin(9600);
  groundTrackingInit();
  // Start I²C bus
  Wire.begin();
  delay(3000);  //Let everything settle before initialising accelerometer
  compassReady = compass_init();
  // if (Serial) {
  //   if (compassReady) {
  //     currentHeading = getHeading();
  //     Serial.print("Compass: ");
  //     Serial.println(currentHeading);
  //   } else {
  //     Serial.println("Compass not available");
  //   }
  // }
  accelerometerReady = accelerometer_init();
  if (!accelerometerReady) {
    // if (Serial) Serial.print("Failed to initialise accelerometer!!");
    currentState = INIT_FAILED;
  } else {
    getAccelerometerEuler();
    currentDirectionDeg = eulerDeg[0];  //Before we work out which direction to turn, remember what straightahead is
    currentState = SWEEP;
    currentHeading = getHeading();
    // if (Serial) {
    //   Serial.print("Gyro straightahead = ");
    //   Serial.print(currentDirectionDeg);
    // }
  }
  motor_Init();
  distanceSensorInit();
  //Check to see if peripheral uno ready
  proximitySensors = getProximityState();
  if (!unoQAvailable) {
    // if (Serial) Serial.print("Peripheral Uno not ready");
  }
  delay(1000);
  wdt_enable(WDTO_4S);
}

// 1. Sweep the area forward and find the furthest distance; if multiple > 200 cm (max reliable distance) then choose one at random
// 2. Rotate car to that direction
// 3. Drive in that direction
// 4. Keep driving and checking distance until obstacle in front is < 20cm
// 5  Goto (1).

void loop() {
  wdt_reset();
  if (leftGround()) {
    // if (Serial) Serial.println("Left ground!");
    currentState = OFF_GROUND;
  } else if (currentState == OFF_GROUND) {
    //Back on the ground - restart from beginning
    resetGyro();
  }
  // if (Serial) {
  //   Serial.print("STATE: ");
  //   Serial.println(currentState);
  // }
  getAccelerometerEuler();            //Note that if this fails, as no more recent data received, then we just use the last calculated value
  currentDirectionDeg = eulerDeg[0];  //Remember what straightahead is
  currentDirectionRad = euler[0];

  if (rollingOrPitching()) {
    //We have run into something or have run over something
    if (currentState == DRIVE) {
      drive(STOP, currentDirectionRad, 0);
      backOut();
      currentState = SWEEP;
    }
  }
  switch (currentState) {
    case SWEEP:
      currentState = sweepAndFindDirection();
      break;
    case ROTATING:
      //Rotate to that direction
      // if (Serial) Serial.println("Rotating...");
      if (!rotateTo(directionToDrive)) {
        //Something wrong with gyro as failed to find correct direction
        resetGyro();
      } else {
        //Returns when car is pointed in the right direction
        currentState = DRIVE;
        lastDistanceToObstacle = 10000;
      }
      break;
    case UTURN_SWEEP:
      //Turn the robot around and do a sweep
      // if (Serial) Serial.println("U turn...");
      aboutTurn();
      currentState = SWEEP;
      break;
    case BACK_OUT:
      // Reverse for 0.5 second, turn around and do another sweep
      // if (Serial) Serial.println("Reversing");
      backOut();
      currentState = UTURN_SWEEP;
      break;
    case DRIVE:
      //Drive straight and scan (note that yaw should be reset)
      driveAndScan();
      if (currentDriveState == STOPPED) {
        if (proximitySensors) {
          adjustDirection();
          currentState = ROTATING;
        } else {
          //Reached the end of the current drive - do another sweep
          currentState = SWEEP;
        }
      }
      break;
    case INIT_FAILED:
      //Cant do anything
      // if (Serial) Serial.println("Init failed, halting...");
      drive(STOP, currentDirectionRad, 0);
      currentDriveState = STOPPED;
      break;
    case OFF_GROUND:
      drive(STOP, currentDirectionRad, 0);
      break;
    default:
      // if (Serial) Serial.println("In unknown state...");
      drive(STOP, currentDirectionRad, 0);
      currentDriveState = STOPPED;
      break;
  }
  delay(LOOP_TIME);
}

void resetGyro() {
    delay(1000);
    wdt_reset();
    accelerometerReady = accelerometer_init();
    wdt_reset();
    currentState = SWEEP;
}

void adjustDirection() {
  //Something low down caused the stop
  if (checkFrontRightProximity(proximitySensors)) {
    //Rotate a bit left
    directionToDrive -= 20;
  } else if (checkFrontLeftProximity(proximitySensors)) {
    //Rotate a bit to the right
    directionToDrive += 20;
  }
}

Robot_State sweepAndFindDirection() {
  // if (Serial) {
  //   Serial.print("Sweeping, straightahead = ");
  //   Serial.println(currentDirectionDeg);
  // }
  sweep(distances);
  // Array to hold arcs that represent similar distances, i.e. an object or obstacle in front
  for (int i = 0; i < MAX_NUMBER_OF_OBJECTS_IN_SWEEP; i++) {
    arcs[i].avgDistance = 0;
  }
  int numObjects = findObjectsInSweep(distances, NUMBER_OF_ANGLES_IN_SWEEP, arcs, MAX_NUMBER_OF_OBJECTS_IN_SWEEP);
  // if (Serial) {
  //   Serial.println("Arcs found:");
  //   for (int j = 0; j < numObjects; j++) {
  //     if (arcs[j].avgDistance == 0) {
  //       //null entry
  //       continue;
  //     }
  //     Serial.print("Arc ");
  //     Serial.print(j);
  //     Serial.print(": start=");
  //     Serial.print(arcs[j].startIndex);
  //     Serial.print(", end=");
  //     Serial.print(arcs[j].endIndex);
  //     Serial.print(", width=");
  //     Serial.print(arcs[j].width);
  //     Serial.print(", center=");
  //     Serial.print(arcs[j].centerIndex);
  //     Serial.print(", avg=");
  //     Serial.println(arcs[j].avgDistance);
  //   }
  // }
  SWEEP_STATUS sweepStatus = checkSurroundings(arcs, numObjects, &furthestObjectIndex);
  // if (Serial) {
  //   Serial.print("Sweep status: ");
  //   Serial.println(sweepStatus);
  // }
  if (sweepStatus == CLEAR_TO_DRIVE) {
    furthestDistance = arcs[furthestObjectIndex].avgDistance;
    int16_t servoDirection = arcs[furthestObjectIndex].centreDirection;  //This is the degree relative to where we are currently pointed, where 90 is straightahead
    int16_t relDirection = SERVO_CENTRE - servoDirection;            //Straight ahead is 0, +90 is full right, -90 is full left relative to current direction (yaw)
    //Convert to direction based on what the accelerometer things (where 0 is the original starting direction)
    //We need to convert that to a value relative to the accelerometer as it is our only constant point of reference
    directionToDrive = currentDirectionDeg + relDirection;
    currentState = ROTATING;
    // if (Serial) {
    //   Serial.print("Distance to drive: ");
    //   Serial.print(furthestDistance);
    //   Serial.print(" in servo direction: ");
    //   Serial.print(servoDirection);
    //   Serial.print(" in Direction: ");
    //   Serial.print(directionToDrive);
    //   Serial.print(" Current direction: ");
    //   Serial.println(currentDirectionDeg);
    // }
  } else if (sweepStatus == BLOCKED_AHEAD) {
    //Turn 180 and sweep
    currentState = UTURN_SWEEP;
  } else if (sweepStatus == CANNOT_TURN) {
    currentState = BACK_OUT;
  }
  return currentState;
}

void driveAndScan() {
  uint16_t distanceClear = clearDistanceAhead();
  //Also check for any immediate obstructions, lower down
  proximitySensors = getProximityState();
  if (checkFrontProximity(proximitySensors)) {
    drive(STOP, currentDirectionRad, 0);
    currentDriveState = STOPPED;
  } else {
    // if (abs(distanceClear - lastDistanceToObstacle) < 3) {
    //   //Stuck on something - reverse
    //   drive(BACK, 50);
    //   delay(1000);
    //   drive(STOP, 0);
    //   currentDriveState = STOPPED;
    //   return;
    // }
    if (distanceClear > 100) {
      //Charge!
      drive(FORWARD, currentDirectionRad, 125);
      currentDriveState = DRIVE_FORWARD;
    } else if (distanceClear < 100 && distanceClear > 50) {
      //Slow
      drive(FORWARD, currentDirectionRad, 75);
      currentDriveState = DRIVE_FORWARD;
    } else if (distanceClear > MIN_DISTANCE_TO_MOVE) {
      //Dead slow
      drive(FORWARD, currentDirectionRad, 50);
      currentDriveState = DRIVE_FORWARD;
    } else {
      //Stop!
      drive(STOP, currentDirectionRad, 0);
      currentDriveState = STOPPED;
    }
  }
  lastDistanceToObstacle = distanceClear;
}
