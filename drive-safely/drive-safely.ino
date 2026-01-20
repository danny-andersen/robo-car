#include <avr/wdt.h>

#include "ground-tracking.h"
#include "Wire.h"
#include "cmps-12.h"
#include "inter-i2c.h"
#include "motor-driver.h"
#include "movement.h"
#include "robo-car.h"
#include "distance-sensor.h"
#include "status.h"

unsigned long driveTimer = 0;
int16_t directionToDrive = 0;
uint16_t currentHeading = 0;    //Current compass heading pointed in
float currentDirectionRad = 0;  //This is the straightahead direction in Radians, used by the motor drive routine to keep dead ahead
Robot_State currentState = INIT;
Drive_State currentDriveState = STOPPED;

uint16_t furthestDistance = 0;
uint16_t distances[NUMBER_OF_ANGLES_IN_SWEEP];  //Gives a step size of 1 deg
Arc arcs[MAX_NUMBER_OF_OBJECTS_IN_SWEEP];       // up to 20 arcs
uint8_t furthestObjectIndex = 0;

bool accelerometerReady = false;
bool compassReady = false;
uint8_t proximitySensors = 0;
bool drivingForward = false;  //Set to true when driving forward

float batteryVoltage = 0.0;

unsigned long statusTimer = 0;

void setup() {
  // Serial.begin(9600);
  groundTrackingInit();
  // Start I²C bus
  Wire.begin();
  statusInit();
  showBatteryStatus();
  delay(3000);  //Let everything settle before initialising accelerometer
  compassReady = compass_init();
  waitUntilCalibrated();
  motor_Init();
  distanceSensorInit(); 
  //Check to see if peripheral nano ready
  proximitySensors = getProximityState();
  // if (!nanoAvailable) {
  // if (Serial) Serial.print("Peripheral Uno not ready");
  // }
  waitUntilCalibrated();
  delay(1000);
  currentHeading = getCompassBearing();
  wdt_enable(WDTO_4S);
  currentState = SWEEP;
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
    currentState = SWEEP;
  }

  statusTimer += LOOP_TIME;
  if (statusTimer > STATUS_TIME) {
    statusTimer = 0;
    batteryVoltage = getBatteryVoltage();
    setStatusLed(batteryVoltage);
    //TODO : send status to PI
  }

  currentHeading = getCompassBearing();
  currentDirectionRad = getCompassBearingRads();

  // if (Serial) {
  //   Serial.print("STATE: ");
  //   Serial.println(currentState);
  // }
  switch (currentState) {
    case SWEEP:
      currentState = sweepAndFindDirection();
      break;
    case ROTATING:
      //Rotate to that direction
      // if (Serial) Serial.println("Rotating...");
      if (!rotateTo(directionToDrive)) {
        //Something wrong with gyro as failed to find correct direction - retry
        currentState = SWEEP;
      } else {
        //Returns when car is pointed in the right direction
        currentState = DRIVE;
      }
      break;
    case UTURN_SWEEP:
      //Turn the robot around and do a sweep
      // if (Serial) Serial.println("U turn...");
      aboutTurn();
      currentState = SWEEP;
      break;
    case BACK_OUT:
      // Reverse for 0.5 second, and do another sweep
      // if (Serial) Serial.println("Reversing");
      backOut();
      currentState = SWEEP;
      break;
    case DRIVE:
      //Drive straight and scan (note that yaw should be reset)
      if (!drivingForward) {
        //Send starting motor to peripheral to start counting wheel rotation
        sendStartMotorCmd();
        drivingForward = true;
      }
      driveAndScan();
      break;
    case INIT_FAILED:
      //Cant do anything
      // if (Serial) Serial.println("Init failed, halting...");
      drive(STOP, currentDirectionRad, 0);
      currentDriveState = STOPPED;
      break;
    case OFF_GROUND:
      sendStopMotorCmd();
      drive(STOP, currentDirectionRad, 0);
      break;
    default:
      // if (Serial) Serial.println("In unknown state...");
      sendStopMotorCmd();
      drive(STOP, currentDirectionRad, 0);
      currentDriveState = STOPPED;
      break;
  }
  delay(LOOP_TIME);
}

Robot_State adjustDirection() {
  Robot_State returnState = ROTATING;
  //Something low down caused the stop
  if (checkFrontRightProximity(proximitySensors) && !checkFrontLeftProximity(proximitySensors)) {
    //Rotate a bit left
    directionToDrive -= 20;

  } else if (checkFrontLeftProximity(proximitySensors) && !checkFrontRightProximity(proximitySensors)) {
    //Rotate a bit to the right
    directionToDrive += 20;
  } else if (checkFrontProximity(proximitySensors)) {
    //Need to backout a little and sweep
    returnState = BACK_OUT;
  }
  return returnState;
}

Robot_State sweepAndFindDirection() {
  // if (Serial) {
  //   Serial.print("Sweeping, straightahead = ");
  //   Serial.println(currentHeading);
  // }
  sweep(distances);
  // Array to hold arcs that represent similar distances, i.e. an object or obstacle in front
  for (int i = 0; i < MAX_NUMBER_OF_OBJECTS_IN_SWEEP; i++) {
    arcs[i].avgDistance = 0;
  }
  int numObjects = findObjectsInSweep(distances, NUMBER_OF_ANGLES_IN_SWEEP, arcs, MAX_NUMBER_OF_OBJECTS_IN_SWEEP);
  SWEEP_STATUS sweepStatus = checkSurroundings(arcs, numObjects, &furthestObjectIndex);
  if (sweepStatus == CLEAR_TO_DRIVE) {
    furthestDistance = arcs[furthestObjectIndex].avgDistance;
    int16_t servoDirection = arcs[furthestObjectIndex].centreDirection;  //This is the degree relative to where we are currently pointed, where 90 is straightahead
    int16_t relDirection = SERVO_CENTRE - servoDirection;                //Straight ahead is 0, +90 is full right, -90 is full left relative to current direction (yaw)
    //Convert to direction based on what the accelerometer things (where 0 is the original starting direction)
    //We need to convert that to a value relative to the accelerometer as it is our only constant point of reference
    directionToDrive = normalise(currentHeading + relDirection);
    currentState = ROTATING;
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
  //Check speed, distance and for any immediate obstructions
  bool status = false;
  do {
    //Try and read a valid proximatey status continuously
    //Eventually the watchdog will trigger if cant get it
    status = getStatusCmd();
  } while (!status);
  bool wheelTrapped = ((periStatus.currentLeftSpeed == 0 || periStatus.currentRightSpeed == 0) && periStatus.distanceTravelled > 10);
  // bool rolled = rollingOrPitching();
  bool rolled = false;
  proximitySensors = getProximityState();
  bool hitSomething = checkFrontProximity(proximitySensors);
  if (wheelTrapped || rolled) {
    ////Weve hit something or one of the wheels aint turning
    currentState = BACK_OUT;
  } else if (hitSomething) {
    currentState = adjustDirection();
  } else if (distanceClear > 100) {
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
    sendStopMotorCmd();
    //Reached the end of the current drive - do another sweep
    drivingForward = false;
    currentState = SWEEP;
  }
}
