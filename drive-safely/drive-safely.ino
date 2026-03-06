#include <avr/wdt.h>

#include "Wire.h"

#include "robo-car.h"
#include "ground-tracking.h"
#include "distance-sensor.h"
#include "cmps-12.h"
#include "nano-comms.h"
#include "pi-comms.h"
#include "proximity.h"
#include "motor-driver.h"
#include "movement.h"
#include "status.h"

int16_t directionToDrive = 0;
uint8_t distanceToDrive = 0;
float currentDirectionRad = 0;  //This is the straightahead direction in Radians, used by the motor drive routine to keep dead ahead
Drive_State currentDriveState = STOPPED;

bool drivingForward = false;  //Set to true when driving forward

unsigned long statusTimer = 0; //Total time since it was last booted
unsigned long lastLoopTime = 0;  //Last time that we went through a loop

void setup() {
  Serial.begin(115200);
  D_println(F("Starting..."));
  // Serial.begin(9600);
  systemStatus.robotState = INIT;
  groundTrackingInit();
  statusInit();
  // Start I²C bus
  Wire.begin();
  showBatteryStatus();
  delay(3000);  //Let everything settle before initialising accelerometer
  compass_init();
  waitUntilCalibrated();
  motor_Init();
  distanceSensorInit();
  //Wait until nano and pi peripherals are ready
  do {
    getCombinedProximity();
    D_println(piCommsError);
    delay(100);
  } while (piCommsError || nanoCommsError);
  //Reset movement stats
  sendStartMotorCmd();
  sendStopMotorCmd();
  //Allow time for environment to clear
  delay(5000);
  systemStatus.currentBearing = getCompassBearing();
  //Send init status to PI for logging
  updateStatus();
  statusTimer = 0;
  wdt_enable(WDTO_4S);

  systemStatus.robotState = SWEEP;
}

// 1. Sweep the area forward and find the furthest distance; if multiple > 200 cm (max reliable distance) then choose one at random
// 2. Rotate car to that direction
// 3. Drive in that direction
// 4. Keep driving and checking distance until obstacle in front is < 20cm
// 5  Goto (1).

void loop() {
  lastLoopTime = millis();
  wdt_reset();
  if (leftGround()) {
    // if (Serial) Serial.println("Left ground!");
    systemStatus.robotState = OFF_GROUND;
    sendStopMotorCmd();
    currentDriveState = STOPPED;
    drive(STOP, currentDirectionRad, 0);

  } else if (systemStatus.robotState == OFF_GROUND) {
    //Back on the ground - restart from beginning
    systemStatus.robotState = SWEEP;
  }

  // readBearingAndAttitude(&systemStatus.currentBearing, &systemStatus.pitch, &systemStatus.roll);
  systemStatus.currentBearing = getCompassBearing();
  currentDirectionRad = getCompassBearingRads();
  readAttitude(&(systemStatus.pitch), &(systemStatus.roll));

  if (statusTimer > STATUS_TIME) {
    statusTimer = 0;
    updateStatus();
  } else if (lastRobotState != systemStatus.robotState) {
    sendSystemStatus();
    lastRobotState = systemStatus.robotState;
  }

  // if (Serial) {
  //   Serial.print("STATE: ");
  //   Serial.println(currentState);
  // }
  switch (systemStatus.robotState) {
    case SWEEP:
      sweepAndRequestDirection();
      break;
    case WAITING_FOR_DIRECTION:
      getDirectionToDrive();
      break;
    case ROTATING:
      //Rotate to that direction
      // if (Serial) Serial.println("Rotating...");
      if (!rotateTo(directionToDrive)) {
        //Something wrong with gyro as failed to find correct direction - retry
        systemStatus.robotState = SWEEP;
      } else {
        //Returns when robot is pointed in the right direction
        systemStatus.robotState = DRIVE;
      }
      break;
    case UTURN_SWEEP:
      //Turn the robot around and do a sweep
      // if (Serial) Serial.println("U turn...");
      aboutTurn();
      systemStatus.robotState = SWEEP;
      break;
    case BACK_OUT:
      // Reverse for 0.5 second, and do another sweep
      // if (Serial) Serial.println("Reversing");
      backOut();
      systemStatus.robotState = SWEEP;
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
    case OFF_GROUND:
      break;
    default:
      // if (Serial) Serial.println("In unknown state...");
      sendStopMotorCmd();
      currentDriveState = STOPPED;
      drive(STOP, currentDirectionRad, 0);
      systemStatus.robotState = STOPPED;
      break;
  }
  unsigned long loopTaken = (millis() - lastLoopTime);
  D_println(loopTaken);
  statusTimer += loopTaken;
  if (loopTaken < LOOP_TIME) {
    unsigned long delayTime = LOOP_TIME - loopTaken;
    delay(delayTime);
    statusTimer += delayTime;
  } 
}

Robot_State adjustDirection() {
  Robot_State returnState = ROTATING;
  //Something low down caused the stop
  if (checkFrontRightProximity(systemStatus.proximityState) && !checkFrontLeftProximity(systemStatus.proximityState)) {
    //Rotate a bit left
    directionToDrive -= 20;

  } else if (checkFrontLeftProximity(systemStatus.proximityState) && !checkFrontRightProximity(systemStatus.proximityState)) {
    //Rotate a bit to the right
    directionToDrive += 20;
  } else if (checkFrontProximity(systemStatus.proximityState)) {
    //Need to backout a little and sweep
    drivingForward = false;
    currentDriveState = STOPPED;
    sendStopMotorCmd();
    returnState = BACK_OUT;
  }
  return returnState;
}

void getDirectionToDrive() {
  getPiStatusCmd();
  D_print(F("Direction: "));
  D_println(piStatus.directionToDrive);
  if (piStatus.directionToDrive == NO_SAFE_DIRECTION) {
    //PI cannot determine a safe direction - determine next step locally
    getFallBackDirection();
    sendSystemStatus();
  } else if (piStatus.directionToDrive != NO_DIRECTION) {
    directionToDrive = piStatus.directionToDrive;
    distanceToDrive = piStatus.distanceToDrive;
    systemStatus.robotState = ROTATING;
    sendSystemStatus();
  }
}

void sweepAndRequestDirection() {
  // if (Serial) {
  //   Serial.print("Sweeping, straightahead = ");
  //   Serial.println(currentHeading);
  // }
  sweep(distances);
  // Array to hold arcs that represent similar distances, i.e. an object or obstacle in front
  for (int i = 0; i < MAX_NUMBER_OF_OBJECTS_IN_SWEEP; i++) {
    arcs[i].avgDistance = 0;
  }
  numObjects = findObjectsInSweep(distances, NUMBER_OF_ANGLES_IN_SWEEP, arcs, MAX_NUMBER_OF_OBJECTS_IN_SWEEP);
  //Send list of obstacles to PI
  sendObstacles(systemStatus.currentBearing, numObjects, &arcs[0]);
  //Update robot state and send status, to trigger PI to determine next move to make
  piStatus.directionToDrive = 1000; //Reset current direction to drive
  systemStatus.robotState = WAITING_FOR_DIRECTION;
  sendSystemStatus();
}

void getFallBackDirection() {
  //The following is called when the PI cannot determine the next safe move, so a default move based on local info is determined (which maybe to backou)
  SWEEP_STATUS sweepStatus = checkSurroundings(arcs, numObjects, &furthestObjectIndex);
  if (sweepStatus == CLEAR_TO_DRIVE) {
    distanceToDrive = arcs[furthestObjectIndex].avgDistance - MIN_DISTANCE_TO_MOVE;  //Distance to move is slightly less than nearest object
    int16_t servoDirection = arcs[furthestObjectIndex].centreDirection;              //This is the degree relative to where we are currently pointed, where 90 is straightahead
    int16_t relDirection = SERVO_CENTRE - servoDirection;                            //Straight ahead is 0, +90 is full right, -90 is full left relative to current direction (yaw)
    //Convert to direction based on what the accelerometer things (where 0 is the original starting direction)
    //We need to convert that to a value relative to the accelerometer as it is our only constant point of reference
    directionToDrive = normalise(systemStatus.currentBearing + relDirection);
    systemStatus.robotState = ROTATING;
  } else if (sweepStatus == BLOCKED_AHEAD) {
    //Turn 180 and sweep
    systemStatus.robotState = UTURN_SWEEP;
  } else if (sweepStatus == CANNOT_TURN) {
    systemStatus.robotState = BACK_OUT;
  }
}

void driveAndScan() {
  uint16_t distanceClear = clearDistanceAhead();
  //Check speed, distance and for any immediate obstructions
  getCombinedProximity();
  bool wheelTrapped = ((systemStatus.leftWheelSpeed == 0 || systemStatus.rightWheelSpeed == 0) && systemStatus.distanceTravelled > 10);
  // bool wheelTrapped = false;
  // bool rolled = rollingOrPitching();
  bool rolled = false;
  bool hitSomething = checkFrontProximity(systemStatus.proximityState);
  if (wheelTrapped || rolled) {
    ////Weve hit something or one of the wheels aint turning
    systemStatus.robotState = BACK_OUT;
    sendStopMotorCmd();
    currentDriveState = STOPPED;
    drivingForward = false;
  } else if (hitSomething) {
    systemStatus.robotState = adjustDirection();
  } else if (systemStatus.distanceTravelled >= distanceToDrive || distanceClear <= MIN_DISTANCE_TO_MOVE) {
    //Driven far enough - stop
    drive(STOP, currentDirectionRad, 0);
    currentDriveState = STOPPED;
    sendStopMotorCmd();
    //Reached the end of the current drive - do another sweep
    drivingForward = false;
    systemStatus.robotState = SWEEP;
  } else if (distanceClear > 100) {
    //Lots of space
    drive(FORWARD, currentDirectionRad, 100);
    currentDriveState = DRIVE_FORWARD;
  } else if (distanceClear < 100 && distanceClear > 50) {
    //Slow
    drive(FORWARD, currentDirectionRad, 75);
    currentDriveState = DRIVE_FORWARD;
  } else {
    //Dead slow
    drive(FORWARD, currentDirectionRad, 50);
    currentDriveState = DRIVE_FORWARD;
  }
}
