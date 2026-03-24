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


unsigned long statusTimer = 0; //Total time since it was last booted
unsigned long lastLoopTime = 0;  //Last time that we went through a loop

int rotation_cnt = 0;

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
  //Allow time for environment to clear
  delay(5000);
  //Rotate through 360 to help the compass to calibrate
  // aboutTurn();
  //Start by pointing due north
  // rotateTo(0);
  delay(1000);
  systemStatus.currentBearing = getCompassBearing();
  //Send init status to PI for logging
  updateStatus();
  statusTimer = 0;
  wdt_enable(WDTO_4S);

  systemStatus.robotState = SWEEP;
}


//Sweep (which tells the PI to do a LIDAR scan
//and then request direction to drive, which waits for PI to process and save the scan
//Wait 10 seconds - and then rotate the robot +45 degrees and repeat
//After 8 rotations, drive to 0 for 30cm

void loop() {
  lastLoopTime = millis();
  wdt_reset();
  if (leftGround()) {
    // if (Serial) Serial.println("Left ground!");
    systemStatus.robotState = OFF_GROUND;
  } else if (systemStatus.robotState == OFF_GROUND) {
    //Back on the ground - restart from beginning
    systemStatus.robotState = SWEEP;
  }

  systemStatus.currentBearing = getCompassBearing();
  currentDirectionRad = getCompassBearingRads();

  if (statusTimer > STATUS_TIME) {
    statusTimer = 0;
    updateStatus();
  } else if (lastRobotState != systemStatus.robotState) {
    sendSystemStatus();
    lastRobotState = systemStatus.robotState;
  }

  D_print(F("STATE: "));
  D_println(systemStatus.robotState);
  switch (systemStatus.robotState) {
    case SWEEP:
      sweepAndRequestDirection();
      break;
    case WAITING_FOR_DIRECTION:
      getDirectionToDrive();
      break;
    case ROTATING:
      //Rotate to that direction
      rotateTo(directionToDrive);
      if (rotation_cnt > 0) {
        systemStatus.robotState = SWEEP;
      } else {
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
      //Do nothing
      break;
  }

  if (systemStatus.robotState == ROTATING) {
    if (rotation_cnt < 8) {
      //Ignore what the PI returned - just rotate +45 degrees in this test
      directionToDrive = normalise(systemStatus.currentBearing + 45);
      // directionToDrive = 0;
      rotation_cnt++;
    } else {
      //Drive north for 30cm
      directionToDrive = 0;
      distanceToDrive = 30;
      rotation_cnt = 0;
    }
    //Wait a bit to allow test to be terminated
    delay(3000);
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
  D_print("Bearing 1: ");
  D_println(systemStatus.currentBearing);
  sweep();
  D_print("Bearing 1b: ");
  D_println(systemStatus.currentBearing);
  // Array to hold arcs that represent similar distances, i.e. an object or obstacle in front
  for (int i = 0; i < MAX_NUMBER_OF_OBJECTS_IN_SWEEP; i++) {
    arcs[i].avgDistance = 0;
  }
  D_print("Bearing 1c: ");
  D_println(systemStatus.currentBearing);
  numObjects = findObjectsInSweep();
  //Send list of obstacles to PI
  D_print("Bearing 2: ");
  D_println(systemStatus.currentBearing);
  sendObstacles(systemStatus.currentBearing, numObjects, &arcs[0]);
  //Update robot state and send status, to trigger PI to determine next move to make
  systemStatus.robotState = WAITING_FOR_DIRECTION;
  D_print("Bearing 3: ");
  D_println(systemStatus.currentBearing);
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