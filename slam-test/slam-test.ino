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
  //Allow time for environment to clear
  delay(5000);
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
      systemStatus.robotState = SWEEP;
      break;
    case OFF_GROUND:
      //Do nothing
      break;
  }

  if (systemStatus.robotState == ROTATING) {
    //Ignore what the PI returned - just rotate +45 degrees in this test
    directionToDrive = normalise(systemStatus.currentBearing + 45);
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

