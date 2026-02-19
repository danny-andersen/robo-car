// #include "compass.h"
#include <avr/wdt.h>
#include "Wire.h"
#include "robo-car.h"
#include "cmps-12.h"
bool leftGround() {
  return false;
}
#include "distance-sensor.h"
#include "motor-driver.h"

#include "i2c-nano.h"
#include "i2c-pi.h"
#include "proximity.h"
#include "status.h"

bool compassReady = false;

uint16_t currentHeading = 0;
uint16_t lastHeading = 0;

int16_t tempC = 0;
int16_t humidity = 0;
uint16_t volts = 0;

uint16_t furthestDistance = 0;
uint16_t distances[NUMBER_OF_ANGLES_IN_SWEEP];  //Gives a step size of 1 deg
Arc arcs[MAX_NUMBER_OF_OBJECTS_IN_SWEEP];       // up to 20 arcs
uint8_t furthestObjectIndex = 0;


void setup() {
  Serial.begin(115200);
  D_println("Starting...");

  statusInit();
  // Start I²C bus
  Wire.begin();
  Wire.setWireTimeout();

  delay(2000);
  compassReady = compass_init();

  distanceSensorInit();

  motor_Init();
  // D_print("Sensors init: ");
  // D_print("Compass: ");
  // D_println(compassReady);
}

void printNanoStatus() {
  D_print("Nano Proximity: ");
  D_print(nanoStatus.proximityState);
  D_print(" left: ");
  D_print(nanoStatus.currentLeftSpeed);
  D_print(" right: ");
  D_print(nanoStatus.currentRightSpeed);
  D_print(" avg: ");
  D_print(nanoStatus.averageSpeed);
  D_print(" distance: ");
  D_println(nanoStatus.distanceTravelled);
}


void loop() {
  systemStatus.currentBearing = getCompassBearing();
  readAttitude(&systemStatus.pitch, &systemStatus.roll);
  systemStatus.batteryVoltage = getBatteryVoltageInt();
  setStatusLed(systemStatus.batteryVoltage / 100.0);
  D_print("Heading: ");
  D_print(systemStatus.currentBearing);
  D_print(" Pitch: ");
  D_print(systemStatus.pitch);
  D_print(" Roll: ");
  D_print(systemStatus.roll);
  D_print(" Voltage: ");
  D_print(systemStatus.batteryVoltage / 100.0);

  if (getTempHumidityInt(&systemStatus.tempC, &systemStatus.humidity)) {
    D_print(" RH = ");
    D_print(systemStatus.humidity / 10.0);
    D_print("%, T = ");
    D_print(systemStatus.tempC / 10.0);
    D_println(" C");
  } else {
    D_println();
  }

  nanoCommsError = getNanoStatusCmd();
  if (nanoCommsError) {
    D_print("Failed to send to Nano: ");
    D_println(systemStatus.errorField);
  } else {
    printNanoStatus();
  }

  // nanoCommsError = getProximityState();
  if (nanoCommsError) {
    D_print("Failed to send to Nano: ");
    D_println(nanoCommsError);
  } else if (systemStatus.proximityState > 0) {
    if (checkFrontProximity(systemStatus.proximityState)) {
      D_print("Nano FRONT ");
      if (checkFrontRightProximity(systemStatus.proximityState)) D_print("RIGHT ");
      if (checkFrontLeftProximity(systemStatus.proximityState)) D_print("LEFT ");
    }
    if (checkRearProximity(systemStatus.proximityState)) {
      D_print("REAR ");
      if (checkRearRightProximity(systemStatus.proximityState)) D_print("RIGHT ");
      if (checkRearLeftProximity(systemStatus.proximityState)) D_print("LEFT ");
    }
    D_println();
  }

  //Start up motor - I2C is susceptible to noise on the bus
  // driveMotor(FORWARD, 50, 50);
  sendStartMotorCmd();
  D_print("Start motor: ");
  printNanoStatus();

  piCommsError = getPiStatusCmd();
  if (piCommsError) {
    //Failed to get a response
    D_print("Failed to talk to PI: ");
    D_print(piCommsError);
    D_print(" Errorfield: ");
    D_println(systemStatus.errorField);
  } else {
    D_print("PI Ready: ");
    D_print(piStatus.ready);
    D_print(" Lidar: ");
    D_print(piStatus.lidarStatus);
    D_print(" Dirn: ");
    D_println(piStatus.directionToDrive);
    if (piStatus.lidarStatus > 0) {
      if (checkFrontProximity(piStatus.lidarStatus)) {
        D_print("LIDAR FRONT ");
        if (checkFrontRightProximity(piStatus.lidarStatus)) D_print("RIGHT ");
        if (checkFrontLeftProximity(piStatus.lidarStatus)) D_print("LEFT ");
      }
      if (checkRearProximity(piStatus.lidarStatus)) {
        D_print("LIDAR REAR ");
        if (checkRearRightProximity(piStatus.lidarStatus)) D_print("RIGHT ");
        if (checkRearLeftProximity(piStatus.lidarStatus)) D_print("LEFT ");
      }
      D_println();
    }
  }

  if (!piCommsError) {
    //Pi is available
    //OR the lidar proximity status in with the IR proximity sensors
    systemStatus.proximityState |= piStatus.lidarStatus;
  }

  D_print("Combined Proximity: ");
  if (checkFrontProximity(systemStatus.proximityState)) {
    D_print("FRONT ");
    if (checkFrontRightProximity(systemStatus.proximityState)) D_print("RIGHT ");
    if (checkFrontLeftProximity(systemStatus.proximityState)) D_print("LEFT ");
  }
  if (checkRearProximity(systemStatus.proximityState)) {
    D_print("REAR ");
    if (checkRearRightProximity(systemStatus.proximityState)) D_print("RIGHT ");
    if (checkRearLeftProximity(systemStatus.proximityState)) D_print("LEFT ");
  }
  D_println();

  // Send combined status to PI
  int8_t piCommsError = sendSystemStatus();
  if (piCommsError) {
    D_print("Failed to send status to PI: ");
    D_println(piCommsError);
  }

  sweep(distances);
  // Array to hold arcs that represent similar distances, i.e. an object or obstacle in front
  for (int i = 0; i < MAX_NUMBER_OF_OBJECTS_IN_SWEEP; i++) {
    arcs[i].avgDistance = 0;
  }
  for (int i = 0; i < MAX_NUMBER_OF_OBJECTS_IN_SWEEP; i++) {
    arcs[i].avgDistance = 0;
  }
  uint8_t numObjects = findObjectsInSweep(distances, NUMBER_OF_ANGLES_IN_SWEEP, arcs, MAX_NUMBER_OF_OBJECTS_IN_SWEEP);
  // uint8_t numObjects = 5;
  for (int i = 0; i < numObjects; i++) {
    D_print("Rel Dirn: ");
    D_print(arcs[i].centreDirection);
    D_print(" Width: ");
    D_print(arcs[i].width);
    D_print(" Dist: ");
    D_println(arcs[i].avgDistance);
  }
  //Send Obstacles to PI
  int8_t obsStatus = sendObstacles(systemStatus.currentBearing, numObjects, &arcs[0]);
  if (!obsStatus) {
    D_print("Sent all ");
    D_print(numObjects);
    D_println(" obs");

  } else {
    D_print("Failed to send all obs: ");
    D_println(obsStatus);
  }
  driveMotor(STOP, 50, 50);
  sendStopMotorCmd();
  D_print("Stop motor: ");
  printNanoStatus();
  piCommsError = sendSystemStatus();

  delay(30000);
}
