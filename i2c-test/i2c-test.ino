// #include "compass.h"
#include <avr/wdt.h>
#include "Wire.h"
#include "cmps-12.h"
#include "robo-car.h"
bool leftGround() {
  return false;
}
#include "distance-sensor.h"

#include "inter-i2c.h"
#include "status.h"


bool compassReady = false;
int8_t piOnBus = 1;    //0 = on bus, any other value is an error
int8_t nanoOnBus = 1;  //0 = on bus, any other value is an error

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
  Serial.begin(9600);
  Serial.println("Starting...");

  statusInit();
  // Start I²C bus
  Wire.begin();

  delay(2000);
  compassReady = compass_init();

  distanceSensorInit();

  Serial.print("Sensors init: ");
  Serial.print("Compass: ");
  Serial.println(compassReady);
}

void loop() {
  systemStatus.currentBearing = getCompassBearing();
  readAttitude(&systemStatus.pitch, systemStatus.roll);
  systemStatus.batteryVoltage = getBatteryVoltageInt();
  setStatusLed(systemStatus.batteryVoltage / 100.0);
  Serial.print("Heading: ");
  Serial.print(systemStatus.currentBearing);
  Serial.print(" Pitch: ");
  Serial.print(systemStatus.pitch);
  Serial.print(" Roll: ");
  Serial.print(systemStatus.roll);
  Serial.print(" Voltage: ");
  Serial.print(systemStatus.batteryVoltage / 100.0);

  if (getTempHumidityInt(&systemStatus.tempC, &systemStatus.humidity)) {
    Serial.print(" RH = ");
    Serial.print(systemStatus.humidity / 10.0);
    Serial.print("%, T = ");
    Serial.print(systemStatus.tempC / 10.0);
    Serial.println(" C");
  } else {
    Serial.println();
  }


  nanoOnBus = getStatusCmd();
  if (nanoOnBus) {
    Serial.print("Failed to send to Nano: ");
    Serial.println(nanoOnBus);
  } else {
    Serial.print("Nano Proximity: ");
    Serial.print(periStatus.proximityState);
    Serial.print(" avg speed: ");
    Serial.print(periStatus.averageSpeed);
    Serial.print(" distance: ");
    Serial.println(periStatus.distanceTravelled);
  }

  // nanoOnBus = getProximityState();
  if (nanoOnBus) {
    Serial.print("Failed to send to Nano: ");
    Serial.println(nanoOnBus); 
  } else if (systemStatus.proximityState > 0) {
    if (checkFrontProximity(systemStatus.proximityState)) {
      Serial.print("Nano FRONT ");
      if (checkFrontRightProximity(systemStatus.proximityState)) Serial.print("RIGHT ");
      if (checkFrontLeftProximity(systemStatus.proximityState)) Serial.print("LEFT ");
    }
    if (checkRearProximity(systemStatus.proximityState)) {
      Serial.print("REAR ");
      if (checkRearRightProximity(systemStatus.proximityState)) Serial.print("RIGHT ");
      if (checkRearLeftProximity(systemStatus.proximityState)) Serial.print("LEFT ");
    }
    Serial.println();
  }

  // Send status to PI
  int8_t piOnBus = sendSystemStatus();
  if (piOnBus) {
    Serial.print("Failed to send system status to PI: ");
    Serial.println(piOnBus);
  }

  piOnBus = getPiStatusCmd();
  if (piOnBus) {
    //Failed to get a response
    Serial.print("Failed to talk to PI: ");
    Serial.println(piOnBus);
  } else {
    Serial.print("PI Ready: ");
    Serial.print(piStatus.ready);
    Serial.print(" Lidar: ");
    Serial.print(piStatus.lidarStatus);
    Serial.print(" Direction: ");
    Serial.println(piStatus.directionToDrive);
    if (piStatus.lidarStatus > 0) {
      if (checkFrontProximity(piStatus.lidarStatus)) {
        Serial.print("LIDAR FRONT ");
        if (checkFrontRightProximity(piStatus.lidarStatus)) Serial.print("RIGHT ");
        if (checkFrontLeftProximity(piStatus.lidarStatus)) Serial.print("LEFT ");
      }
      if (checkRearProximity(piStatus.lidarStatus)) {
        Serial.print("LIDAR REAR ");
        if (checkRearRightProximity(piStatus.lidarStatus)) Serial.print("RIGHT ");
        if (checkRearLeftProximity(piStatus.lidarStatus)) Serial.print("LEFT ");
      }
      Serial.println();
    }
  }

  if (!getPiStatusCmd()) {
    //Pi is available
    //OR the lidar proximity status in with the IR proximity sensors
    systemStatus.proximityState |= piStatus.lidarStatus;
  }

  Serial.print("Combined Proximity: ");
  if (checkFrontProximity(systemStatus.proximityState)) {
    Serial.print("FRONT ");
    if (checkFrontRightProximity(systemStatus.proximityState)) Serial.print("RIGHT ");
    if (checkFrontLeftProximity(systemStatus.proximityState)) Serial.print("LEFT ");
  }
  if (checkRearProximity(systemStatus.proximityState)) {
    Serial.print("REAR ");
    if (checkRearRightProximity(systemStatus.proximityState)) Serial.print("RIGHT ");
    if (checkRearLeftProximity(systemStatus.proximityState)) Serial.print("LEFT ");
  }
  systemStatus.proximityState |= piStatus.lidarStatus;
  Serial.println();


  // sweep(distances);
  // // Array to hold arcs that represent similar distances, i.e. an object or obstacle in front
  // for (int i = 0; i < MAX_NUMBER_OF_OBJECTS_IN_SWEEP; i++) {
  //   arcs[i].avgDistance = 0;
  // }
  // for (int i = 0; i < MAX_NUMBER_OF_OBJECTS_IN_SWEEP; i++) {
  //   arcs[i].avgDistance = 0;
  // }
  // uint8_t numObjects = findObjectsInSweep(distances, NUMBER_OF_ANGLES_IN_SWEEP, arcs, MAX_NUMBER_OF_OBJECTS_IN_SWEEP);
  // // uint8_t numObjects = 5;
  // for (int i = 0; i < numObjects; i++) {
  //   Serial.print("Rel Dirn: ");
  //   Serial.print(arcs[i].centreDirection);
  //   Serial.print(" Width: ");
  //   Serial.print(arcs[i].width);
  //   Serial.print(" Distance: ");
  //   Serial.println(arcs[i].avgDistance);
  // }
  // //Send Obstacles to PI
  // int8_t obsStatus = sendObstacles(systemStatus.currentBearing, numObjects, &arcs[0]);
  // if (!obsStatus) {
  //   Serial.print("Sent all ");
  //   Serial.print(numObjects);
  //   Serial.println(" obstacles");

  // } else {
  //   Serial.print("Failed to send all obstacles: ");
  //   Serial.println(obsStatus);
  // }

  delay(1000);
}
