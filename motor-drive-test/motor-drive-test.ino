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

bool accelerometerReady = false;
int driveCount = 0;
unsigned long driveTimer = 0;
unsigned long DRIVETIME = 3000;
unsigned long DELAYTIME = 100;
float forwardDirectionRad = 0;  //This is the straightahead direction in Radians, used by the motor drive routine to keep dead ahead
float forwardRad = 0;
bool forward = true;
bool driving = false;

void setup() {
  Serial.begin(115200);
  systemStatus.robotState = INIT;
  // Start I²C bus
  Wire.begin();
  Wire.setWireTimeout(10000);  //10ms
  delay(3000);                 //Let everything settle before initialising accelerometer
  statusInit();
  showBatteryStatus();
  groundTrackingInit();
  compass_init();
  waitUntilCalibrated();
  motor_Init();
  //Wait until nano and pi peripherals are ready
  do {
    getCombinedProximity();
    D_println(piCommsError);
    delay(100);
  } while (piCommsError || nanoCommsError);

  delay(1000);
  wdt_enable(WDTO_4S);
  //Remember the direction straightahead to drive to
  forwardDirectionRad = getCompassBearingRads();
  systemStatus.currentBearing = getCompassBearing();
  updateStatus();
}

// Drive motor forward and then backward to ensure yaw correction is correct - do this 5 times

void loop() {
  wdt_reset();
  if (driveCount > 5) {
    //Test over
    drive(STOP, forwardDirectionRad, 0);
    delay(DELAYTIME);
    return;
  }
  systemStatus.currentBearing = getCompassBearing();
  if (leftGround()) {
    // if (Serial) Serial.println("Left ground!");
    systemStatus.robotState = OFF_GROUND;
    sendStopMotorCmd();
    drive(STOP, forwardDirectionRad, 0);
    driveTimer = 0;
    driveCount = 0;
    forward = true;
    driving = false;
    sendStopMotorCmd();
    getNanoStatusCmd();
    sendSystemStatus();
  } else {
    systemStatus.robotState = forward ? DRIVE : BACK_OUT;
    if (!driving) {
      driving = true;
      sendStartMotorCmd();
    }
    getNanoStatusCmd();
    if (forward) {
      drive(FORWARD, forwardDirectionRad, 125);
    } else {
      drive(BACK, forwardDirectionRad, 125);
    }
  }
  if (systemStatus.robotState != OFF_GROUND) sendSystemStatus();
  delay(DELAYTIME);

  driveTimer += DELAYTIME;
  if (driveTimer > DRIVETIME) {
    forward = !forward;
    driveTimer = 0;
    driveCount += 1;
    driving = false;
    drive(STOP, forwardDirectionRad, 0);
    sendStopMotorCmd();
    getNanoStatusCmd();
    updateStatus();
    delay(500); //Let it come to a stop
  }
}
