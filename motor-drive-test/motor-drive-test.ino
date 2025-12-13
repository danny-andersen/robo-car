#include "accelerometer.h"
#include "motor-driver.h"

bool accelerometerReady = false;
int driveCount = 0;
unsigned long driveTimer = 0;
unsigned long DRIVETIME = 2000;
unsigned long DELAYTIME = 50;
float forwardRad = 0;

void setup() {
  // Serial.begin(9600);
  delay(3000);  //Let everything settle before initialising accelerometer
  accelerometerReady = accelerometer_init();
  getAccelerometerEuler();
  forwardRad = euler[0];  //Straightahead (should be 0)
  motor_Init();
  delay(1000);
}

// Drive motor forward and then backward to ensure yaw correction is correct - do this 5 times

void loop() {
  if (driveCount > 5) {
    //Test over
    return;
  }
  drive(FORWARD, forwardRad, 150);
  delay(DELAYTIME);
  driveTimer += DELAYTIME;
  if (driveTimer > DRIVETIME) {
    backOut();
    //Go back to start
    driveTimer = 0;
    do {
      drive(FORWARD, forwardRad, 100);
      delay(DELAYTIME);
      driveTimer += DELAYTIME;
    } while (driveTimer < 1000);
    drive(STOP, forwardRad, 100);
    aboutTurn();  //turn around instead of reversing
    getAccelerometerEuler();
    forwardRad = euler[0];  //New forward direction
    driveTimer = 0;
    driveCount += 1;
  }
}
