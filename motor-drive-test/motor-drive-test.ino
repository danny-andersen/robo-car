#include <avr/wdt.h>
#include "accelerometer.h"
bool leftGround() {
  return false;
}
#include "inter-i2c.h"
#include "robo-car.h"
#include "motor-driver.h"

bool accelerometerReady = false;
int driveCount = 0;
unsigned long driveTimer = 0;
unsigned long DRIVETIME = 3000;
unsigned long DELAYTIME = 50;
float forwardRad = 0;
bool forward = true;

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
  if (forward) {
    drive(FORWARD, forwardRad, 125);

  } else {
    drive(BACK, forwardRad, 125);
}
delay(DELAYTIME);
driveTimer += DELAYTIME;
if (driveTimer > DRIVETIME) {
  forward = !forward;
  driveTimer = 0;
  driveCount += 1;
}
}
