#include <avr/wdt.h>
#include "accelerometer.h"
bool leftGround() {
  return false;
}
#include "inter-i2c.h"
#include "robo-car.h"
#include "motor-driver.h"

#define MAX_SPEED 55

bool accelerometerReady = false;
int driveCount = 0;
unsigned long driveTimer = 0;
unsigned long DRIVETIME = 3000;
unsigned long DELAYTIME = 250;
float forwardRad = 0;
bool forward = true;
uint8_t motorSpeed = 25;

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");

  // Start I²C bus
  Wire.begin();

  delay(3000);  //Let everything settle before initialising accelerometer
  accelerometerReady = accelerometer_init();
  getAccelerometerEuler();
  forwardRad = euler[0];  //Straightahead (should be 0)
  motor_Init();
  delay(1000);
}

// Drive motor forward and check speed returned from wheel sensors

void loop() {
  Serial.print("Starting motor at speed: ");
  Serial.println(motorSpeed);
  sendStartMotorCmd();
  drive(FORWARD, forwardRad, motorSpeed);
  driveTimer = 0;
  do {
    delay(500);
    driveTimer += DELAYTIME;
    getStatusCmd();
  } while (driveTimer < DRIVETIME);
  Serial.println("Stopping motor");
  drive(STOP, forwardRad, motorSpeed);
  sendStopMotorCmd();
  delay(3000);
  motorSpeed += 10;
  if (motorSpeed > MAX_SPEED) {
    motorSpeed = 25;
  }
}
