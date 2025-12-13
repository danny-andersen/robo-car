#include "distance-sensor.h"

int maxDistance = 0;
int maxDirection = 0;
int minDistance = 10000;
int minDirection = 0;
bool sweepComplete = false;
int steps = 1;

void setup() {
  Serial.begin(9600);
  distanceSensorInit();
}

//NOTE: 0 degrees on robocar is full right, 180 degrees is full left. 90 degrees is straight ahead.

void loop() {
  if (sweepComplete) return;
  //Find the direction with the max and min distance
  double* distances = HCSR04.measureDistanceCm();
  int currentDistance = distances[0];
  Serial.print(servoPosition);
  Serial.print(" deg: ");
  Serial.print(currentDistance);
  Serial.println(" cm");
  if (currentDistance > maxDistance) {
    maxDistance = currentDistance;
    maxDirection = servoPosition;
  } else if (currentDistance < minDistance) {
    minDistance = currentDistance;
    minDirection = servoPosition;
  }
  servoPosition += steps;
  if (servoPosition >= 180) {
    steps = -steps;
    servoPosition = 180;
  } else if (servoPosition <= 0) {
    //Done a sweep
    Serial.print("Max distance Position:");
    Serial.print(maxDirection);
    Serial.print(" deg: ");
    Serial.print(maxDistance);
    Serial.println(" cm");
    Serial.print("Min distance Position:");
    Serial.print(minDirection);
    Serial.print(" deg: ");
    Serial.print(minDistance);
    Serial.println(" cm");
    sweepComplete = true;
    servoPosition = SERVO_CENTRE;  //Re-centre when done
    //Now check the find direction function works
    int furthestPosition = findFurthestDistance();
    Serial.println();
    Serial.print("Furthest position found to be: ");
    Serial.print(furthestPosition);
    Serial.print(" (in servo angle): ");
    Serial.print(furthestPosition + 90);
    Serial.println("------------------------------------------------");
  }
  servo.write(servoPosition);
  delay(20);
}