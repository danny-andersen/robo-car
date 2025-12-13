#include <HCSR04.h>
#include <Servo.h>

#define PIN_TRIG 13  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define PIN_ECHO 12  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define PIN_SERVO 10
#define SERVO_CENTRE 90  //Positioned straight forward (0 is full right, 180 is full left)
#define MAX_DISTANCE_CAN_MEASURE 200  //Anything above this distance is suspect
#define NUMBER_OF_MAX_DISTANCES_TO_FIND 10

Servo servo;
int servoPosition;

void distanceSensorInit() {
  HCSR04.begin(PIN_TRIG, PIN_ECHO);
  servo.attach(PIN_SERVO);
  servo.write(SERVO_CENTRE);  //centre the sensor
  servoPosition = SERVO_CENTRE;
  delay(1000);
  double* distances = HCSR04.measureDistanceCm();
  delay(1000);
  randomSeed(analogRead(A3));
}

void sweep(uint8_t *distances, uint8_t *positions, int maxDistance, int maxNumberToFind) {
  bool sweepComplete = false;
  int steps = 5;
  int found = 0;
  bool maxFound = false;
  do {
    //Find the direction with the max and min distance
    double* measured = HCSR04.measureDistanceCm();
    int currentDistance = (measured[0]);
    // Serial.print(servoPosition);
    // Serial.print(" deg: ");
    // Serial.print(currentDistance);
    // Serial.println(" cm");
    if ((currentDistance > distances[found] && !maxFound) || (currentDistance >= MAX_DISTANCE_CAN_MEASURE)) {
      distances[found] = currentDistance;
      positions[found] = servoPosition;
      Serial.print(found);
      Serial.print(": Found max:");
      Serial.print(distances[found]);
      Serial.print(" cm at ");
      Serial.print(positions[found]);
      Serial.println(" deg");

      if (currentDistance >= MAX_DISTANCE_CAN_MEASURE) {
        distances[found] = MAX_DISTANCE_CAN_MEASURE; //Reset distance to max can reliably measure
        if (found <= maxNumberToFind-1) found++;
        maxFound = true;
      }
    }
    servoPosition += steps;
    if (servoPosition >= 180) {
      steps = -steps;
      servoPosition = 180;
    } else if (servoPosition <= 0) {
      sweepComplete = true;
      servoPosition = SERVO_CENTRE;  //Re-centre when done
    }
    servo.write(servoPosition);
    delay(50);
  } while (!sweepComplete);
}

int findFurthestDistance() {
  uint8_t maxDistances[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t positions[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  sweep(maxDistances, positions, MAX_DISTANCE_CAN_MEASURE, NUMBER_OF_MAX_DISTANCES_TO_FIND);
  for (int i=0; i< NUMBER_OF_MAX_DISTANCES_TO_FIND; i++){
    Serial.print("Position: ");
    Serial.print(positions[i]);
    Serial.print(" Distance: ");
    Serial.print(maxDistances[i]);
  }
  if (maxDistances[0] < MAX_DISTANCE_CAN_MEASURE) {
    //only one max distance found - return that, with 0 being straight ahead
    return positions[0] - SERVO_CENTRE; 
  }
  //We have multiple max distances as we have many positions greater than the range - need to choose one
  //Count the number we have
  int numberToChoose = 0;
  for (int i=0; i< NUMBER_OF_MAX_DISTANCES_TO_FIND; i++){
    if (maxDistances[i] >= MAX_DISTANCE_CAN_MEASURE) numberToChoose++;
  }
  //Choose one at "random" and return the position
  return positions[random(numberToChoose+1)] - SERVO_CENTRE;

}

