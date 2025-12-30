#include <HCSR04.h>
#include <Servo.h>

#define PIN_TRIG 13  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define PIN_ECHO 12  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define PIN_SERVO 10
#define SERVO_CENTRE 90               //Positioned straight forward (0 is full right, 180 is full left)
#define MAX_DISTANCE_CAN_MEASURE 150  //Anything above this distance is suspect

// Struct to hold arc info, which represent objects found in the forward field of view
struct Arc {
  uint8_t startIndex;  //0 is 90 right, 180 is 90 left
  uint8_t endIndex;
  uint8_t centreDirection;
  uint8_t width;         //of arc
  uint16_t avgDistance;  //distance
};

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

uint16_t clearDistanceAhead() {
  double* measured = HCSR04.measureDistanceCm();
  uint16_t currentDistance = (uint16_t)lrint(measured[0]);
  return currentDistance;
}

void sweep(uint16_t* distances) {
  bool sweepComplete = false;
  int steps = 1;
  servoPosition = SERVO_CENTRE;
  double* measured;
  servo.write(SERVO_CENTRE);
  //Sweep right then left then back to centre. Each point in the sweep arc gets measured twice
  //Take the average of each reading into the distances array
  //Sweep right
  for (int a = SERVO_CENTRE; a >= 0; a--) {
    servo.write(a);
    delay(10);
    measured = HCSR04.measureDistanceCm();
    if (measured[0] > MAX_DISTANCE_CAN_MEASURE) {
      distances[a] = MAX_DISTANCE_CAN_MEASURE;
    } else {
      distances[a] = measured[0];
    }
    if (leftGround()) {
      return;
    }
    wdt_reset();
  }
  //Sweep left, re-measuring and averaging what we have so far
  for (int a = 0; a <= SERVO_CENTRE; a++) {
    servo.write(a);
    delay(10);
    measured = HCSR04.measureDistanceCm();
    if (measured[0] > MAX_DISTANCE_CAN_MEASURE) {
      distances[a] = MAX_DISTANCE_CAN_MEASURE;
    } else {
      distances[a] = (distances[a] + measured[0]) / 2;
    }

    if (leftGround()) {
      return;
    }
    wdt_reset();
  }
  //Continue sweeping left
  for (int a = SERVO_CENTRE; a < 180; a++) {
    servo.write(a);
    delay(10);
    measured = HCSR04.measureDistanceCm();
    if (measured[0] > MAX_DISTANCE_CAN_MEASURE) {
      distances[a] = MAX_DISTANCE_CAN_MEASURE;
    } else {
      distances[a] = measured[0];
    }
    wdt_reset();
  }
  //Sweep right back to centre
  for (int a = 179; a >= 90; a--) {
    servo.write(a);
    delay(10);
    measured = HCSR04.measureDistanceCm();
    if (measured[0] > MAX_DISTANCE_CAN_MEASURE) {
      distances[a] = MAX_DISTANCE_CAN_MEASURE;
    } else {
      distances[a] = (distances[a] + measured[0]) / 2;
    }
    if (leftGround()) {
      return;
    }
    wdt_reset();
  }

  //Now smooth response by averaging distances over a 5 degree arc
  for (int i = 0; i < 180; i++) {
    unsigned long sum = 0;
    for (int j = -2; j <= 2; j++) {
      int a = i + j;
      if (a < 0) a = 0;
      if (a >= 180) a = 179;
      sum += distances[a];
    }
    distances[i] = sum / 5;
  }
}

// Function returns number of arcs found, fills provided array
int findObjectsInSweep(uint16_t arr[], int size, Arc arcs[], int maxArcs) {
  int arcCount = 0;
  int i = 0;

  while (i < size && arcCount < maxArcs) {
    // Normalize distance (cap at 200)
    uint16_t startVal = arr[i];

    // Define tolerance range ±20%
    float lowerBound = startVal * 0.8;
    float upperBound = startVal * 1.2;

    // Start of arc
    int start = i;
    long sum = 0;
    int count = 0;

    // Expand arc while values stay within ±20% of startVal
    while (i < size) {
      uint16_t val = arr[i];
      if (val < lowerBound || val > upperBound) break;  // arc ends
      sum += val;
      count++;
      i++;
    }

    // Only consider arcs with width >= 10 degrees
    if (count >= 10) {
      Arc arc;
      arc.startIndex = start;
      arc.endIndex = i - 1;
      arc.width = count;
      arc.centreDirection = start + int((count / 2) + 0.5);
      arc.avgDistance = int((sum / count) + 0.5);

      arcs[arcCount++] = arc;
    }
  }

  return arcCount;  // number of arcs stored
}

//Analyse the distance of the objects found in the sweep to determine
//(a) whether is safe to drive and which direction
//(b) Or whether we need to turn around as we are blocked in
//This may need a reverse if there is no room to turn around
SWEEP_STATUS checkSurroundings(Arc arcs[], uint8_t maxObjects, uint8_t* bestDirectionIndex) {
  //Determine greatest distance
  uint8_t furthestObjectIndex = 0;
  uint8_t closestObjectIndex = 0;
  uint8_t widthOfObject = arcs[0].width;
  SWEEP_STATUS retStatus = CLEAR_TO_DRIVE;
  for (int i = 1; i < maxObjects; i++) {
    if (arcs[i].avgDistance == 0) {
      //null entry
      continue;
    }
    if (arcs[i].avgDistance > arcs[furthestObjectIndex].avgDistance) {
      furthestObjectIndex = i;
    }
    if (arcs[i].avgDistance < arcs[closestObjectIndex].avgDistance) {
      closestObjectIndex = i;
    }
  }
  //Now re-check best object using the width
  for (int i = 1; i < maxObjects; i++) {
    if (arcs[i].avgDistance == 0) {
      //null entry
      continue;
    }
    if (arcs[i].avgDistance == arcs[furthestObjectIndex].avgDistance && arcs[i].width > widthOfObject) {
      //Object is the same distance away (probably max) but is wider - go to that one
      furthestObjectIndex = i;
    }
  }
  if (arcs[furthestObjectIndex].avgDistance <= MIN_DISTANCE_AHEAD) {
    //No point going any further, turn around and do another sweep
    //Check if any objects are within minimum safe distance, if so need to back out rather than rotate
    if (arcs[closestObjectIndex].avgDistance <= MIN_DISTANCE_TO_TURN) {
      retStatus = CANNOT_TURN;
    } else {
      retStatus = BLOCKED_AHEAD;
    }
  }
  *bestDirectionIndex = furthestObjectIndex;
  return retStatus;
}
