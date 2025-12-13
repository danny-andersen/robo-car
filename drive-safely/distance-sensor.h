#include <HCSR04.h>
#include <Servo.h>

#define PIN_TRIG 13  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define PIN_ECHO 12  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define PIN_SERVO 10
#define SERVO_CENTRE 90  //Positioned straight forward (0 is full right, 180 is full left)
#define MAX_DISTANCE_CAN_MEASURE 200  //Anything above this distance is suspect

// Struct to hold arc info, which represent objects found in the forward field of view
struct Arc {
  uint8_t startIndex;  //0 is 90 right, 180 is 90 left
  uint8_t endIndex;
  uint8_t centerIndex;
  uint8_t width;  //of arc
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

void sweep(uint16_t *distances) {
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
    delay(20);
    measured = HCSR04.measureDistanceCm();
    if (measured[0] > MAX_DISTANCE_CAN_MEASURE) measured[0] = 200;
    distances[a] = measured[0];
  }
  //Sweep left, re-measuring and averaging what we have so far
  for(int a = 0; a <= SERVO_CENTRE; a++) {
    servo.write(a);
    delay(20);
    measured = HCSR04.measureDistanceCm();
    if (measured[0] > MAX_DISTANCE_CAN_MEASURE) measured[0] = 200;
    distances[a] = (distances[a] + measured[0]) / 2;
  }
  //Continue sweeping left
  for (int a = SERVO_CENTRE; a < 180; a++) {
    servo.write(a);
    delay(20);
    measured = HCSR04.measureDistanceCm();
    if (measured[0] > MAX_DISTANCE_CAN_MEASURE) measured[0] = 200;
    distances[a] = measured[0];
  }
  //Sweep right back to centre
  for (int a = 179; a >= 90; a--) {
    servo.write(a);
    delay(20);
    measured = HCSR04.measureDistanceCm();
    if (measured[0] > MAX_DISTANCE_CAN_MEASURE) measured[0] = 200;
    distances[a] = (distances[a] + measured[0]) / 2;
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
      arc.centerIndex = start + count / 2;
      arc.avgDistance = (float)sum / count;

      arcs[arcCount++] = arc;
    }
  }

  return arcCount;  // number of arcs stored
}


