#include <HCSR04.h>

#define TRIG_PIN 13      // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN 12      // Arduino pin tied to echo pin on the ultrasonic sensor.


void setup () {
  Serial.begin(9600);
  HCSR04.begin(TRIG_PIN, ECHO_PIN);
}

void loop () {
  double* distances = HCSR04.measureDistanceCm();
  
  Serial.print(distances[0]);
  Serial.println(" cm");
  
  delay(250);
}