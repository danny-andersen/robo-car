#include "ground-tracking.h"

void setup() {
  Serial.begin(9600);
  groundTrackingInit();
}

void loop() {
  readGroundTracking();
  Serial.print("Ground Tracking: L: ");
  Serial.print(leftTrack);
  Serial.print(" M: ");
  Serial.print(midTrack);
  Serial.print(" R: ");
  Serial.println(rightTrack);
  Serial.print("Left Ground? ");
  Serial.println(leftGround());
  delay(500);
}
