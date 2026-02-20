#ifndef GROUND_TRACKING_H
#define GROUND_TRACKING_H


#define GROUND_TRACKING_LEFT_PIN A2
#define GROUND_TRACKING_MID_PIN A1
#define GROUND_TRACKING_RIGHT_PIN A0
#define NO_GROUND 950

uint16_t leftTrack = 0;
uint16_t midTrack = 0;
uint16_t rightTrack = 0;

void groundTrackingInit() {
  pinMode(GROUND_TRACKING_RIGHT_PIN, INPUT);
  pinMode(GROUND_TRACKING_MID_PIN, INPUT);
  pinMode(GROUND_TRACKING_LEFT_PIN, INPUT);
}

void readGroundTracking() {
  rightTrack = analogRead(GROUND_TRACKING_RIGHT_PIN);
  midTrack = analogRead(GROUND_TRACKING_MID_PIN);
  leftTrack = analogRead(GROUND_TRACKING_LEFT_PIN);
}

bool leftGround() {
  readGroundTracking();
  return (leftTrack >= NO_GROUND && rightTrack >= NO_GROUND && midTrack >= NO_GROUND);
}

#endif