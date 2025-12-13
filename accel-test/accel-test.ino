#include "accelerometer.h"

bool accelerometerReady = false;

void setup() {
  Serial.begin(9600);
  accelerometerReady = accelerometer_init();
  delay(2000);
}

void loop() {
  if (!accelerometerReady) return;
  static float Yaw;
  //   if (getAccelerometerAllReading()) {
  if (getAccelerometerEuler()) {
    Serial.print("euler\t");
    Serial.print(euler[0]);
    Serial.print("\t");
    Serial.print(euler[1]);
    Serial.print("\t");
    Serial.println(euler[2]);
    Serial.print("eulerDeg\t");
    Serial.print(eulerDeg[0]);
    Serial.print("\t");
    Serial.print(eulerDeg[1]);
    Serial.print("\t");
    Serial.println(eulerDeg[2]);
    // Serial.print("areal\t");
    // Serial.print(aaReal.x);
    // Serial.print("\t");
    // Serial.print(aaReal.y);
    // Serial.print("\t");
    // Serial.println(aaReal.z);
  } else {
    Serial.println("No reading...");
  }

  delay(200);
}
