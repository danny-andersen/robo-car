#include "compass.h"

uint16_t currentHeading = 0;  

void setup() {
  Serial.begin(9600);

  // Start I²C bus (SDA, SCL - adapt for your MCU)
  Wire.begin();

  if (!compass_init()) {
    Serial.println("QMC5883P initialization failed!");
    while (true) delay(1000);
  }
  Serial.println("Sensor is ready.");
}

void loop() {
  currentHeading = getHeading();
  Serial.print("Compass: ");
  Serial.println(currentHeading);
  // float xyz[3];
  // if (mag.readXYZ(xyz)) {
  //   // Apply soft-iron correction
  //   xyz[0] *= SCALE_AVG / SCALE_X;
  //   xyz[1] *= SCALE_AVG / SCALE_Y;

  //   // Calculate heading with magnetic declination
  //   float heading = mag.getHeadingDeg(/*decl*/ magDeclination);
  //   Serial.print("X:");
  //   Serial.print(xyz[0]);
  //   Serial.print(" Y:");
  //   Serial.print(xyz[1]);
  //   Serial.print(" Z:");
  //   Serial.print(xyz[2]);
  //   Serial.print(" Heading: ");
  //   Serial.println(heading);
  // }
  delay(250);
}