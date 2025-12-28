// compass.h
#include "qmc5883p.h"

const float SCALE_AVG = 0.452f;
const float SCALE_X   = 0.434f;
const float SCALE_Y   = 0.470f;

const float magDeclination = (0.0 + (7.0 / 60.0)) * M_PI / 180;
const int8_t magOffset = -90;  //Compass is mounted 90deg to the right of the direction of travel 

QMC5883P mag;

bool compass_init() {
  bool ret = mag.begin();
  mag.setHardIronOffsets(-0.036f, 0.164f);
  return ret;
}

uint16_t getHeading() {
  int16_t heading = mag.getHeadingDeg(magDeclination) + magOffset;
  if (heading < 0) {
    heading = 360 + heading;
  }
  return heading;
}