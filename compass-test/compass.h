// compass.h
#include "qmc5883p.h"

const float SCALE_AVG = 0.444f;
const float SCALE_X = 0.438f;
const float SCALE_Y = 0.449f;

const float magDeclination = (0.0 + (7.0 / 60.0)) * M_PI / 180;

bool compass_init(QMC5883P *mag) {
  bool ret = mag->begin();
  mag->setHardIronOffsets(-0.005f, 0.186f);
  return ret;
}