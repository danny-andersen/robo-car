

bool checkFrontRightProximity(uint8_t status) {
  return (status & FRONT_RIGHT_PROX_SET) || (status & TOP_FRONT_RIGHT_PROX_SET);
}

bool checkFrontLeftProximity(uint8_t status) {
  return (status & FRONT_LEFT_PROX_SET) || (status & TOP_FRONT_LEFT_PROX_SET);
}

bool checkFrontProximity(uint8_t status) {
  return (status & FRONT_LEFT_PROX_SET) || (status & FRONT_RIGHT_PROX_SET) || (status & TOP_FRONT_RIGHT_PROX_SET) || (status & TOP_FRONT_LEFT_PROX_SET) || (status & FRONT_FRONT_PROX_SET);
}

bool checkDirectFrontProximity(uint8_t status) {
  //When checking directly in the front, ignore the top left and right, as these are at a more obtuse angle, to detect issues with rotating, rather than driving straight ahead
  return (status & FRONT_LEFT_PROX_SET) || (status & FRONT_RIGHT_PROX_SET) || (status & FRONT_FRONT_PROX_SET);
}
bool checkRearRightProximity(uint8_t status) {
  return status & REAR_RIGHT_PROX_SET;
}

bool checkRearLeftProximity(uint8_t status) {
  return status & REAR_LEFT_PROX_SET;
}

bool checkRearProximity(uint8_t status) {
  return (status & REAR_LEFT_PROX_SET) || (status & REAR_RIGHT_PROX_SET) || (status & REAR_REAR_PROX_SET);
}

bool checkDirectRearOnly(uint8_t status) {
  return (status & REAR_REAR_PROX_SET);
}

void getCombinedProximity() {
  do {
    //Try and read a valid proximatey status continuously
    //Eventually the watchdog will trigger if cant get it - we cant proceed without it
    nanoCommsError = getNanoStatusCmd();
    delay(10);
  } while (nanoCommsError);
  //Get the PI status, which includes the LIDAR proximity state, with the same bits set as the proximity sensors on the nano
  piCommsError = getPiStatusCmd();
}


