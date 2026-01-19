//Functions to control drive motors to perform movement of a wheeled device.

#define ROTATE_CHECK_INTERVAL 100
#define MAX_ROTATE_TIME 3000


int16_t getRotation(int16_t current, int16_t required) {
  //Work out smallest rotation to required direction
  int16_t rotateRight = 0;
  int16_t rotateLeft = 0;
  int16_t rotate = 0;
  if (required > current) {
    rotateRight = required - current;
    rotateLeft = (360 - required) + current;

  } else {
    rotateLeft = current - required;
    rotateRight = (360 - current) + required;
  }
  //Rotation angle - positive is right, negative is rotate left
  if (rotateRight > rotateLeft) {
    rotate = -rotateLeft;
  } else {
    rotate = rotateRight;
  }
  return rotate;
}

bool rotateTo(int16_t directionRequired) {
  // Rotate the car to the direction that is required
  // 0 is straight ahead (no change)
  //-90 is 90 deg to the left
  //+90 is 90 deg to the right
  directionRequired = normalise(directionRequired);
  // getAccelerometerEuler();
  // currentDirn = eulerDeg[0];                          // Before we work out which direction to turn, remember what straightahead is
  // diff = normalise(directionRequired - currentDirn);  //Rotation angle - positive is right, negative is rotate left
  int16_t currentDirn = getCompassBearing();  // Before we work out which direction to turn, remember what straightahead is
  int16_t rotate = getRotation(currentDirn, directionRequired);
  // int16_t startingYaw = yaw;
  // if (Serial) {
  //   Serial.print("Rotating to: ");
  //   Serial.print(directionRequired);
  //   Serial.print(" Rotate Angle: ");
  //   Serial.print(rotate);
  //   Serial.print(", straightahead is ");
  //   Serial.println(currentDirn);
  // }
  if (rotate == 0) return true;
  long timer = 0;
  long backTimer = 0;
  long forwardTimer = 0;
  int16_t diff = rotate;  //Initial rotation amount and direction
  bool triedOtherDirn = false;
  //Start rotation
  if (rotate > 0) {
    // Rotate right
    driveMotor(ROTATE_RIGHT, 75, 75);
  } else {
    // Rotate left
    driveMotor(ROTATE_LEFT, 75, 75);
  }

  do {
    delay(ROTATE_CHECK_INTERVAL);
    wdt_reset();
    timer += ROTATE_CHECK_INTERVAL;  // Prevent continuous spinning if something goes wrong
    if (leftGround()) {
      driveMotor(STOP, 0, 0);
      break;
    }
    // Check for obstacles when turning
    uint8_t proximity = getProximityState();
    if (rotate > 0) {
      // Rotating right
      if (checkFrontRightProximity(proximity) && checkRearLeftProximity(proximity)) {
        // We cant rotate this way
        // Try the other way
        if (!triedOtherDirn) {
          rotate -= rotate;
          backTimer = 0;
          forwardTimer = 0;
          timer = 0;
          triedOtherDirn = true;
        } else {
          driveMotor(STOP, 0, 0);
          return false;
        }
      } else if (checkFrontRightProximity(proximity)) {
        // Cant rotate - need to back up a bit before rotating
        if (backTimer > 0) {
          // We dont have much room to the rear, only back up until obstruction gone
          backTimer = ROTATE_CHECK_INTERVAL;
        } else {
          backTimer = 400;
        }
        forwardTimer = 0;
      } else if (checkRearLeftProximity(proximity)) {
        // Cant rotate - need to go forward a bit before rotating
        if (forwardTimer > 0) {
          // Dont have much room to the front, only go forward until obstruction gone
          forwardTimer = ROTATE_CHECK_INTERVAL;
        } else {
          forwardTimer = 400;
        }
        backTimer = 0;
      }
      if (backTimer > 0) {
        driveMotor(BACK, 75, 75);
        backTimer -= ROTATE_CHECK_INTERVAL;
      } else if (forwardTimer >= 0) {
        driveMotor(FORWARD, 75, 75);
        forwardTimer -= ROTATE_CHECK_INTERVAL;
      } else {
        driveMotor(ROTATE_RIGHT, 75, 75);
      }
    } else {
      //Rotating left
      // Check for obstacles when turning
      if (checkFrontLeftProximity(proximity) && checkRearRightProximity(proximity)) {
        // We cant rotate this way
        // Try the other way
        if (!triedOtherDirn) {
          rotate -= rotate;
          backTimer = 0;
          forwardTimer = 0;
          timer = 0;
          triedOtherDirn = true;
        } else {
          //We are pretty stuck!
          driveMotor(STOP, 0, 0);
          return false;
        }
      } else if (checkFrontLeftProximity(proximity)) {
        // Cant rotate - need to back up a bit before rotating
        if (backTimer > 0) {
          // We dont have much room to the rear, only back up until obstruction gone
          backTimer = ROTATE_CHECK_INTERVAL;
        } else {
          backTimer = 400;
        }
        forwardTimer = 0;
      } else if (checkRearRightProximity(proximity)) {
        // Cant rotate - need to go forward a bit before rotating
        if (forwardTimer > 0) {
          // Dont have much room to the front, only go forward until obstruction gone
          forwardTimer = ROTATE_CHECK_INTERVAL;
        } else {
          forwardTimer = 400;
        }
        backTimer = 0;
      }
      if (backTimer > 0) {
        driveMotor(BACK, 75, 75);
        backTimer -= ROTATE_CHECK_INTERVAL;
      } else if (forwardTimer >= 0) {
        driveMotor(FORWARD, 75, 75);
        forwardTimer -= ROTATE_CHECK_INTERVAL;
      } else {
        // Rotate left
        driveMotor(ROTATE_LEFT, 75, 75);
      }
    }
    currentDirn = getCompassBearing();  // Before we work out which direction to turn, remember what straightahead is
    rotate = getRotation(currentDirn, directionRequired);
    // if (Serial) {
    //   Serial.print("Rotating ");
    //   Serial.print(rotate > 0 ? "RIGHT " : "LEFT ");
    //   Serial.print(rotate);
    //   Serial.print(" Current dirn: ");
    //   Serial.print(currentDirn);
    //   Serial.print(" Required dirn: ");
    //   Serial.print(directionRequired);
    //   Serial.print(" Timer: ");
    //   Serial.println(timer);
    // }

    // if (timer > 500) {
    //   if (abs(startingYaw - yaw) <= 10) {
    //     //We must have rotated a complete 360 - reset the gyro
    //     driveMotor(STOP, 0, 0);
    //     return false;
    //   }
    // }
    // } while (timer <= 5000 && ((directionRequired > yaw) && ((directionRequired - yaw) < 180)) || ((directionRequired < yaw) && ((directionRequired - yaw) >= 180)));
  } while (((rotate > 0 && diff > 0) || (rotate < 0 && diff < 0)) && timer < MAX_ROTATE_TIME);
  driveMotor(STOP, 0, 0);
  if (timer >= MAX_ROTATE_TIME) {
    return false; //Failed to make the turn in time
  }
  return true;
}

void drive(Motor_Direction direction, float straightAheadRad, uint8_t speed) {
  // Drives the motor in the required direction, allowing for unbalanced wheels / motors /etc.
  // By checking for the yaw in direction and allowing for it by adjusting the motor speed
  // Note: Only required when driving straight!
  uint8_t leftSpeed = speed;
  uint8_t rightSpeed = speed;
  if (direction == FORWARD || direction == BACK) {
    float currentYaw = getCompassBearingRads();
    // straightAheadRad = normalise_rad(straightAheadRad);
    leftSpeed = (straightAheadRad - currentYaw) * Kp + speed;
    rightSpeed = (currentYaw - straightAheadRad) * Kp + speed;
    if (leftSpeed > maxSpeed) {
      leftSpeed = maxSpeed;
    } else if (leftSpeed < minSpeed) {
      leftSpeed = minSpeed;
    }
    if (rightSpeed > maxSpeed) {
      rightSpeed = maxSpeed;
    } else if (rightSpeed < minSpeed) {
      rightSpeed = 10;
    }
    if (direction == BACK) {
      // Reverse the left and right speeds, as need to reverse the yaw adjustment
      uint8_t s = leftSpeed;
      leftSpeed = rightSpeed;
      rightSpeed = s;
    }
    // Serial.print("Direction to Drive: ");
    // Serial.print(straightAheadRad);
    // Serial.print("Yaw: ");
    // Serial.print(currentYaw);
    // Serial.print("Left: ");
    // Serial.print(leftSpeed);
    // Serial.print("Right: ");
    // Serial.println(rightSpeed);
  }

  driveMotor(direction, rightSpeed, leftSpeed);
}

void backOut() {
  float forwardDirection = getCompassBearingRads();
  bool removingPitchOrRoll = false;
  unsigned long backoutTimer = 0;
  drive(BACK, forwardDirection, 50);
  do {
    if (leftGround() || getRearProximity()) {
      // Something behind us or we have back been picked up - stop
      break;
    }
    if (rollingOrPitching()) {
      // Move forward out of trouble
      removingPitchOrRoll = true;
      drive(FORWARD, forwardDirection, 50);
    } else if (removingPitchOrRoll) {
      // Moved out of trouble - stop
      break;
    } else {
      //Continue driving back
      drive(BACK, forwardDirection, 50);
    }
    delay(50);
    wdt_reset();
    backoutTimer += 50;
  } while (backoutTimer < 500);
  drive(STOP, forwardDirection, 0);
}

void aboutTurn() {
  int16_t forwardDirection = getCompassBearing();
  int16_t reversed = normalise(forwardDirection + 180);
  rotateTo(reversed);
}

 
