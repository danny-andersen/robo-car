//Functions to control drive motors to perform movement of a wheeled device.

#define ROTATE_CHECK_INTERVAL 100
#define MAX_ROTATE_TIME 5000

Robot_State lastRobotState = INIT;


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
  if (rotate == 0) return true;
  long timer = 0;
  long backTimer = 0;
  long forwardTimer = 0;
  int16_t diff = rotate;  //Initial rotation amount and direction
  bool triedOtherDirn = false;
  Robot_State nextState = ROTATING;  //This is used to determine the next rotation state after moving forward or back after an obstruction
  //   ROTATING_LEFT,
  // ROTATING_RIGHT,
  // ROTATING_LEFT_BLOCKED_RIGHT, //Tried rotating right but blocked, so going left
  // ROTATING_RIGHT_BLOCKED_LEFT, //Tried rotating left but blocked, so going right
  // ROTATING_FRONT_BLOCKED_BACKING_OUT, //Front obstruction but clear at back, so backup a bit and then rotate (if clear)
  // ROTATING_REAR_BLOCKED_GO_FORWARD, //Rear obstruction but clear at front, so go forward a bit and then rotate (if clear)

  if (rotate > 0) {
    systemStatus.robotState = ROTATING_RIGHT;
  } else {
    systemStatus.robotState = ROTATING_LEFT;
  }

  do {

    //If rotating left and front left or rear right - cant go this way.
    //If both front - need to reverse a bit first until one of the sensors is freed up, unless rear sensor is set. If so, try backing up a bit to see if one of the front sensor clears

    //If front right and rear right, or front left and rear left, then cant rotate right or left. Try backing up a bit to see if the front sensor clears.
    //If it doesnt or no progress is made, try rotating a bit opposite to which side is blocked - if cant rotate then go forward a bit and then try and rotate.

    if (lastRobotState != systemStatus.robotState) {
      //Update PI log
      sendSystemStatus();
      lastRobotState = systemStatus.robotState;
    }

    //Drive motors based on current state
    switch (systemStatus.robotState) {
      case ROTATING_RIGHT:
        driveMotor(ROTATE_RIGHT, 75, 75);
        break;
      case ROTATING_LEFT:
        driveMotor(ROTATE_LEFT, 75, 75);
        break;
      case ROTATING_LEFT_BLOCKED_RIGHT:
        driveMotor(ROTATE_LEFT, 75, 75);
        break;
      case ROTATING_RIGHT_BLOCKED_LEFT:
        driveMotor(ROTATE_RIGHT, 75, 75);
        break;
      case ROTATING_FRONT_BLOCKED_BACKING_OUT:
        backTimer -= ROTATE_CHECK_INTERVAL;
        driveMotor(BACK, 50, 50);
        break;
      case ROTATING_REAR_BLOCKED_GO_FORWARD:
        forwardTimer -= ROTATE_CHECK_INTERVAL;
        driveMotor(FORWARD, 50, 50);
        break;
    }

    delay(ROTATE_CHECK_INTERVAL);
    timer += ROTATE_CHECK_INTERVAL;  // Prevent continuous spinning if something goes wrong
    wdt_reset();
    currentDirn = getCompassBearing();  // Before we work out which direction to turn, remember what straightahead is
    rotate = getRotation(currentDirn, directionRequired);
    if (leftGround()) {
      driveMotor(STOP, 0, 0);
      sendSystemStatus();
      break;
    }
    getCombinedProximity();
    //Check on what obstructions we have based on our current rotation state - need to work out what movement is possible
    switch (systemStatus.robotState) {
      case ROTATING_RIGHT:
        if (checkFrontRightProximity(systemStatus.proximityState) && checkRearLeftProximity(systemStatus.proximityState)) {
          //If rotating right and front right or rear left - cant go this way round, try the other way
          systemStatus.robotState = ROTATING_LEFT_BLOCKED_RIGHT;
        } else if (checkFrontRightProximity(systemStatus.proximityState) && !checkDirectRearOnly(systemStatus.proximityState)) {
          //If rotating right and blocked front right, but we are clear to the back so can try backing up a little
          systemStatus.robotState = ROTATING_FRONT_BLOCKED_BACKING_OUT;
          nextState = ROTATING_RIGHT;
          backTimer = 500;

        } else if (checkRearLeftProximity(systemStatus.proximityState) && !checkDirectFrontProximity(systemStatus.proximityState)) {
          //Only the back is blocked, so go forward a little
          systemStatus.robotState = ROTATING_REAR_BLOCKED_GO_FORWARD;
          forwardTimer = 500;
        }
        break;
      case ROTATING_LEFT:
        if (checkFrontLeftProximity(systemStatus.proximityState) && checkRearRightProximity(systemStatus.proximityState)) {
          //If rotating right and front right or rear left - cant go this way round, try the other way
          systemStatus.robotState = ROTATING_RIGHT_BLOCKED_LEFT;
        } else if (checkFrontLeftProximity(systemStatus.proximityState) && !checkDirectRearOnly(systemStatus.proximityState)) {
          //If rotating right and blocked front right, but we are clear to the back so can try backing up a little
          systemStatus.robotState = ROTATING_FRONT_BLOCKED_BACKING_OUT;
          backTimer = 500;
          nextState = ROTATING_LEFT;
        } else if (checkRearRightProximity(systemStatus.proximityState) && !checkDirectFrontProximity(systemStatus.proximityState)) {
          systemStatus.robotState = ROTATING_REAR_BLOCKED_GO_FORWARD;
          forwardTimer = 500;
          nextState = ROTATING_LEFT;
        }
        break;
      case ROTATING_LEFT_BLOCKED_RIGHT:
        if (!checkFrontRightProximity(systemStatus.proximityState) && !checkDirectFrontProximity(systemStatus.proximityState) && checkRearLeftProximity(systemStatus.proximityState)) {
          //We have cleared the front right issue but not the rear left - go forward a bit and then retry the rotation
          forwardTimer = 500;
          nextState = ROTATING_RIGHT;
          systemStatus.robotState = ROTATING_REAR_BLOCKED_GO_FORWARD;
        } else if (checkFrontRightProximity(systemStatus.proximityState) && !checkDirectRearOnly(systemStatus.proximityState) && !checkRearLeftProximity(systemStatus.proximityState)) {
          //We have cleared the rear left issue but not the front - back up a bit and then retry
          backTimer = 500;
          systemStatus.robotState = ROTATING_FRONT_BLOCKED_BACKING_OUT;
          nextState = ROTATING_RIGHT;
        } else if (checkFrontLeftProximity(systemStatus.proximityState)) {
          //We cant go round this way either - try going forward or backing out
          if (!checkDirectRearOnly(systemStatus.proximityState)) {
            //Clear directly behind - reverse up a bit
            systemStatus.robotState = ROTATING_FRONT_BLOCKED_BACKING_OUT;
            nextState = ROTATING_RIGHT;
            backTimer = 500;
          } else if (!checkDirectFrontProximity(systemStatus.proximityState)) {
            //Clear directly in front - try going forward slightly
            systemStatus.robotState = ROTATING_REAR_BLOCKED_GO_FORWARD;
            forwardTimer = 500;
            nextState = ROTATING_RIGHT;
          } else {
            // We cant rotate this way either and cant go forward or back....
          }
        } else if (checkRearRightProximity(systemStatus.proximityState)) {
          //We cant go round this way either - try going forward or backing out
          if (!checkDirectFrontProximity(systemStatus.proximityState)) {
            //Clear directly in front - try going forward slightly
            systemStatus.robotState = ROTATING_REAR_BLOCKED_GO_FORWARD;
            nextState = ROTATING_RIGHT;
            forwardTimer = 500;
          } else {
            if (!checkDirectRearOnly(systemStatus.proximityState)) {
              //Clear directly behind - reverse up a bit
              systemStatus.robotState = ROTATING_FRONT_BLOCKED_BACKING_OUT;
              backTimer = 500;
              nextState = ROTATING_RIGHT;
            } else {
              // ??
            }
          }
        }
        break;
      case ROTATING_RIGHT_BLOCKED_LEFT:
        if (!checkFrontLeftProximity(systemStatus.proximityState) && !checkDirectFrontProximity(systemStatus.proximityState) && checkRearRightProximity(systemStatus.proximityState)) {
          //We have cleared the front left issue but not the rear right - go forward a bit and then retry the rotation
          forwardTimer = 500;
          nextState = ROTATING_LEFT;
          systemStatus.robotState = ROTATING_REAR_BLOCKED_GO_FORWARD;
        } else if (checkFrontLeftProximity(systemStatus.proximityState) && !checkDirectRearOnly(systemStatus.proximityState) && !checkRearLeftProximity(systemStatus.proximityState)) {
          //We have cleared the rear left issue but not the front - back up a bit and then retry
          backTimer = 500;
          nextState = ROTATING_LEFT;
          systemStatus.robotState = ROTATING_FRONT_BLOCKED_BACKING_OUT;
        } else if (checkFrontRightProximity(systemStatus.proximityState)) {
          //We cant go round this way either - try going forward or backing out
          if (!checkDirectRearOnly(systemStatus.proximityState)) {
            //Clear directly behind - reverse up a bit
            systemStatus.robotState = ROTATING_FRONT_BLOCKED_BACKING_OUT;
            nextState = ROTATING_LEFT;
          } else if (!checkDirectFrontProximity(systemStatus.proximityState)) {
            //Clear directly in front - try going forward slightly
            systemStatus.robotState = ROTATING_REAR_BLOCKED_GO_FORWARD;
            nextState = ROTATING_LEFT;
          } else {
            // ??
          }
        } else if (checkRearLeftProximity(systemStatus.proximityState)) {
          //We cant go round this way either - try going forward or backing out
          if (!checkDirectFrontProximity(systemStatus.proximityState)) {
            //Clear directly in front - try going forward slightly
            systemStatus.robotState = ROTATING_REAR_BLOCKED_GO_FORWARD;
            nextState = ROTATING_LEFT;
          } else {
            if (!checkDirectRearOnly(systemStatus.proximityState)) {
              //Clear directly behind - reverse up a bit
              systemStatus.robotState = ROTATING_FRONT_BLOCKED_BACKING_OUT;
              nextState = ROTATING_LEFT;
              backTimer = 500;
            } else {
              // ??
            }
          }
        }
        break;
      case ROTATING_FRONT_BLOCKED_BACKING_OUT:
        if (checkDirectRearOnly(systemStatus.proximityState) || (backTimer <= 0)) {
          //Can't back up any further - start from the top
          if (nextState == ROTATING_RIGHT and checkFrontRightProximity(systemStatus.proximityState)) {
            //Still blocked right - try going left
            systemStatus.robotState = ROTATING_LEFT_BLOCKED_RIGHT;
          } else if (nextState == ROTATING_LEFT and checkFrontLeftProximity(systemStatus.proximityState)) {
            systemStatus.robotState = ROTATING_RIGHT_BLOCKED_LEFT;
          } else {
            systemStatus.robotState = nextState;
          }
        } else if ((nextState == ROTATING_RIGHT and !checkFrontRightProximity(systemStatus.proximityState)) || (nextState == ROTATING_LEFT and !checkFrontLeftProximity(systemStatus.proximityState))) {
          //Now clear to the front - resume rotating
          systemStatus.robotState = nextState;
        }
        break;

      case ROTATING_REAR_BLOCKED_GO_FORWARD:
        if (checkDirectFrontProximity(systemStatus.proximityState) || (forwardTimer <= 0)) {
          //Can't go forward any further - start from the top
          if (nextState == ROTATING_RIGHT and checkRearLeftProximity(systemStatus.proximityState)) {
            //Still blocked - try going left
            systemStatus.robotState = ROTATING_LEFT_BLOCKED_RIGHT;
          } else if (nextState == ROTATING_LEFT and checkFrontLeftProximity(systemStatus.proximityState)) {
            systemStatus.robotState = ROTATING_RIGHT_BLOCKED_LEFT;
          } else {
            systemStatus.robotState = nextState;
          }
        } else if ((nextState == ROTATING_RIGHT and !checkRearLeftProximity(systemStatus.proximityState)) || (nextState == ROTATING_LEFT and !checkRearRightProximity(systemStatus.proximityState))) {
          //Now clear to the rear - resume rotating
          systemStatus.robotState = nextState;
        }
        break;
    }


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
    return false;  //Failed to make the turn in time
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
  do {
    getCombinedProximity();
    if (leftGround() || checkRearProximity(systemStatus.proximityState)) {
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
