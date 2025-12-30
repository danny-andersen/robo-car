#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_BIN_1 8
#define PIN_Motor_AIN_1 7
#define PIN_Motor_STBY 3

#define speed_Max 255

#define ROTATE_CHECK_INTERVAL 100
#define MAX_ROTATE_TIME 3000


enum Motor_Direction {
  FORWARD,         //(1)
  BACK,            //(2)
  ROTATE_LEFT,     //(3)
  ROTATE_RIGHT,    //(4)
  LEFT_FORWARD,    //(5)
  LEFT_BACKWARD,   //(6)
  RIGHT_FORWARD,   //(7)
  RIGHT_BACKWARD,  //(8)
  STOP,
  INITIAL
};

static unsigned long lastDriveTime = 0;
static Motor_Direction lastDriveDirection = INITIAL;
const int Kp = 15;
const int maxSpeed = 255;
const int minSpeed = 10;

int16_t yaw = 0;
int16_t startingYaw = 0;  // Before we work out which direction to turn, remember what straightahead is
int16_t diff = 0;
long timer = 0;
long backTimer = 0;
long forwardTimer = 0;
float minmax = M_PI - 0.175;


void motor_Init() {
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
}

void driveMotor(Motor_Direction direction, uint8_t rightSpeed, uint8_t leftSpeed) {
  digitalWrite(PIN_Motor_STBY, HIGH);
  switch (direction) {
    case FORWARD:
      digitalWrite(PIN_Motor_AIN_1, HIGH);
      analogWrite(PIN_Motor_PWMA, rightSpeed);
      digitalWrite(PIN_Motor_BIN_1, HIGH);
      analogWrite(PIN_Motor_PWMB, leftSpeed);
      break;
    case BACK:
      digitalWrite(PIN_Motor_AIN_1, LOW);
      analogWrite(PIN_Motor_PWMA, rightSpeed);
      digitalWrite(PIN_Motor_BIN_1, LOW);
      analogWrite(PIN_Motor_PWMB, leftSpeed);
      break;
    case ROTATE_LEFT:
      digitalWrite(PIN_Motor_AIN_1, HIGH);  // forward
      analogWrite(PIN_Motor_PWMA, rightSpeed);
      digitalWrite(PIN_Motor_BIN_1, LOW);  // reverse
      analogWrite(PIN_Motor_PWMB, leftSpeed);
      break;
    case ROTATE_RIGHT:
      digitalWrite(PIN_Motor_AIN_1, LOW);
      analogWrite(PIN_Motor_PWMA, rightSpeed);
      digitalWrite(PIN_Motor_BIN_1, HIGH);
      analogWrite(PIN_Motor_PWMB, leftSpeed);
      break;
    case LEFT_FORWARD:
      digitalWrite(PIN_Motor_AIN_1, HIGH);
      analogWrite(PIN_Motor_PWMA, rightSpeed);
      digitalWrite(PIN_Motor_BIN_1, HIGH);
      analogWrite(PIN_Motor_PWMB, leftSpeed / 2);
    case RIGHT_FORWARD:
      digitalWrite(PIN_Motor_AIN_1, HIGH);
      analogWrite(PIN_Motor_PWMA, rightSpeed / 2);
      digitalWrite(PIN_Motor_BIN_1, HIGH);
      analogWrite(PIN_Motor_PWMB, leftSpeed);
      break;
    case LEFT_BACKWARD:
      digitalWrite(PIN_Motor_AIN_1, LOW);
      analogWrite(PIN_Motor_PWMA, rightSpeed);
      digitalWrite(PIN_Motor_BIN_1, LOW);
      analogWrite(PIN_Motor_PWMB, leftSpeed / 2);
    case RIGHT_BACKWARD:
      digitalWrite(PIN_Motor_AIN_1, LOW);
      analogWrite(PIN_Motor_PWMA, rightSpeed / 2);
      digitalWrite(PIN_Motor_BIN_1, LOW);
      analogWrite(PIN_Motor_PWMB, leftSpeed);
      break;
    case STOP:
      analogWrite(PIN_Motor_PWMA, 0);
      analogWrite(PIN_Motor_PWMB, 0);
      digitalWrite(PIN_Motor_STBY, LOW);
      break;
    default:
      analogWrite(PIN_Motor_PWMA, 0);
      digitalWrite(PIN_Motor_STBY, LOW);
      break;
  }
}

// Make sure direction is in the range -170 to +170
// Which is what the accelerometer returns
int16_t normalise(int16_t dirn_int) {
  dirn_int = ((dirn_int + 180) % 360) - 180;
  // //180 cant be achieved on accelerometer
  // float dirn = (float)dirn_int;
  // Shift into (-170, 170]
  if (dirn_int > 170)
    dirn_int = 170;
  else if (dirn_int < -170)
    dirn_int = -170;
  return dirn_int;
}

float normalise_rad(float dirn) {
  dirn = fmod(dirn, M_PI);  // reduce to [0, π) or (-π, π)
  if (dirn > minmax)
    dirn = minmax;
  else if (dirn <= -minmax)
    dirn = -minmax;
  return dirn;
}

bool rotateTo(int16_t directionRequired) {
  // Rotate the car to the direction that is required
  // 0 is straight ahead (no change)
  //-90 is 90 deg to the left
  //+90 is 90 deg to the right
  directionRequired = normalise(directionRequired);
  yaw = eulerDeg[0];
  startingYaw = yaw;  // Before we work out which direction to turn, remember what straightahead is
  diff = directionRequired - yaw;
  // if (Serial) {
  //   Serial.print("Rotating to: ");
  //   Serial.print(directionRequired);
  //   Serial.print(" Diff: ");
  //   Serial.print(diff);
  //   Serial.print(" straightahead is ");
  //   Serial.println(yaw);
  // }
  timer = 0;

  if (diff > 0) {
    // Rotate right
    driveMotor(ROTATE_RIGHT, 75, 75);
    do {
      delay(ROTATE_CHECK_INTERVAL);
      wdt_reset();
      timer += ROTATE_CHECK_INTERVAL;  // Prevent continuous spinning if something goes wrong
      if (leftGround()) {
        driveMotor(STOP, 0, 0);
        break;
      }
      // if (Serial) {
      //   Serial.print("ROTATE_RIGHT Current yaw: ");
      //   Serial.print(yaw);
      //   Serial.print(" Required yaw: ");
      //   Serial.println(directionRequired);
      // }
      // Check for obstacles when turning
      uint8_t proximity = getProximityState();
      if (checkFrontRightProximity(proximity) && checkRearLeftProximity(proximity)) {
        // We cant rotate this way
        // Try the other way
        diff = -diff;
        backTimer = 0;
        forwardTimer = 0;
        timer = 0;
        break;
      }
      if (checkFrontRightProximity(proximity)) {
        // Cant rotate - need to back up a bit before rotating
        if (forwardTimer > 0) {
          // We dont have much room to the rear, only back up until obstruction gone
          backTimer = ROTATE_CHECK_INTERVAL;
        } else {
          backTimer = 400;
        }
        forwardTimer = 0;
      } else if (checkRearLeftProximity(proximity)) {
        // Cant rotate - need to go forward a bit before rotating
        if (backTimer > 0) {
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
      getAccelerometerEuler();
      yaw = eulerDeg[0];
      diff = directionRequired - yaw;
      // if (timer > 500) {
      //   if (abs(startingYaw - yaw) <= 10) {
      //     //We must have rotated a complete 360 - reset the gyro
      //     driveMotor(STOP, 0, 0);
      //     return false;
      //   }
      // }
      // } while (timer <= 5000 && ((directionRequired > yaw) && ((directionRequired - yaw) < 180)) || ((directionRequired < yaw) && ((directionRequired - yaw) >= 180)));
    } while (diff > 0 && timer < MAX_ROTATE_TIME);
  }
  if (diff < 0) {
    // Rotate left
    driveMotor(ROTATE_LEFT, 75, 75);
    do {
      delay(ROTATE_CHECK_INTERVAL);
      wdt_reset();
      timer += ROTATE_CHECK_INTERVAL;
      if (leftGround()) {
        driveMotor(STOP, 0, 0);
        break;
      }
      // if (Serial) {
      //   Serial.print("ROTATE_LEFT Current yaw: ");
      //   Serial.print(yaw);
      //   Serial.print(" Required yaw: ");
      //   Serial.println(directionRequired);
      // }
      // Check for obstacles when turning
      uint8_t proximity = getProximityState();
      if (checkFrontLeftProximity(proximity) && checkRearRightProximity(proximity)) {
        // We are stuck - stop
        break;
      }
      if (checkFrontLeftProximity(proximity)) {
        // Cant rotate - need to back up a bit before rotating
        if (forwardTimer > 0) {
          // We dont have much room to the rear, only back up until obstruction gone
          backTimer = ROTATE_CHECK_INTERVAL;
        } else {
          backTimer = 400;
        }
        forwardTimer = 0;
      } else if (checkRearRightProximity(proximity)) {
        // Cant rotate - need to go forward a bit before rotating
        if (backTimer > 0) {
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
        driveMotor(ROTATE_LEFT, 75, 75);
      }
      getAccelerometerEuler();
      yaw = eulerDeg[0];
      diff = directionRequired - yaw;
      // if (timer > 500) {
      //   if (abs(startingYaw - yaw) <= 10) {
      //     //We must have rotated a complete 360 - reset the gyro
      //     driveMotor(STOP, 0, 0);
      //     return false;
      //   }
      // }
      // } while (timer <= 5000 && ((directionRequired < yaw) && ((directionRequired - yaw) < 180)) || ((directionRequired > yaw) && ((directionRequired - yaw) >= 180)));
    } while (diff < 0 && timer < MAX_ROTATE_TIME);
  }
  driveMotor(STOP, 0, 0);
  return true;
}

void drive(Motor_Direction direction, float straightAheadRad, uint8_t speed) {
  // Drives the motor in the required direction, allowing for unbalanced wheels / motors /etc.
  // By checking for the yaw in direction and allowing for it by adjusting the motor speed
  // Note: Only required when driving straight!
  uint8_t leftSpeed = speed;
  uint8_t rightSpeed = speed;
  if (direction == FORWARD || direction == BACK) {
    float currentYaw = euler[0];
    // To avoid issues when the we are around 180 deg from the original direction
    // We need to normalise the yaw
    straightAheadRad = normalise_rad(straightAheadRad);
    currentYaw = normalise_rad(currentYaw);
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
  getAccelerometerEuler();
  float forwardDirection = euler[0];
  bool removingPitchOrRoll = false;
  unsigned long backoutTimer = 0;
  do {
    if (leftGround()) {
      break;
    }
    getAccelerometerEuler();
    if (rollingOrPitching()) {
      // Move forward out of trouble
      drive(FORWARD, forwardDirection, 50);
    } else if (removingPitchOrRoll) {
      // Moved out of trouble - stop
      break;
    } else {
      drive(BACK, forwardDirection, 50);
    }
    if (getRearProximity()) {
      // Something behind us - stop reversing
      break;
    }
    delay(50);
    wdt_reset();
    backoutTimer += 50;
  } while (backoutTimer < 500);
  drive(STOP, forwardDirection, 0);
}

void aboutTurn() {
  getAccelerometerEuler();
  int16_t forwardDirection = eulerDeg[0];
  int16_t reversed = normalise(forwardDirection + 170);
  rotateTo(reversed);
}
