#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_BIN_1 8
#define PIN_Motor_AIN_1 7
#define PIN_Motor_STBY 3

#define speed_Max 255

enum Motor_Direction
{
  FORWARD,        //(1)
  BACK,           //(2)
  ROTATE_LEFT,    //(3)
  ROTATE_RIGHT,   //(4)
  LEFT_FORWARD,   //(5)
  LEFT_BACKWARD,  //(6)
  RIGHT_FORWARD,  //(7)
  RIGHT_BACKWARD, //(8)
  STOP,
  INITIAL
};

static unsigned long lastDriveTime = 0;
static Motor_Direction lastDriveDirection = INITIAL;
const int Kp = 15;
const int maxSpeed = 255;
const int minSpeed = 10;

void motor_Init()
{
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
}

void driveMotor(Motor_Direction direction, uint8_t rightSpeed, uint8_t leftSpeed)
{
  digitalWrite(PIN_Motor_STBY, HIGH);
  switch (direction)
  {
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
    digitalWrite(PIN_Motor_AIN_1, HIGH); // forward
    analogWrite(PIN_Motor_PWMA, rightSpeed);
    digitalWrite(PIN_Motor_BIN_1, LOW); // reverse
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
int16_t normalise(int16_t dirn_int)
{
  // dirn = ((dirn + 180) % 360) - 180;
  // //180 cant be achieved on accelerometer
  float dirn = (float)dirn_int;
  dirn = fmod(dirn, 360.0);
  // Shift into (-180, 180]
  if (dirn > 180.0)
    dirn -= 360.0;
  else if (dirn <= -180.0)
    dirn += 360.0;
  if (dirn > 170)
    dirn = 170;
  else if (dirn < -170)
    dirn = -170;
  return (int16_t)lrint(dirn);
}

float normalise_rad(float dirn)
{
  dirn = fmod(dirn, 2.0 * M_PI); // reduce to [0, 2π) or (-2π, 2π)
  if (dirn > M_PI)
    dirn -= 2.0 * M_PI;
  else if (dirn <= -M_PI)
    dirn += 2.0 * M_PI;
  return dirn;
}

bool rotateTo(int16_t directionRequired)
{
  // Rotate the car to the direction that is required
  // 0 is straight ahead (no change)
  //-90 is 90 deg to the left
  //+90 is 90 deg to the right
  directionRequired = normalise(directionRequired);
  float yaw = eulerDeg[0]; 
  float startingYaw = yaw; // Before we work out which direction to turn, remember what straightahead is
  float diff = normalise(directionRequired - yaw);
  // if (Serial) {
  //   Serial.print("Rotating to: ");
  //   Serial.print(directionRequired);
  //   Serial.print(" Diff: ");
  //   Serial.print(diff);
  //   Serial.print(" straightahead is ");
  //   Serial.println(yaw);
  // }

  long timer = 0;
  long backTimer = 0;
  long forwardTimer = 0;
  if (diff > 0)
  {
    // Rotate right
    driveMotor(ROTATE_RIGHT, 75, 75);
    do
    {
      delay(50);
      timer += 50; // Prevent continuous spinning if something goes wrong
      if (leftGround())
      {
        driveMotor(STOP, 0, 0);
        break;
      }
      getAccelerometerEuler();
      yaw = eulerDeg[0];
      diff = normalise(directionRequired - yaw);
      if (diff <= 0) break;
      if (timer > 500) {
        float startDiff = normalise(startingYaw - yaw);
        if (startDiff <= 10) {
          //We must have rotated a complete 360 - reset the gyro
          return false;
        }
      }
      // if (Serial) {
      //   Serial.print("ROTATE_RIGHT Current yaw: ");
      //   Serial.print(yaw);
      //   Serial.print("Required yaw: ");
      //   Serial.println(directionRequired);
      // }
      // Check for obstacles when turning
      uint8_t proximity = getProximityState();
      if (checkFrontRightProximity(proximity) && checkRearLeftProximity(proximity))
      {
        // We cant rotate this way
        // Try the other way
        diff = -diff;
        backTimer = 0;
        forwardTimer = 0;
        timer = 0;
        break;
      }
      if (checkFrontRightProximity(proximity))
      {
        // Cant rotate - need to back up a bit before rotating
        if (forwardTimer > 0)
        {
          // We dont have much room to the rear, only back up until obstruction gone
          backTimer = 50;
        }
        else
        {
          backTimer = 200;
        }
        forwardTimer = 0;
      }
      else if (checkRearLeftProximity(proximity))
      {
        // Cant rotate - need to go forward a bit before rotating
        if (backTimer > 0)
        {
          // Dont have much room to the front, only go forward until obstruction gone
          forwardTimer = 50;
        }
        else
        {
          forwardTimer = 200;
        }
        backTimer = 0;
      }
      if (backTimer > 0)
      {
        driveMotor(BACK, 75, 75);
        backTimer -= 50;
      }
      else if (forwardTimer >= 0)
      {
        driveMotor(FORWARD, 75, 75);
        forwardTimer -= 50;
      }
      else
      {
        driveMotor(ROTATE_RIGHT, 75, 75);
      }
      wdt_reset();
      // } while (timer <= 5000 && ((directionRequired > yaw) && ((directionRequired - yaw) < 180)) || ((directionRequired < yaw) && ((directionRequired - yaw) >= 180)));
    } while (diff > 0 && timer < 3000);
  }
  if (diff < 0)
  {
    // Rotate left
    driveMotor(ROTATE_LEFT, 75, 75);
    do
    {
      delay(50);
      timer += 50;
      if (leftGround())
      {
        driveMotor(STOP, 0, 0);
        break;
      }
      getAccelerometerEuler();
      yaw = eulerDeg[0];
      diff = normalise(directionRequired - yaw);
      if (diff >= 0) break;
      if (timer > 500) {
        float startDiff = normalise(startingYaw - yaw);
        if (startDiff <= 10) {
          //We must have rotated a complete 360 - reset the gyro
          return false;
        }
      }
      // if (Serial) {
      //   Serial.print("ROTATE_LEFT Current yaw: ");
      //   Serial.print(yaw);
      //   Serial.print("Required yaw: ");
      //   Serial.println(directionRequired);
      // }
      // Check for obstacles when turning
      uint8_t proximity = getProximityState();
      if (checkFrontLeftProximity(proximity) && checkRearRightProximity(proximity))
      {
        // We are stuck - stop
        break;
      }
      if (checkFrontLeftProximity(proximity))
      {
        // Cant rotate - need to back up a bit before rotating
        if (forwardTimer > 0)
        {
          // We dont have much room to the rear, only back up until obstruction gone
          backTimer = 50;
        }
        else
        {
          backTimer = 200;
        }
        forwardTimer = 0;
      }
      else if (checkRearRightProximity(proximity))
      {
        // Cant rotate - need to go forward a bit before rotating
        if (backTimer > 0)
        {
          // Dont have much room to the front, only go forward until obstruction gone
          forwardTimer = 50;
        }
        else
        {
          forwardTimer = 200;
        }
        backTimer = 0;
      }
      if (backTimer > 0)
      {
        driveMotor(BACK, 75, 75);
        backTimer -= 50;
      }
      else if (forwardTimer >= 0)
      {
        driveMotor(FORWARD, 75, 75);
        forwardTimer -= 50;
      }
      else
      {
        driveMotor(ROTATE_LEFT, 75, 75);
      }
      wdt_reset();
      // } while (timer <= 5000 && ((directionRequired < yaw) && ((directionRequired - yaw) < 180)) || ((directionRequired > yaw) && ((directionRequired - yaw) >= 180)));
    } while (diff < 0 && timer < 5000);
  }
  driveMotor(STOP, 0, 0);
  return true;
}

void drive(Motor_Direction direction, float straightAheadRad, uint8_t speed)
{
  // Drives the motor in the required direction, allowing for unbalanced wheels / motors /etc.
  // By checking for the yaw in direction and allowing for it by adjusting the motor speed
  // Note: Only required when driving straight!
  uint8_t leftSpeed = speed;
  uint8_t rightSpeed = speed;
  if (direction == FORWARD || direction == BACK)
  {
    float currentYaw = euler[0];
    // To avoid issues when the we are around 180 deg from the original direction
    // We need to normalise the yaw
    straightAheadRad = normalise_rad(straightAheadRad);
    currentYaw = normalise_rad(currentYaw);
    // if (straightAheadRad > HALF_PI) {
    //   straightAheadRad = M_PI - straightAheadRad;
    // } else if (straightAheadRad < -HALF_PI) {
    //   straightAheadRad = -M_PI - straightAheadRad;
    // }
    // if (currentYaw > HALF_PI) {
    //   currentYaw = M_PI - currentYaw;
    // } else if (currentYaw < -HALF_PI) {
    //   currentYaw = -M_PI - currentYaw;
    // }
    leftSpeed = (straightAheadRad - currentYaw) * Kp + speed;
    rightSpeed = (currentYaw - straightAheadRad) * Kp + speed;
    if (leftSpeed > maxSpeed)
    {
      leftSpeed = maxSpeed;
    }
    else if (leftSpeed < minSpeed)
    {
      leftSpeed = minSpeed;
    }
    if (rightSpeed > maxSpeed)
    {
      rightSpeed = maxSpeed;
    }
    else if (rightSpeed < minSpeed)
    {
      rightSpeed = 10;
    }
    if (direction == BACK)
    {
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

void backOut()
{
  getAccelerometerEuler();
  float forwardDirection = euler[0];
  bool removingPitchOrRoll = false;
  unsigned long backoutTimer = 0;
  do
  {
    if (leftGround())
    {
      break;
    }
    getAccelerometerEuler();
    if (rollingOrPitching())
    {
      // Move forward out of trouble
      drive(FORWARD, forwardDirection, 50);
    }
    else if (removingPitchOrRoll)
    {
      // Moved out of trouble - stop
      break;
    }
    else
    {
      drive(BACK, forwardDirection, 100);
    }
    if (getRearProximity())
    {
      // Something behind us - stop reversing
      break;
    }
    delay(50);
    wdt_reset();
    backoutTimer += 50;
  } while (backoutTimer < 500);
  drive(STOP, forwardDirection, 0);
}

void aboutTurn()
{
  getAccelerometerEuler();
  int16_t forwardDirection = eulerDeg[0];
  int16_t reversed = forwardDirection + 180;
  rotateTo(reversed);
}
