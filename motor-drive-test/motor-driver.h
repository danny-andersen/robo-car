#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_BIN_1 8
#define PIN_Motor_AIN_1 7
#define PIN_Motor_STBY 3

#define speed_Max 255

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
      digitalWrite(PIN_Motor_AIN_1, HIGH);  //forward
      analogWrite(PIN_Motor_PWMA, rightSpeed);
      digitalWrite(PIN_Motor_BIN_1, LOW);  //reverse
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

void rotateTo(int16_t directionRequired) {
  //Rotate the car to the direction that is required
  //0 is straight ahead (no change)
  //-90 is 90 deg to the left
  //+90 is 90 deg to the right
  getAccelerometerEuler();
  float yaw = eulerDeg[0];  //Before we work out which direction to turn, remember what straightahead is
  float lastYaw = yaw;
  if (Serial.available()) {
    Serial.print("Rotating to: ");
    Serial.print(directionRequired);
    Serial.print(" straightahead is ");
    Serial.println(yaw);
  }

  if (directionRequired > yaw) {
    //Rotate right
    driveMotor(ROTATE_RIGHT, 50, 50);
    do {
      delay(20);
      getAccelerometerEuler();
      yaw = eulerDeg[0];
      if (abs(yaw - lastYaw) > 200) {
        //We have flipped over 180 and so change polarity
        yaw = directionRequired;
      }
      lastYaw = yaw;
      if (Serial.available()) {
        Serial.print("Current yaw: ");
        Serial.print(yaw);
        Serial.print("Required yaw: ");
        Serial.println(directionRequired);
      }
    } while (directionRequired > yaw);
  } else if (directionRequired < yaw) {
    //Rotate left
    driveMotor(ROTATE_LEFT, 50, 50);
    do {
      getAccelerometerEuler();
      yaw = eulerDeg[0];
      if (abs(yaw - lastYaw) > 200) {
        //We have flipped over 180 and so change polarity
        yaw = directionRequired;
      }
      lastYaw = yaw;
      if (Serial.available()) {
        Serial.print("Current yaw: ");
        Serial.print(yaw);
        Serial.print("Required yaw: ");
        Serial.println(directionRequired);
      }
    } while (directionRequired < yaw);
  }
  driveMotor(STOP, 0, 0);
}

void drive(Motor_Direction direction, float straightAheadRad, uint8_t speed) {
  //Drives the motor in the required direction, allowing for unbalanced wheels / motors /etc.
  //By checking for the yaw in direction and allowing for it by adjusting the motor speed
  //Note: Only required when driving straight!
  uint8_t leftSpeed = speed;
  uint8_t rightSpeed = speed;
  if (direction == FORWARD || direction == BACK) {
    getAccelerometerEuler();  //Note that if this fails, as no more recent data received, then we just use the last calculated value
    float currentYaw = euler[0];
    //To avoid issues when the we are around 180 deg from the original direction
    //We need to normalise the yaw to pi/2 rather than to pi
    if (straightAheadRad > HALF_PI) {
      straightAheadRad = M_PI - straightAheadRad;
    } else if (straightAheadRad < -HALF_PI) {
      straightAheadRad = -M_PI - straightAheadRad;
    }
    if (currentYaw > HALF_PI) {
      currentYaw = M_PI - currentYaw;
    } else if (currentYaw < -HALF_PI) {
      currentYaw = -M_PI - currentYaw;
    }
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
      //Reverse the left and right speeds, as need to reverse the yaw adjustment
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
  float forward = euler[0];
  unsigned long backoutTimer = 0;
  do {
    drive(BACK, forward, 100);
    delay(50);
    backoutTimer += 50;
  } while (backoutTimer < 1000);
  drive(STOP, forward, 0);
}

void aboutTurn() {
  getAccelerometerEuler();
  int16_t forwardDirection = eulerDeg[0];
  int16_t reversed = forwardDirection + 180;
  reversed = ((reversed + 180) % 360) - 180;
  //180 cant be achieved on accelerometer
  if (reversed >= 179) reversed = 179;
  else if (reversed <= -179) reversed = -179;
  rotateTo(reversed);
}
