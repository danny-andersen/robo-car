#define UNO_PERIPHERAL_ADDR 0X08

//Commands
#define REQ_PROXIMITY_STATE_CMD 0x01     //Requesting the state of the proximity sensors attached to the uno
#define REQ_DIRECTION_TO_DRIVE_CMD 0x02  //Requesting the determination of which direction to drive next (based on obstacles in front)
#define SENDING_OBSTACLES_CMD 0x03       //About to start sending obstacles
#define NEXT_OBSTACLE_CMD 0x04           //The next obstacle in the list
#define MOTOR_STARTING_CMD 0x05           //Motor is starting - reset wheel pulse counters
#define MOTOR_STOPPING_CMD 0x06           //Motor is stopping
#define REQ_STATUS_CMD 0x07           //Return proximity status, distance travelled, current speed

//Data definitions
#define FRONT_LEFT_PROX_BIT 0
#define FRONT_RIGHT_PROX_BIT 1
#define REAR_LEFT_PROX_BIT 2
#define REAR_RIGHT_PROX_BIT 3
#define TOP_FRONT_LEFT_PROX_BIT 4
#define TOP_FRONT_RIGHT_PROX_BIT 5
#define FRONT_LEFT_PROX_SET 0x01
#define FRONT_RIGHT_PROX_SET 0x02
#define REAR_LEFT_PROX_SET 0x04
#define REAR_RIGHT_PROX_SET 0x08
#define TOP_FRONT_LEFT_PROX_SET 0x10
#define TOP_FRONT_RIGHT_PROX_SET 0x20


#define MAX_NUMBER_OF_OBJECTS_IN_SWEEP 20

struct DirectionData {
  uint16_t directionToDrive;
};

struct ObstacleData {
  int8_t relDirection;   //Direction relative to the current compass heading of the centre of the obstacle
  uint8_t width;         //The width in degrees of view of the obstacle in front
  uint16_t avgDistance;  //How far away the obstacle is
};

struct ObstaclesCmd {
  uint16_t currentCompassDirn;   //Current compass direction (i.e. straightahead)
  uint8_t numOfObstaclesToSend;  //Number of obstacles found that are to be sent for analysis
};

struct StatusStruct {
  uint8_t proximityState;  //Status of proximity sensors
  uint8_t currentLeftSpeed; //Current speed of left wheel in cm/s 
  uint8_t currentRightSpeed; //Current speed of right wheel in cm/s
  uint8_t averageSpeed; //Avg speed of current drive in cm/s
  uint16_t distanceTravelled; //Distance travelled since motored started in cm (average of left and right)
};

bool unoQAvailable = true;

ObstaclesCmd obstaclesCmd;
ObstacleData obstacle;
DirectionData direction = { 0 };
StatusStruct periStatus;

bool waitForResponse() {
  unsigned long waitingTime = 0;
  while (Wire.available() == 0 && waitingTime < 500) {
    //Wait for response;
    delay(10);
    waitingTime += 10;
  }
  return Wire.available() != 0;
}

bool rxStatus() {
  bool retStatus = false;
  Wire.requestFrom(UNO_PERIPHERAL_ADDR, sizeof(periStatus));
  if (waitForResponse()) {
    Wire.readBytes((byte *)&periStatus, sizeof(periStatus));
    retStatus = true;
    Serial.print("Received status:");
    Serial.print(" Proximity: ");
    Serial.print(periStatus.proximityState);
    Serial.print(" Left Speed: ");
    Serial.print(periStatus.currentLeftSpeed);
    Serial.print(" Right Speed: ");
    Serial.print(periStatus.currentRightSpeed);
    Serial.print(" Avg Speed: ");
    Serial.print(periStatus.averageSpeed);
    Serial.print(" Distance travelled: ");
    Serial.println(periStatus.distanceTravelled);
  }
  return retStatus;
}


void sendStartMotorCmd() {
  //Send command to peripheral nano that the drive motors are starting
  Wire.beginTransmission(UNO_PERIPHERAL_ADDR);
  Wire.write(MOTOR_STARTING_CMD);
  Wire.endTransmission();
}

bool sendStopMotorCmd() {
  //Send command to peripheral nano that the drive motors are starting
  Wire.beginTransmission(UNO_PERIPHERAL_ADDR);
  Wire.write(MOTOR_STOPPING_CMD);
  Wire.endTransmission();
  //Request a full status response
  return rxStatus();
}

bool getStatusCmd() {
  Wire.beginTransmission(UNO_PERIPHERAL_ADDR);
  Wire.write(REQ_STATUS_CMD);
  Wire.endTransmission();
  //Should get a full status response
  return rxStatus();

}

uint8_t getProximityState() {
  //Send command to peri uno to get status of all proximity sensors
  Wire.beginTransmission(UNO_PERIPHERAL_ADDR);
  Wire.write(REQ_PROXIMITY_STATE_CMD);
  Wire.endTransmission();
  //Request data
  Wire.requestFrom(UNO_PERIPHERAL_ADDR, 1);
  uint8_t proximityState = 0xFF; //Invalid state
  if (waitForResponse()) {
    proximityState = Wire.read();
    if (Serial) {
      Serial.print("Rx proximity state: ");
      Serial.println(proximityState);  // print the character
      Serial.print("Front Left: ");
      Serial.print((proximityState >> FRONT_LEFT_PROX_BIT) & 0x01);
      Serial.print(" Front Right: ");
      Serial.print((proximityState >> FRONT_RIGHT_PROX_BIT) & 0x01);
      Serial.print(" Rear Left: ");
      Serial.print((proximityState >> REAR_LEFT_PROX_BIT) & 0x01);
      Serial.print(" Rear Right: ");
      Serial.print((proximityState >> REAR_RIGHT_PROX_BIT) & 0x01);
      Serial.print(" Top Front Left: ");
      Serial.print((proximityState >> TOP_FRONT_LEFT_PROX_BIT) & 0x01);
      Serial.print(" Top Front RIGHT: ");
      Serial.println((proximityState >> TOP_FRONT_RIGHT_PROX_BIT) & 0x01);
    }
    if (Wire.available() > 0) {
      if (Serial) {
        Serial.print("Still have bytes to read?? : ");
        Serial.println(Wire.available());
      }
      while (Wire.available() > 0) Serial.print(Wire.read());
      Serial.println();
    }
  } else {
    if (Serial) {
      Serial.println("Timed out from UnoQ");
    }
  }
  return proximityState;
}

bool checkFrontRightProximity(uint8_t status) {
  return (status & FRONT_RIGHT_PROX_SET) || (status & TOP_FRONT_RIGHT_PROX_SET);
}

bool getFrontRightProximity() {
  uint8_t status = getProximityState();
  return checkFrontRightProximity(status);
}

bool checkFrontLeftProximity(uint8_t status) {
  return (status & FRONT_LEFT_PROX_SET) || (status & TOP_FRONT_LEFT_PROX_SET);
}

bool getFrontLeftProximity() {
  uint8_t status = getProximityState();
  return checkFrontLeftProximity(status);
}

bool checkTopFrontRightProximity(uint8_t status) {
  return (status & TOP_FRONT_RIGHT_PROX_SET);
}

bool getTopFrontRightProximity() {
  uint8_t status = getProximityState();
  return checkTopFrontRightProximity;
}

bool checkTopFrontLeftProximity(uint8_t status) {
  return (status & TOP_FRONT_LEFT_PROX_SET);
}

bool getTopFrontLeftProximity() {
  uint8_t status = getProximityState();
  return checkTopFrontLeftProximity;
}

bool checkFrontProximity(uint8_t status) {
  return (status & FRONT_LEFT_PROX_SET) || (status & FRONT_RIGHT_PROX_SET) || (status & TOP_FRONT_RIGHT_PROX_SET) || (status & TOP_FRONT_LEFT_PROX_SET);
}

bool getFrontProximity() {
  uint8_t status = getProximityState();
  return checkFrontProximity(status);
}

bool checkRearRightProximity(uint8_t status) {
  return status & REAR_RIGHT_PROX_SET;
}

bool getRearRightProximity() {
  uint8_t status = getProximityState();
  return checkRearRightProximity(status);
}

bool checkRearLeftProximity(uint8_t status) {
  return status & REAR_LEFT_PROX_SET;
}

bool getRearLeftProximity() {
  uint8_t status = getProximityState();
  return checkRearLeftProximity(status);
}

bool checkRearProximity(uint8_t status) {
  return (status & REAR_LEFT_PROX_SET) || (status & REAR_RIGHT_PROX_SET);
}

bool getRearProximity() {
  uint8_t status = getProximityState();
  return checkRearProximity(status);
}

void sendObstacles(uint16_t heading) {
  Serial.println("Sending obstacle cmd...");
  obstaclesCmd.currentCompassDirn = heading;
  obstaclesCmd.numOfObstaclesToSend = MAX_NUMBER_OF_OBJECTS_IN_SWEEP;
  Wire.beginTransmission(UNO_PERIPHERAL_ADDR);
  Wire.write(SENDING_OBSTACLES_CMD);
  Wire.write((byte *)&obstaclesCmd, sizeof(obstaclesCmd));
  Wire.endTransmission();
  Serial.println("Sending obstacles");
  //Now send each obstacle
  for (int i = 0; i < MAX_NUMBER_OF_OBJECTS_IN_SWEEP; i++) {
    obstacle.relDirection = 90 - ((i + 2) * 5);
    obstacle.width = i;
    obstacle.avgDistance = i * 10;
    Wire.beginTransmission(UNO_PERIPHERAL_ADDR);
    Wire.write(NEXT_OBSTACLE_CMD);
    Wire.write((byte *)&obstacle, sizeof(obstacle));
    Wire.endTransmission();
    // Serial.print("Sent obstacle ");
    // Serial.println(i);
  }
}

void getDirectionToDrive() {
  Wire.beginTransmission(UNO_PERIPHERAL_ADDR);
  Wire.write(REQ_DIRECTION_TO_DRIVE_CMD);
  Wire.endTransmission();
  //Request data
  Wire.requestFrom(UNO_PERIPHERAL_ADDR, 2);
  if (waitForResponse()) {

    Wire.readBytes((byte *)&direction, sizeof(direction));
    if (Serial) {
      Serial.print("Received direction to drive: ");
      Serial.println(direction.directionToDrive);
    }
  } else {
    if (Serial) Serial.println("Timed out waiting for response from Uno peri for direction");
  }
}


