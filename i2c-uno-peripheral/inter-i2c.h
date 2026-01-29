#define UNO_PERIPHERAL_ADDR 0x08
#define PI_ADDR 0x09

//Commands
#define REQ_PROXIMITY_STATE_CMD 0x01     //Requesting the state of the proximity sensors attached to the uno
#define REQ_DIRECTION_TO_DRIVE_CMD 0x02  //Requesting the determination of which direction to drive next (based on obstacles in front)
#define SENDING_OBSTACLES_CMD 0x03       //About to start sending obstacles
#define NEXT_OBSTACLE_CMD 0x04           //The next obstacle in the list
#define MOTOR_STARTING_CMD 0x05          //Motor is starting - reset wheel pulse counters
#define MOTOR_STOPPING_CMD 0x06          //Motor is stopping
#define REQ_STATUS_CMD 0x07              //Return proximity status, distance travelled, current speed
#define SEND_SYSTEM_STATUS_CMD 0x08      //Sends system status to PI

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

struct ObstacleData {
  uint16_t bearing;      //Compass heading of the centre of the obstacle
  uint16_t width;        //The width in degrees of view of the obstacle in front
  uint16_t avgDistance;  //How far away the obstacle is
};

struct ObstaclesCmd {
  int16_t currentCompassDirn;    //Current compass direction (i.e. straightahead)
  uint8_t numOfObstaclesToSend;  //Number of obstacles found that are to be sent for analysis
};

struct StatusStruct {
  uint8_t proximityState;      //Status of proximity sensors
  uint8_t currentLeftSpeed;    //Current speed of left wheel in cm/s
  uint8_t currentRightSpeed;   //Current speed of right wheel in cm/s
  uint8_t averageSpeed;        //Avg speed of current drive in cm/s
  uint16_t distanceTravelled;  //Distance travelled since motored started in cm (average of left and right)
};

struct PiStatusStruct {
  uint8_t ready;              //0 == not ready
  uint8_t lidarStatus;        //Bits set according to proximity bits
  uint16_t directionToDrive;  //1000 = direction not set
};

struct SystemStatusStruct {
  int16_t humidity;
  int16_t tempC;
  uint16_t batteryVoltage;
  uint8_t robotState;
  uint8_t proximityState;
  int16_t currentBearing;
  int8_t pitch;
  int8_t roll;
  uint8_t rightWheelSpeed;
  uint8_t leftWheelSpeed;
  uint8_t averageSpeed;
  uint8_t distanceTravelled;
};

bool nanoAvailable = true;

ObstaclesCmd obstaclesCmd;
ObstacleData obstacle;
StatusStruct periStatus;
PiStatusStruct piStatus;
SystemStatusStruct systemStatus;

uint8_t sendStatus = 0;
uint8_t numToReceive = 0;

bool waitForResponse() {
  unsigned long waitingTime = 0;
  while (Wire.available() == 0 && waitingTime < 500) {
    //Wait for response;
    delay(10);
    waitingTime += 10;
  }
  return Wire.available() != 0;
}

void flush() {
  //Read any bytes still in the buffer
  delay(10);
  if (Wire.available() > 0) {
    if (Serial) {
      Serial.print("Still have bytes to read?? : ");
      Serial.println(Wire.available());
    }
    while (Wire.available() > 0) Serial.print(Wire.read());
    Serial.println();
  }
}

int8_t rxStatus() {

  Wire.requestFrom(UNO_PERIPHERAL_ADDR, sizeof(periStatus));
  if (waitForResponse()) {
    Wire.readBytes((byte *)&periStatus, sizeof(periStatus));
    //Copy to system status to send to PI
    systemStatus.proximityState = periStatus.proximityState;
    systemStatus.rightWheelSpeed = periStatus.currentRightSpeed;
    systemStatus.leftWheelSpeed = periStatus.currentLeftSpeed;
    systemStatus.averageSpeed = periStatus.averageSpeed;
    systemStatus.distanceTravelled = periStatus.distanceTravelled;
    sendStatus = 0;
  } else {
    sendStatus = 1;
  }
  return sendStatus;
}

void sendStartMotorCmd() {
  //Send command to peripheral nano that the drive motors are starting
  Wire.beginTransmission(UNO_PERIPHERAL_ADDR);
  Wire.write(MOTOR_STARTING_CMD);
  Wire.endTransmission();
}

int8_t sendStopMotorCmd() {
  //Send command to peripheral nano that the drive motors are starting
  Wire.beginTransmission(UNO_PERIPHERAL_ADDR);
  Wire.write(MOTOR_STOPPING_CMD);
  sendStatus = Wire.endTransmission();
  if (!sendStatus) {
    return rxStatus();
  } else {
    return sendStatus;
  }
}

int8_t getStatusCmd() {
  Wire.beginTransmission(UNO_PERIPHERAL_ADDR);
  Wire.write(REQ_STATUS_CMD);
  sendStatus = Wire.endTransmission();
  //Should get a full status response
  if (!sendStatus) {
    return rxStatus();
  } else {
    return sendStatus;
  }
}

int8_t readPiStatus() {
  Wire.requestFrom(PI_ADDR, sizeof(piStatus));
  if (waitForResponse()) {
    //Should get pi status back acknowledging receipt
    Wire.readBytes((byte *)&piStatus, sizeof(piStatus));
    if (piStatus.ready == 1) {
      sendStatus = 0;  //Correct ack
    } else {
      sendStatus = piStatus.ready;  //Incorrect status read
    }
  }
  flush();
  return sendStatus;
}

int8_t getPiStatusCmd() {
  Wire.beginTransmission(PI_ADDR);
  Wire.write(REQ_STATUS_CMD);
  sendStatus = Wire.endTransmission();
  //Should get a status response
  if (!sendStatus) {
    sendStatus = readPiStatus();
  }
  return sendStatus;
}

void getCombinedProximity() {
  bool status = false;
  int8_t piOnBus = 1;
  do {
    //Try and read a valid proximatey status continuously
    //Eventually the watchdog will trigger if cant get it - we cant proceed without it
    status = getStatusCmd();
    //Get the PI status, which includes the LIDAR proximity state, with the same bits set as the proximity sensors on the nano
    piOnBus = getPiStatusCmd();
  } while (!status && piOnBus);
  //OR the lidar proximity status in with the IR proximity sensors
  systemStatus.proximityState |= piStatus.lidarStatus;
}

uint8_t getProximityState() {
  //Send command to peri uno to get status of all proximity sensors
  Wire.beginTransmission(UNO_PERIPHERAL_ADDR);
  Wire.write(REQ_PROXIMITY_STATE_CMD);
  sendStatus = Wire.endTransmission();
  //Request data
  if (!sendStatus) {
    Wire.requestFrom(UNO_PERIPHERAL_ADDR, 1);
    if (waitForResponse()) {
      systemStatus.proximityState = Wire.read();  // if (Serial) {
      //   Serial.print("Rx proximity state: ");
      //   Serial.println(proximityState);  // print the character
      //   Serial.print("Front Left: ");
      //   Serial.print((proximityState >> FRONT_LEFT_PROX_BIT) & 0x01);
      //   Serial.print(" Front Right: ");
      //   Serial.print((proximityState >> FRONT_RIGHT_PROX_BIT) & 0x01);
      //   Serial.print(" Rear Left: ");
      //   Serial.print((proximityState >> REAR_LEFT_PROX_BIT) & 0x01);
      //   Serial.print(" Rear Right: ");
      //   Serial.print((proximityState >> REAR_RIGHT_PROX_BIT) & 0x01);
      //   Serial.print(" Top Front Left: ");
      //   Serial.print((proximityState >> TOP_FRONT_LEFT_PROX_BIT) & 0x01);
      //   Serial.print(" Top Front RIGHT: ");
      //   Serial.println((proximityState >> TOP_FRONT_RIGHT_PROX_BIT) & 0x01);
      // }
      // if (Wire.available() > 0) {
      //   if (Serial) {
      //     Serial.print("Still have bytes to read?? : ");
      //     Serial.println(Wire.available());
      //   }
      //   while (Wire.available() > 0) Serial.print(Wire.read());
      //   Serial.println();
      // }
    } else {
      sendStatus = 1;
      // if (Serial) {
      //   Serial.println("Timed out from Nano");
      // }
    }
  }
  return sendStatus;
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
  //Get the PI status, which includes the LIDAR proximity state
  getPiStatusCmd();
  //OR this in with the IR proximity sensors
  status |= piStatus.lidarStatus;
  return checkFrontLeftProximity(status);
}

bool checkFrontProximity(uint8_t status) {
  return (status & FRONT_LEFT_PROX_SET) || (status & FRONT_RIGHT_PROX_SET) || (status & TOP_FRONT_RIGHT_PROX_SET) || (status & TOP_FRONT_LEFT_PROX_SET);
}

bool getFrontProximity() {
  uint8_t status = getProximityState();
  //Get the PI status, which includes the LIDAR proximity state
  getPiStatusCmd();
  //OR this in with the IR proximity sensors
  status |= piStatus.lidarStatus;
  return checkFrontProximity(status);
}

bool checkRearRightProximity(uint8_t status) {
  return status & REAR_RIGHT_PROX_SET;
}

bool getRearRightProximity() {
  uint8_t status = getProximityState();
  //Get the PI status, which includes the LIDAR proximity state
  getPiStatusCmd();
  //OR this in with the IR proximity sensors
  status |= piStatus.lidarStatus;
  return checkRearRightProximity(status);
}

bool checkRearLeftProximity(uint8_t status) {
  return status & REAR_LEFT_PROX_SET;
}

bool getRearLeftProximity() {
  uint8_t status = getProximityState();
  //Get the PI status, which includes the LIDAR proximity state
  getPiStatusCmd();
  //OR this in with the IR proximity sensors
  status |= piStatus.lidarStatus;
  return checkRearLeftProximity(status);
}

bool checkRearProximity(uint8_t status) {
  return (status & REAR_LEFT_PROX_SET) || (status & REAR_RIGHT_PROX_SET);
}

bool getRearProximity() {
  systemStatus.proximityState = getProximityState();
  //Get the PI status, which includes the LIDAR proximity state
  getPiStatusCmd();
  //OR this in with the IR proximity sensors
  systemStatus.proximityState |= piStatus.lidarStatus;
  return checkRearProximity(systemStatus.proximityState);
}

int8_t sendSystemStatus() {
  Wire.beginTransmission(PI_ADDR);
  Wire.write(SEND_SYSTEM_STATUS_CMD);
  Wire.write((byte *)&systemStatus, sizeof(systemStatus));
  sendStatus = Wire.endTransmission();
  if (!sendStatus) {
    //Should get PI status in response
    sendStatus = readPiStatus();
  }
  return sendStatus;
}

