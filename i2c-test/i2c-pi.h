#define PI_REQUEST_INTERVAL 50  //Required gap (ms) between commands being sent to the PI, to give time for PI to dequeue and process the previous command
#define I2C_RETRY_CNT 2 //Number of times to try to send a command to the Nano or the PI


struct ObstacleData {
  uint8_t obstacleNo;
  uint16_t bearing;      //Compass heading of the centre of the obstacle
  uint16_t width;        //The width in degrees of view of the obstacle in front
  uint16_t avgDistance;  //How far away the obstacle is
  uint8_t checksum;
};

struct ObstaclesCmd {
  int16_t currentCompassDirn;    //Current compass direction (i.e. straightahead)
  uint8_t numOfObstaclesToSend;  //Number of obstacles found that are to be sent for analysis
  uint8_t checksum;
};

struct PiStatusStruct {
  uint8_t ready;              //0 == not ready
  uint8_t lidarStatus;        //Bits set according to proximity bits
  uint16_t directionToDrive;  //1000 = direction not set
  uint8_t checksum;
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
  uint8_t errorField;  //Any errors (currently I2C errors)
  uint8_t checksum;
};

ObstaclesCmd obstaclesCmd;
ObstacleData obstacle;
StatusStruct rdstatus;
PiStatusStruct piStatus, rdpiStatus;
SystemStatusStruct systemStatus;

uint8_t numToReceive = 0;

int8_t piOnBus = 1;    //0 means up and on the bus
int8_t nanoOnBus = 1;  //0 means up and on the bus - anything else is an error

long lastPiRequestTime = 0;  //The time of the last request to the PI - we need a gap of >20ms between commands to the PI, to give it time to process the last request

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


bool waitForResponse(uint8_t noOfBytes) {
  unsigned long waitingTime = 0;
  while (Wire.available() < noOfBytes && waitingTime < 100) {
    //Wait for response;
    delay(10);
    waitingTime += 10;
  }
  if (Wire.available() != noOfBytes) {
    systemStatus.errorField = I2C_RX_TIMEOUT;
    return false;
  } else {
    return true;
  }
}

void flushBus() {
  //Read any bytes still in the buffer
  if (Wire.available() > 0) {
    // if (Serial) {
    //   Serial.print("Still have bytes to read?? : ");
    //   Serial.println(Wire.available());
    // Serial.println();
    // }
    while (Wire.available() > 0) Wire.read();
    systemStatus.errorField = I2C_EXTRA_BYTES;
  }
}

int8_t rxNanoStatus() {
  Wire.requestFrom(UNO_PERIPHERAL_ADDR, sizeof(StatusStruct));
  if (waitForResponse(sizeof(StatusStruct))) {
    Wire.readBytes((byte *)&rdstatus, sizeof(StatusStruct));
    uint8_t calc = crc8((uint8_t *)&rdstatus, sizeof(StatusStruct) - 1);
    if (calc != rdstatus.checksum) {
      // BAD PACKET
      systemStatus.errorField = I2C_NANO_CRC_ERROR;
      return 1;
    } else {
      nanoStatus = rdstatus;
      //Copy to system status to send to PI
      systemStatus.proximityState = nanoStatus.proximityState;
      systemStatus.rightWheelSpeed = nanoStatus.currentRightSpeed;
      systemStatus.leftWheelSpeed = nanoStatus.currentLeftSpeed;
      systemStatus.averageSpeed = nanoStatus.averageSpeed;
      systemStatus.distanceTravelled = nanoStatus.distanceTravelled;
      //   Serial.print("Rx proximity state: ");
      //   Serial.println(systemStatus.proximityState);  // print the character
      //   Serial.print("Front Left: ");
      //   Serial.print((systemStatus.proximityState >> FRONT_LEFT_PROX_BIT) & 0x01);
      //   Serial.print(" Front Right: ");
      //   Serial.print((systemStatus.proximityState >> FRONT_RIGHT_PROX_BIT) & 0x01);
      //   Serial.print(" Rear Left: ");
      //   Serial.print((systemStatus.proximityState >> REAR_LEFT_PROX_BIT) & 0x01);
      //   Serial.print(" Rear Right: ");
      //   Serial.print((systemStatus.proximityState >> REAR_RIGHT_PROX_BIT) & 0x01);
      //   Serial.print(" Top Front Left: ");
      //   Serial.print((systemStatus.proximityState >> TOP_FRONT_LEFT_PROX_BIT) & 0x01);
      //   Serial.print(" Top Front RIGHT: ");
      //   Serial.println((systemStatus.proximityState >> TOP_FRONT_RIGHT_PROX_BIT) & 0x01);
      // }
    }
    nanoOnBus = 0;
  } else {
    nanoOnBus = 1;
  }
  return nanoOnBus;
}

int8_t sendNanoCmd(uint8_t cmd) {
  flushBus();
  uint8_t retryCnt = 0;
  bool reqFailed = false;
  do {
    reqFailed = false;
    Wire.beginTransmission(UNO_PERIPHERAL_ADDR);
    Wire.write(cmd);
    nanoOnBus = Wire.endTransmission();
    if (!nanoOnBus) {
      nanoOnBus = rxNanoStatus();
    } else {
      reqFailed = true;
      systemStatus.errorField = nanoOnBus;
      if (nanoOnBus == 5) {
        //Reset timeout
        Wire.clearWireTimeoutFlag();
      }
    }
  } while (nanoOnBus && retryCnt++ < I2C_RETRY_CNT);

  if (nanoOnBus) {
    if (reqFailed) {
      systemStatus.errorField = nanoOnBus;
    } else {
      //errorfield set by read
    }
  } else if (retryCnt > 1) {
    systemStatus.errorField = I2C_NANO_RETRIED;
  }
  return nanoOnBus;
}

int8_t sendStartMotorCmd() {
  //Send command to peripheral nano that the drive motors are starting
  return sendNanoCmd(MOTOR_STARTING_CMD);
}

int8_t sendStopMotorCmd() {
  return sendNanoCmd(MOTOR_STOPPING_CMD);
}

int8_t getNanoStatusCmd() {
  return sendNanoCmd(REQ_STATUS_CMD);
}

int8_t readPiStatus() {
  Wire.requestFrom(PI_ADDR, sizeof(PiStatusStruct));
  if (waitForResponse(sizeof(PiStatusStruct))) {
    //Should get pi status back acknowledging receipt
    Wire.readBytes((byte *)&rdpiStatus, sizeof(PiStatusStruct));
    uint8_t calc = crc8((uint8_t *)&rdpiStatus, sizeof(PiStatusStruct) - 1);
    if (calc == rdpiStatus.checksum && rdpiStatus.ready == 1) {
      piStatus = rdpiStatus;
      piOnBus = 0;  //Correct ack
    } else {
      systemStatus.errorField = I2C_PI_CRC_ERROR;
      // if (Serial && calc != rdpiStatus.checksum) {
      //   Serial.print("Read PI: Incorrect crc rx: ");
      //   Serial.print(rdpiStatus.checksum);
      //   Serial.print(" exp: ");
      //   Serial.println(calc);
      // }
      piOnBus = 1;  //Incorrect status read
    }
  }
  flushBus();
  return piOnBus;
}

int8_t sendPiCmd(uint8_t cmd, byte *data = 0, int dataSize = 0) {
  uint8_t retryCnt = 0;
  bool reqFailed = false;
  do {
    long txGap = millis() - lastPiRequestTime;
    if (txGap < PI_REQUEST_INTERVAL) {
      //Wait for PI to be ready to receive next message
      delay(PI_REQUEST_INTERVAL - txGap);
    }
    flushBus();
    reqFailed = false;
    Wire.beginTransmission(PI_ADDR);
    Wire.write(cmd);
    if (dataSize != 0) {
      //Send command data
      Wire.write(data, dataSize);
    }
    piOnBus = Wire.endTransmission();
    //Should get a status response
    if (!piOnBus) {
      piOnBus = readPiStatus();
    } else {
      reqFailed = true;
    }
    lastPiRequestTime = millis();
  } while (piOnBus && retryCnt++ < I2C_RETRY_CNT);
  if (piOnBus) {
    if (reqFailed) {
      systemStatus.errorField = piOnBus + 5;
    } else {
      //errorField set by read
    }
  } else if (retryCnt > 1) {
    systemStatus.errorField = I2C_PI_RETRIED;
  }
  return piOnBus;
}

int8_t getPiStatusCmd() {
  return sendPiCmd(REQ_STATUS_CMD);
}

void getCombinedProximity() {
  do {
    //Try and read a valid proximatey status continuously
    //Eventually the watchdog will trigger if cant get it - we cant proceed without it
    nanoOnBus = getNanoStatusCmd();
  } while (nanoOnBus);
  //Get the PI status, which includes the LIDAR proximity state, with the same bits set as the proximity sensors on the nano
  piOnBus = getPiStatusCmd();
  if (!piOnBus) {
    //OR the lidar proximity status in with the IR proximity sensors
    systemStatus.proximityState |= piStatus.lidarStatus;
  }
}

int8_t sendSystemStatus() {
  systemStatus.checksum = crc8((uint8_t *)&systemStatus, sizeof(systemStatus) - 1);
  piOnBus = sendPiCmd(SEND_SYSTEM_STATUS_CMD, (byte *)&systemStatus, sizeof(systemStatus));
  if (!piOnBus) {
    systemStatus.errorField = 0;  //Zero error field as PI has logged it
  }
  return piOnBus;
}

int8_t sendObstacles(uint16_t heading, uint8_t numObjects, Arc *arcp) {
  // Serial.println("Sending obstacle cmd...");
  obstaclesCmd.currentCompassDirn = heading;
  obstaclesCmd.numOfObstaclesToSend = numObjects;
  obstaclesCmd.checksum = crc8((uint8_t *)&obstaclesCmd, sizeof(obstaclesCmd) - 1);
  int8_t retryCnt = 0;
  do {
    piOnBus = sendPiCmd(SENDING_OBSTACLES_CMD, (byte *)&obstaclesCmd, sizeof(obstaclesCmd));
    if (!piOnBus) {
      //In business
      //Now send each obstacle
      // Serial.println("Sending obstacles");
      int i = 0;
      for (; i < numObjects && !piOnBus; i++) {
        obstacle.obstacleNo = i;
        obstacle.bearing = normalise(heading + (SERVO_CENTRE - arcp->centreDirection));
        obstacle.width = arcp->width;
        obstacle.avgDistance = arcp->avgDistance;
        obstacle.checksum = crc8((uint8_t *)&obstacle, sizeof(obstacle) - 1);
        arcp++;
        piOnBus = sendPiCmd(NEXT_OBSTACLE_CMD, (byte *)&obstacle, sizeof(obstacle));
      }
      if (piOnBus) {
        if (Serial) {
          Serial.print("Failed to send all obs - sent: ");
          Serial.println(i);
        }
      }
    }
  } while (piOnBus && retryCnt++ < 2);
  return piOnBus;
}
