struct ObstacleData {
  uint16_t bearing;      //Compass heading of the centre of the obstacle
  uint16_t width;        //The width in degrees of view of the obstacle in front
  uint16_t avgDistance;  //How far away the obstacle is
  uint8_t checksum;
};

struct ObstaclesCmd {
  int16_t currentCompassDirn;    //Current compass direction (i.e. straightahead)
  uint8_t numOfObstaclesToSend;  //Number of obstacles found that are to be sent for analysis
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


bool waitForResponse(uint8_t noOfBytes) {
  unsigned long waitingTime = 0;
  while (Wire.available() < noOfBytes && waitingTime < 500) {
    //Wait for response;
    delay(10);
    waitingTime += 10;
  }
  return Wire.available() >= noOfBytes;
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

int8_t rxNanoStatus() {
  Wire.requestFrom(UNO_PERIPHERAL_ADDR, sizeof(StatusStruct));
  if (waitForResponse(sizeof(StatusStruct))) {
    Wire.readBytes((byte *)&rdstatus, sizeof(StatusStruct));
    uint8_t calc = crc8((uint8_t *)&rdstatus, sizeof(StatusStruct) - 1);
    if (calc != rdstatus.checksum) {
      // BAD PACKET
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

void sendStartMotorCmd() {
  //Send command to peripheral nano that the drive motors are starting
  Wire.beginTransmission(UNO_PERIPHERAL_ADDR);
  Wire.write(MOTOR_STARTING_CMD);
  nanoOnBus = Wire.endTransmission();
  if (!nanoOnBus) {
    return rxNanoStatus();
  } else {
    return nanoOnBus;
  }
}

int8_t sendStopMotorCmd() {
  //Send command to peripheral nano that the drive motors are starting
  Wire.beginTransmission(UNO_PERIPHERAL_ADDR);
  Wire.write(MOTOR_STOPPING_CMD);
  nanoOnBus = Wire.endTransmission();
  if (!nanoOnBus) {
    return rxNanoStatus();
  } else {
    return nanoOnBus;
  }
}

int8_t getNanoStatusCmd() {
  Wire.beginTransmission(UNO_PERIPHERAL_ADDR);
  Wire.write(REQ_STATUS_CMD);
  nanoOnBus = Wire.endTransmission();
  //Should get a full status response
  if (!nanoOnBus) {
    return rxNanoStatus();
  } else {
    return nanoOnBus;
  }
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
      if (Serial && calc != rdpiStatus.checksum) {
        Serial.print("Read PI: Incorrect crc rx: ");
        Serial.print(rdpiStatus.checksum);
        Serial.print(" exp: ");
        Serial.println(calc);
      }
      piOnBus = 1;  //Incorrect status read
    }
  }
  flush();
  return piOnBus;
}

int8_t getPiStatusCmd() {
  Wire.beginTransmission(PI_ADDR);
  Wire.write(REQ_STATUS_CMD);
  piOnBus = Wire.endTransmission();
  //Should get a status response
  if (!piOnBus) {
    piOnBus = readPiStatus();
  }
  return piOnBus;
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

int8_t sendSystemStatus() {
  systemStatus.checksum = crc8((uint8_t *)&systemStatus, sizeof(systemStatus) - 1);
  Wire.beginTransmission(PI_ADDR);
  Wire.write(SEND_SYSTEM_STATUS_CMD);
  Wire.write((byte *)&systemStatus, sizeof(systemStatus));
  piOnBus = Wire.endTransmission();
  if (!piOnBus) {
    //Should get PI status in response
    piOnBus = readPiStatus();
  }
  return piOnBus;
}

int8_t sendObstacles(uint16_t heading, uint8_t numObjects, Arc *arcp) {
  // Serial.println("Sending obstacle cmd...");
  obstaclesCmd.currentCompassDirn = heading;
  obstaclesCmd.numOfObstaclesToSend = numObjects;
  Wire.beginTransmission(PI_ADDR);
  Wire.write(SENDING_OBSTACLES_CMD);
  Wire.write((byte *)&obstaclesCmd, sizeof(obstaclesCmd));
  piOnBus = Wire.endTransmission();
  if (!piOnBus) {
    //Should get PI status in response
    piOnBus = readPiStatus();
    if (!piOnBus) {
      //In business
      //Now send each obstacle
      // Serial.println("Sending obstacles");
      int i = 0;
      for (; i < numObjects && !piOnBus; i++) {
        obstacle.bearing = normalise(heading + (SERVO_CENTRE - arcp->centreDirection));
        obstacle.width = arcp->width;
        obstacle.avgDistance = arcp->avgDistance;
        arcp++;
        Wire.beginTransmission(PI_ADDR);
        Wire.write(NEXT_OBSTACLE_CMD);
        Wire.write((byte *)&obstacle, sizeof(obstacle));
        Wire.endTransmission();
        // Serial.print("Sent obstacle ");
        // Serial.println(i);
        //Should get PI status in response
        piOnBus = readPiStatus();
      }
      // if (piOnBus) {
      //   if (Serial) {
      //     Serial.print("Failed to send all obstacles, sent: ");
      //     Serial.println(i);
      //   }
      // }
    }
  }
  return piOnBus;
}
