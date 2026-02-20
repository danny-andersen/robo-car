#ifndef NANO_COMMS_H
#define NANO_COMMMS_H


#define UNO_PERIPHERAL_ADDR 0x08
#define PI_ADDR 0x09

//Commands
#define REQ_DIRECTION_TO_DRIVE_CMD 0x02  //Requesting the determination of which direction to drive next (based on obstacles in front)
#define SENDING_OBSTACLES_CMD 0x03       //About to start sending obstacles
#define NEXT_OBSTACLE_CMD 0x04           //The next obstacle in the list
#define MOTOR_STARTING_CMD 0x05          //Motor is starting - reset wheel pulse counters
#define MOTOR_STOPPING_CMD 0x06          //Motor is stopping
#define REQ_STATUS_CMD 0x07              //Return proximity status, distance travelled, current speed
#define SEND_SYSTEM_STATUS_CMD 0x08      //Sends system status to PI
#define PI_STATUS_RESPONSE 0x87      //PI system status response

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
#define REAR_REAR_PROX_SET 0x40 // Extra bit for rear-most proximity (lidar only)
#define FRONT_FRONT_PROX_SET 0x80 // Extra bit for rear-most proximity (lidar only)


#define MAX_NUMBER_OF_OBJECTS_IN_SWEEP 20

#define I2C_RETRY_CNT 2     //Number of times to try to send a command to the Nano or the PI

uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;        // initial value
    const uint8_t poly = 0x31; // polynomial x^8 + x^5 + x^4 + 1

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ poly;
            else
                crc <<= 1;
        }
    }
    return crc;
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
    //   D_print("Still have bytes to read?? : ");
    //   D_println(Wire.available());
    // D_println();
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
      systemStatus.proximityState = nanoStatus.proximityState | piStatus.lidarStatus;  //OR in the current Lidar status with the Proximity sensors
      systemStatus.rightWheelSpeed = nanoStatus.currentRightSpeed;
      systemStatus.leftWheelSpeed = nanoStatus.currentLeftSpeed;
      systemStatus.averageSpeed = nanoStatus.averageSpeed;
      systemStatus.distanceTravelled = nanoStatus.distanceTravelled;
      //   D_print("Rx proximity state: ");
      //   D_println(systemStatus.proximityState);  // print the character
      //   D_print("Front Left: ");
      //   D_print((systemStatus.proximityState >> FRONT_LEFT_PROX_BIT) & 0x01);
      //   D_print(" Front Right: ");
      //   D_print((systemStatus.proximityState >> FRONT_RIGHT_PROX_BIT) & 0x01);
      //   D_print(" Rear Left: ");
      //   D_print((systemStatus.proximityState >> REAR_LEFT_PROX_BIT) & 0x01);
      //   D_print(" Rear Right: ");
      //   D_print((systemStatus.proximityState >> REAR_RIGHT_PROX_BIT) & 0x01);
      //   D_print(" Top Front Left: ");
      //   D_print((systemStatus.proximityState >> TOP_FRONT_LEFT_PROX_BIT) & 0x01);
      //   D_print(" Top Front RIGHT: ");
      //   D_println((systemStatus.proximityState >> TOP_FRONT_RIGHT_PROX_BIT) & 0x01);
      // }
    }
    nanoCommsError = 0;
  } else {
    nanoCommsError = 1;
  }
  return nanoCommsError;
}

int8_t sendNanoCmd(uint8_t cmd) {
  flushBus();
  uint8_t retryCnt = 0;
  bool reqFailed = false;
  do {
    reqFailed = false;
    Wire.beginTransmission(UNO_PERIPHERAL_ADDR);
    Wire.write(cmd);
    nanoCommsError = Wire.endTransmission();
    if (!nanoCommsError) {
      nanoCommsError = rxNanoStatus();
    } else {
      reqFailed = true;
      systemStatus.errorField = nanoCommsError;
      if (nanoCommsError == 5) {
        //Reset timeout
        Wire.clearWireTimeoutFlag();
      }
    }
  } while (nanoCommsError && retryCnt++ < I2C_RETRY_CNT);

  if (nanoCommsError) {
    if (reqFailed) {
      systemStatus.errorField = nanoCommsError;
    } else {
      //errorfield set by read
    }
  } else if (retryCnt > 1) {
    systemStatus.errorField = I2C_NANO_RETRIED;
  }
  return nanoCommsError;
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


#endif