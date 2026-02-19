#define PI_MSG_TIMEOUT 50  //Timeout reading response from PI

uint8_t seqCounter = 0;
uint8_t numToReceive = 0;

uint16_t crc16_update(uint16_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++)
    crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  return crc;
}

uint16_t crc16(const uint8_t *buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++)
    crc = crc16_update(crc, buf[i]);
  return crc;
}

void uartFlushInput() {
  while (Serial.available()) {
    Serial.read();
  }
}
int8_t readPiStatusResponse(uint8_t expectedSeq) {
  const uint8_t expectedType = PI_STATUS_RESPONSE;
  const uint8_t payloadLen = sizeof(PiStatusStruct);

  // 1. Find start byte
  unsigned long start = millis();
  while (true) {
    if (Serial.available() && Serial.read() == 0xAA) break;
    if (millis() - start > PI_MSG_TIMEOUT) return PI_TIMEOUT;
  }

  // 2. Read msgType + seq
  uint8_t hdr[2];
  if (Serial.readBytes(hdr, 2) != 2) return PI_HEADER_LEN_ERR;

  uint8_t msgType = hdr[0];
  uint8_t seq = hdr[1];

  if (msgType != expectedType) return PI_MSG_TYPE_ERROR;
  if (seq != expectedSeq) {
    D_print("Seq error: got: ");
    D_print(seq);
    D_print(" Expected: ");
    D_println(expectedSeq);
    return PI_SEQ_ERROR;
  }

  // 3. Read payload + CRC
  uint8_t buf[payloadLen + 2];
  if (Serial.readBytes(buf, payloadLen + 2) != payloadLen + 2) return PI_DATA_LEN_ERROR;

  uint16_t recvCrc = buf[payloadLen] | (buf[payloadLen + 1] << 8);

  // 4. Compute CRC
  uint8_t crcBuf[2 + payloadLen];
  crcBuf[0] = msgType;
  crcBuf[1] = seq;
  memcpy(&crcBuf[2], buf, payloadLen);

  uint16_t calcCrc = crc16(crcBuf, sizeof(crcBuf));
  if (calcCrc != recvCrc) return PI_CRC_ERROR;

  // 5. Copy struct
  memcpy(&piStatus, buf, payloadLen);
  return 0;
}

int8_t uartSendCommand(uint8_t cmd,
                       const void *payload,
                       uint8_t payloadLen) {
  // Flush stale Pi replies
  uartFlushInput();

  uint8_t seq = seqCounter++;

  // ---- Build CRC buffer ----
  uint8_t crcBuf[2 + payloadLen];
  crcBuf[0] = cmd;
  crcBuf[1] = seq;
  if (payloadLen > 0)
    memcpy(&crcBuf[2], payload, payloadLen);

  uint16_t crc = crc16(crcBuf, 2 + payloadLen);

  // ---- Send frame ----
  Serial.write(0xAA);
  Serial.write(cmd);
  Serial.write(seq);
  if (payloadLen > 0)
    Serial.write((uint8_t *)payload, payloadLen);
  Serial.write(crc & 0xFF);
  Serial.write(crc >> 8);

  // ---- Always read PiStatusStruct reply ----
  piCommsError = readPiStatusResponse(seq);
  return piCommsError;
}

int8_t getPiStatusCmd() {
  uartSendCommand(REQ_STATUS_CMD, nullptr, 0);
  if (piCommsError) {
    systemStatus.errorField = piCommsError;  //Log error in the status byte
  }
  return piCommsError;
}

int8_t sendSystemStatus() {
  systemStatus.timestamp = millis();
  uartSendCommand(SEND_SYSTEM_STATUS_CMD, &systemStatus, sizeof(SystemStatusStruct));
  if (!piCommsError) {
    systemStatus.errorField = 0;  //Zero error field as PI has logged it and no error
  } else {
    systemStatus.errorField = piCommsError;  //Zero error field as PI has logged it and no error
  }
  return piCommsError;
}

int8_t sendObstaclesCmd(const ObstaclesCmd &cmd) {
  return uartSendCommand(SENDING_OBSTACLES_CMD, &cmd, sizeof(ObstaclesCmd));
}

int8_t sendObstacleData(const ObstacleData &ob) {
  return uartSendCommand(NEXT_OBSTACLE_CMD, &ob, sizeof(ObstacleData));
}

int8_t sendObstacles(uint16_t heading, uint8_t numObjects, Arc *arcp) {
  obstaclesCmd.currentCompassDirn = heading;
  obstaclesCmd.numOfObstaclesToSend = numObjects;

  // Send header
  sendObstaclesCmd(obstaclesCmd);
  if (piCommsError) {
    systemStatus.errorField = piCommsError;  //Log error in the status byte
    return piCommsError;
  }

  // Send each obstacle
  uint8_t i = 0;
  for (; i < numObjects && !piCommsError; i++) {
    obstacle.obstacleNo = i;
    obstacle.bearing = normalise(heading + (SERVO_CENTRE - arcp->centreDirection));
    obstacle.width = arcp->width;
    obstacle.avgDistance = arcp->avgDistance;
    sendObstacleData(obstacle);
    arcp++;
  }
  if (piCommsError) {
    systemStatus.errorField = piCommsError;  //Log error in the status byte
    D_print("Failed to send all obs - sent: ");
    D_println(i);
  }
  return piCommsError;
}

