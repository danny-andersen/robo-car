#define CMPS12_ADDRESS 0x60
#define COMPASS_8 0x01    // Register to read 8bit compass from
#define COMPASS 0x02      // Register to read 16bit compass from
#define PITCH 0x04        // Register to read pitch from
#define ACCEL 0x0C        // Register to read raw acceleration from
#define GYRO 0x12         // Register to read raw gyro from
#define CALIBRATION 0x12  // Register to read calibration level

uint8_t getCalibrationStatus() {
  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(CALIBRATION);                 //Sends the register we wish to start reading from
  Wire.endTransmission();
  Wire.requestFrom(CMPS12_ADDRESS, 1);
  while (Wire.available() < 1)
    ;  // Wait for all bytes to come back
  return Wire.read();
}

bool compass_init() {
  //Check that device is on the bus
  bool retStatus = false;
  Wire.begin();
  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(0x00);                        //Read the software version
  uint8_t status = Wire.endTransmission();
  if (status) {
    //Failed
    if (Serial) {
      Serial.print("Compass not present - write returned: ");
      Serial.println(status);
    }
  } else {
      retStatus = true;
  }
  return retStatus;
}

void waitUntilCalibrated() {
  //Wait until calibrated
  uint16_t calreads = 0;
  uint8_t calibration = 0;
  do {
    delay(10);
    calibration = getCalibrationStatus();
    calreads++;
  } while ((calibration & 0xC0) != 0xC0);

  Serial.print("Calibration: ");  // Display 16 bit angle with decimal place
  Serial.print(calibration);
  Serial.print(" reads: ");
  Serial.print(calreads);
  Serial.print(" : ");
  Serial.println(calibration, BIN);
}

uint8_t getVersion() {
  // Read the first byte
  Wire.begin();
  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(0x00);                        //Read the software version
  uint8_t status = Wire.endTransmission();
  Wire.requestFrom(CMPS12_ADDRESS, 1);
  while (Wire.available() < 1)
    ;
  uint8_t vers = Wire.read();
  return vers;
}

int16_t readBearing() {
  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(COMPASS);                     //Sends the register we wish to start reading from
  Wire.endTransmission();

  // Request 2 bytes from the CMPS12
  // both bytes of the 16 bit bearing
  Wire.requestFrom(CMPS12_ADDRESS, 2);

  while (Wire.available() < 2)
    ;  // Wait for all bytes to come back

  uint8_t high_byte = Wire.read();
  uint8_t low_byte = Wire.read();

  int16_t compass = high_byte;  // Calculate 16 bit angle
  compass <<= 8;
  compass += low_byte;

  compass -= 900;  //Compass is mounted 90 rotated
  if (compass < 0) {
    compass += 3600;
  }

  return compass;
}

void readAttitude(int8_t *pitch, int8_t *roll) {
  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(PITCH);                       //Sends the register we wish to start reading from
  Wire.endTransmission();

  // Request 2 bytes from the CMPS12
  // pitch and roll
  Wire.requestFrom(CMPS12_ADDRESS, 2);

  while (Wire.available() < 2)
    ;  // Wait for all bytes to come back

  *pitch = Wire.read();
  *roll = Wire.read();
}

void readBearingAndAttitude(int16_t *compass, int8_t *pitch, int8_t *roll) {
  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(COMPASS);                     //Sends the register we wish to start reading from
  Wire.endTransmission();

  // Request 4 bytes from the CMPS12
  // both bytes of the 16 bit bearing, pitch and roll
  Wire.requestFrom(CMPS12_ADDRESS, 4);

  while (Wire.available() < 4)
    ;  // Wait for all bytes to come back

  uint8_t high_byte = Wire.read();
  uint8_t low_byte = Wire.read();
  *pitch = Wire.read();
  *roll = Wire.read();

  *compass = high_byte;  // Calculate 16 bit angle
  *compass <<= 8;
  *compass += low_byte;

  *compass -= 900;  //Compass is mounted 90 rotated
  if (*compass < 0) {
    *compass += 3600;
  }
}
