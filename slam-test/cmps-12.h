#ifndef CMPS_12_H
#define CMPS_12_H


//Functions for the CMPS-12 compass on an IC2 bus

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
    // if (Serial) {
    //   Serial.print(F("Compass not present - write returned: "));
    //   Serial.println(status);
    // }
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

  // Serial.print("Calibration: ");  // Display 16 bit angle with decimal place
  // Serial.print(calibration);
  // Serial.print(" reads: ");
  // Serial.print(calreads);
  // Serial.print(" : ");
  // Serial.println(calibration, BIN);
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

int16_t getCompassBearing() {
  return (readBearing() / 10);
}

float getCompassBearingRads() {
  int16_t bearing = readBearing();
  return (bearing / 10.0) * (M_PI / 180.0);
}

// Normalise direction in the range 0-360
// Which is what the accelerometer returns
int16_t normalise(int16_t dirn) {
  //Convert so from 0 - 360
  dirn = dirn % 360;
  if (dirn < 0) {
    dirn += 360;
  }
  return dirn;
}

//Normalise direction to 0 - 2*pi
float normalise_rad(float dirn) {
  dirn = fmod(dirn, TWO_PI);
  return dirn;
}

//Calculate the difference between two compass angles
int16_t compass_diff(int16_t a, int16_t b) {
  int16_t diff = a - b;

  // Wrap into 0–359 range
  diff %= 360;
  if (diff < 0)
    diff += 360;

  // Convert to minimal 0–180 difference
  if (diff > 180)
    diff = 360 - diff;

  return diff;
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

  //Although the first value is actually the pitch register, the way the device is mounted this is actually roll
  *roll = Wire.read();
  *pitch = Wire.read();
  //Allow for mounting error
  *pitch -= 5;
}

bool rollingOrPitching() {
  uint8_t currentRoll = 0;
  uint8_t currentPitch = 0;
  readAttitude(&currentPitch, &currentRoll);
  // if (Serial && (abs(currentRoll) >= 10 || abs(currentPitch) >= 10)) {
  //   Serial.print("Rolling: ");
  //   Serial.print(currentRoll);
  //   Serial.print(" pitch: ");
  //   Serial.println(currentPitch);
  // }
  return (abs(currentRoll) >= 10 || abs(currentPitch) >= 10);
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
  //Although the first value is actually the pitch register, the way the device is mounted this is actually roll
  *roll = Wire.read();
  *pitch = Wire.read();
  //Allow for mounting error
  *pitch -= 5;

  *compass = high_byte;  // Calculate 16 bit angle
  *compass <<= 8;
  *compass += low_byte;

  *compass -= 900;  //Compass is mounted 90 rotated
  if (*compass < 0) {
    *compass += 3600;
  }
}

#endif