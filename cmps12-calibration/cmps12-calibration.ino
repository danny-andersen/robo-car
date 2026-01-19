#include <avr/wdt.h>
#include <Wire.h>

#include "cmps-12.h"

int8_t pitch, roll;
int16_t compass;
uint8_t calib_status = 0;

void setup() {
  Serial.begin(9600);  // Start serial port
  bool compassReady = compass_init();
  if (Serial) {
    Serial.print("Compass ready: ");
    Serial.println(compassReady);
    Serial.print("CMPS12 Version: ");
    Serial.println(getVersion());
  }
  waitUntilCalibrated();
  Serial.print("Calibration status = ");
  Serial.println(getCalibrationStatus());
  do {
    Serial.println("Rotate the compass in a figured of 8 for 30 seconds");
    delay(30000);
    Serial.print("Calibration status = ");
    waitUntilCalibrated();
    calib_status = getCalibrationStatus();
    Serial.println(calib_status, HEX);
  } while (calib_status != 0xFF);
  Serial.println("Saving calibration in 3 secs..");
  delay(3000);
  Wire.begin();
  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(0xF0);                        //Start storing calibration
  uint8_t status = Wire.endTransmission();
  if (status) {
    //Failed
    if (Serial) {
      Serial.print("Compass write 0xFO failed - write returned: ");
      Serial.println(status);
    }
  }
  delay(20);
  Wire.begin();
  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(0xF5);                        //Start storing calibration
  status = Wire.endTransmission();
  if (status) {
    //Failed
    if (Serial) {
      Serial.print("Compass write 0xF5 failed - write returned: ");
      Serial.println(status);
    }
  }
  delay(20);
  Wire.begin();
  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(0xF6);                        //Start storing calibration
  status = Wire.endTransmission();
  if (status) {
    //Failed
    if (Serial) {
      Serial.print("Compass write 0xF6 failed - write returned: ");
      Serial.println(status);
    }
  }

  Serial.print("Saved: calibration status = ");
  Serial.println(getCalibrationStatus());
}

void loop() {
  readAttitude(&pitch, &roll);
  Serial.print("  Roll: ");  // Display roll data
  Serial.print(roll, DEC);
  Serial.print("    Pitch: ");  // Display pitch data
  Serial.print(pitch, DEC);

  compass = readBearing();
  Serial.print("    Bearing: ");  // Display 16 bit angle with decimal place
  Serial.print(compass / 10, DEC);
  Serial.print(".");
  Serial.println(compass % 10, DEC);

  delay(500);  // Short delay before next loop
}
