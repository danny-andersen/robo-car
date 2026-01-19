#include <avr/wdt.h>
#include <Wire.h>

#include "cmps-12.h"
#include "inter-i2c.h"
#include "robo-car.h"

bool leftGround() {
  return false;
}
#include "motor-driver.h"
#include "movement.h"


unsigned char high_byte, low_byte, angle8;
int8_t pitch, roll;
int16_t compass;
int16_t gyrox, gyroy, gyroz, temp;
int16_t accelX, accelY, accelZ;

void setup() {
  Serial.begin(9600);  // Start serial port
  delay(3000);
  bool compassReady = compass_init();
  if (Serial) {
    Serial.print("Compass ready: ");
    Serial.println(compassReady);
    Serial.print("CMPS12 Version: ");
    Serial.println(getVersion());
  }
  waitUntilCalibrated();
  Serial.print("Calibration status = ");
  Serial.println(getCalibrationStatus(), HEX);
  delay(5000);
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

  readBearingAndAttitude(&compass, &pitch, &roll);
  Serial.print("  Roll: ");  // Display roll data
  Serial.print(roll, DEC);
  Serial.print("    Pitch: ");  // Display pitch data
  Serial.print(pitch, DEC);
  Serial.print("    Bearing: ");  // Display 16 bit angle with decimal place
  Serial.print(compass / 10, DEC);
  Serial.print(".");
  Serial.println(compass % 10, DEC);
  // Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  // Wire.write(ANGLE_8);                     //Sends the register we wish to start reading from
  // Wire.endTransmission();

  // // Request 5 bytes from the CMPS12
  // // this will give us the 8 bit bearing,
  // // both bytes of the 16 bit bearing, pitch and roll
  // Wire.requestFrom(CMPS12_ADDRESS, 5);

  // while(Wire.available() < 5);        // Wait for all bytes to come back

  // angle8 = Wire.read();               // Read back the 5 bytes
  // high_byte = Wire.read();
  // low_byte = Wire.read();
  // pitch = Wire.read();
  // roll = Wire.read();

  // angle16 = high_byte;                 // Calculate 16 bit angle
  // angle16 <<= 8;
  // angle16 += low_byte;

  // Serial.print("roll: ");               // Display roll data
  // Serial.print(roll, DEC);

  // Serial.print("    pitch: ");          // Display pitch data
  // Serial.print(pitch, DEC);

  // Serial.print("    angle full: ");     // Display 16 bit angle with decimal place
  // Serial.print(angle16 / 10, DEC);
  // Serial.print(".");
  // Serial.print(angle16 % 10, DEC);

  // Serial.print("    angle 8: ");        // Display 8bit angle
  // Serial.println(angle8, DEC);

  // Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  // Wire.write(GYRO);                     //Sends the register we wish to start reading from
  // Wire.endTransmission();

  // Wire.requestFrom(CMPS12_ADDRESS, 8);
  // while(Wire.available() < 8);        // Wait for all bytes to come back
  // high_byte = Wire.read();
  // low_byte = Wire.read();
  // gyrox = high_byte;                 // Calculate 16 bit angle
  // gyrox <<= 8;
  // gyrox += low_byte;
  // high_byte = Wire.read();
  // low_byte = Wire.read();
  // gyroy = high_byte;                 // Calculate 16 bit angle
  // gyroy <<= 8;
  // gyroy += low_byte;
  // high_byte = Wire.read();
  // low_byte = Wire.read();
  // gyroz = high_byte;                 // Calculate 16 bit angle
  // gyroz <<= 8;
  // gyroz += low_byte;
  // high_byte = Wire.read();
  // low_byte = Wire.read();
  // temp = high_byte;                 // Calculate 16 bit angle
  // temp <<= 8;
  // temp += low_byte;
  // Serial.print("Gyrox: ");     // Display 16 bit angle with decimal place
  // Serial.print(gyrox);
  // Serial.print("  Gyroy: ");     // Display 16 bit angle with decimal place
  // Serial.print(gyroy);
  // Serial.print("  Gyroz: ");     // Display 16 bit angle with decimal place
  // Serial.print(gyroz);
  // Serial.print("  Temp: ");     // Display 16 bit angle with decimal place
  // Serial.println(temp);

  // Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  // Wire.write(ACCEL);                     //Sends the register we wish to start reading from
  // Wire.endTransmission();

  // Wire.requestFrom(CMPS12_ADDRESS, 6);
  // while(Wire.available() < 6);        // Wait for all bytes to come back
  // high_byte = Wire.read();
  // low_byte = Wire.read();
  // accelX = high_byte;                 // Calculate 16 bit angle
  // accelX <<= 8;
  // accelX += low_byte;
  // high_byte = Wire.read();
  // low_byte = Wire.read();
  // accelY = high_byte;                 // Calculate 16 bit angle
  // accelY <<= 8;
  // accelY += low_byte;
  // high_byte = Wire.read();
  // low_byte = Wire.read();
  // accelZ = high_byte;                 // Calculate 16 bit angle
  // accelZ <<= 8;
  // accelZ += low_byte;
  // Serial.print("accelX: ");     // Display 16 bit angle with decimal place
  // Serial.print(accelX);
  // Serial.print("  accelY: ");     // Display 16 bit angle with decimal place
  // Serial.print(accelY);
  // Serial.print("  accelZ: ");     // Display 16 bit angle with decimal place
  // Serial.println(accelZ);

  delay(500);  // Short delay before next loop
}
