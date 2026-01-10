#include <Wire.h>

#define CMPS12_ADDRESS 0x60
#define ANGLE_8 1         // Register to read 8bit angle from
#define COMPASS 2         // Register to read 16bit compass from
#define ACCEL 0x0C        // Register to read raw acceleration from
#define GYRO 0x12         // Register to read raw gyro from
#define CALIBRATION 0x12  // Register to read calibration level

unsigned char high_byte, low_byte, angle8;
int8_t pitch, roll;
int16_t compass;
int16_t gyrox, gyroy, gyroz, temp;
int16_t accelX, accelY, accelZ;
uint8_t calibration;

void setup() {
  Serial.begin(9600);  // Start serial port
  Serial.println("CMPS12 test - check version");
  Wire.begin();
  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(0x00);                        //Read the software version
  Wire.endTransmission();
  Wire.requestFrom(CMPS12_ADDRESS, 1);
  while (Wire.available() < 1)
    ;
  uint8_t vers = Wire.read();
  Serial.print("CMPS12 Version: ");
  Serial.println(vers);
}

void loop() {

  //Wait until calibrated
  uint16_t calreads = 0;
  do {
    Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
    Wire.write(CALIBRATION);                 //Sends the register we wish to start reading from
    Wire.endTransmission();
    Wire.requestFrom(CMPS12_ADDRESS, 1);
    while (Wire.available() < 1)
      ;  // Wait for all bytes to come back
    calibration = Wire.read();
    delay(10);
    calreads++;
  } while ((calibration & 0xC0) != 0xC0);
  Serial.print("    Calibration: ");  // Display 16 bit angle with decimal place
  Serial.print(calibration);
  Serial.print(" reads: ");
  Serial.print(calreads);
  Serial.print(" : ");
  Serial.print(calibration, BIN);

  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(COMPASS);                     //Sends the register we wish to start reading from
  Wire.endTransmission();

  // Request 5 bytes from the CMPS12
  // this will give us the 8 bit bearing,
  // both bytes of the 16 bit bearing, pitch and roll
  Wire.requestFrom(CMPS12_ADDRESS, 4);

  while (Wire.available() < 4)
    ;  // Wait for all bytes to come back

  high_byte = Wire.read();
  low_byte = Wire.read();
  pitch = Wire.read();
  roll = Wire.read();

  compass = high_byte;  // Calculate 16 bit angle
  compass <<= 8;
  compass += low_byte;

  Serial.print("  Roll: ");  // Display roll data
  Serial.print(roll, DEC);

  Serial.print("    Pitch: ");  // Display pitch data
  Serial.print(pitch, DEC);

  compass -= 900; //Compass is mounted 90 rotated
  if (compass < 0) {
    compass += 3600;
  }

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

  delay(250);  // Short delay before next loop
}
