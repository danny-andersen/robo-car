// #include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

int16_t gz;
unsigned long now, lastTime = 0;
float dt;
float agz = 0;
/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

int const INTERRUPT_PIN = 2;  // Define the interruption #0 pin
bool blinkState;

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;   // Set true if DMP init was successful
uint8_t MPUIntStatus;    // Holds actual interrupt status byte from MPU
uint8_t devStatus;       // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64];  // FIFO storage buffer

/*---Orientation/Motion Variables---*/
Quaternion q;         // [w, x, y, z]         Quaternion container
VectorInt16 aa;       // [x, y, z]            Accel sensor measurements
VectorInt16 gy;       // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;   // [x, y, z]            Gravity-free accel sensor measurements
VectorFloat gravity;  // [x, y, z]            Gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
int16_t eulerDeg[3];    // Euler in degrees
float ypr[3];         // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

bool accelerometer_init() {
  // Initialize the accelerometer hardware
  // e.g., set up I2C communication, configure registers, etc.
  Wire.begin();
  /* Initializate and configure the DMP*/
  if (Serial) Serial.println("Initialising DMP...");
  devStatus = mpu.dmpInitialize();
  if (mpu.testConnection() == false) {
    if (Serial) Serial.println("MPU6050 connection failed");
    while (true)
      ;
  }

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  /* Making sure it worked (returns 0 if so) */
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration generate offsets and calibrates the MPU6050
    mpu.CalibrateGyro(6);
    // Serial.println("These are the Active offsets: ");
    // mpu.PrintActiveOffsets();
    // Serial.println(F("Enabling DMP..."));  //Turning ON DMP
    mpu.setDMPEnabled(true);
    return true;  // Return TRUE if initialization is successful
  } else {
    if (Serial) {
      Serial.print(F("DMP Initialisation failed (code "));  //Print the error code
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
    return false;
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }
}

const float minmax = M_PI - 0.175;

// Make sure direction is in the range -180 to +180
// Which is what the accelerometer returns
int16_t normalise(int16_t dirn) {
  if (dirn > 360) {
    //Convert so from 0 - 360
    dirn = dirn % 360;
  }
  if (dirn > 180) {
    //Flipped over to negative
    dirn -= 360;
  }
  if (dirn < -180) {
    //Flipped over to positive
    dirn += 360;
  }
  // 180 cant be achieved on accelerometer
  // Shift into (-179, 179]
  if (dirn > 179)
    dirn = 179;
  else if (dirn < -179)
    dirn = -179;
  return dirn;
}

float normalise_rad(float dirn) {
  dirn = fmod(dirn, M_PI);  // reduce to [0, π) or (-π, π)
  if (dirn > minmax)
    dirn = minmax;
  else if (dirn <= -minmax)
    dirn = -minmax;
  return dirn;
}

bool getAccelerometerEuler() {
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    // mpu.dmpGetGravity(&gravity, &q);
    // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetEuler(euler, &q);
    //Convert to degrees -> +90 is 90degrees to the right, -90 is 90 deg to the left
    eulerDeg[0] = int((euler[0] * 180 / M_PI) + 0.5);
    eulerDeg[1] = int((euler[1] * 180 / M_PI) + 0.5);
    eulerDeg[2] = int((euler[2] * 180 / M_PI) + 0.5);

    return true;
  } else {
    return false;
  }
}

bool rollingOrPitching() {
  float currentRoll = euler[1];
  float currentPitch = euler[2];

  return (abs(currentRoll) >= 5 || abs(currentPitch) >= 5);
}

bool getAccelerometerAllReading() {
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetEuler(euler, &q);
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    // mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    return true;
  } else {
    return false;
  }
}