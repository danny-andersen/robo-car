#define LOOP_TIME 100
#define NUMBER_OF_ANGLES_IN_SWEEP 180  //Number of samples to take in a sweep
#define MAX_NUMBER_OF_OBJECTS_IN_SWEEP 20
#define MIN_DISTANCE_TO_MOVE 20  //Dont get any closer than this
#define MIN_DISTANCE_AHEAD 40  //Do point moving ahead if this is the furthest object
#define MIN_DISTANCE_TO_TURN 10  //If we have less than this around then we can't rotate

enum Robot_State {
  INIT,
  INIT_FAILED,
  DRIVE,
  ROTATING,
  SWEEP,
  UTURN_SWEEP,
  BACK_OUT,
  OFF_GROUND,
  ROTATING_LEFT,
  ROTATING_RIGHT,
  ROTATING_LEFT_BLOCKED_RIGHT, //Tried rotating right but blocked, so going left
  ROTATING_RIGHT_BLOCKED_LEFT, //Tried rotating left but blocked, so going right
  ROTATING_FRONT_BLOCKED_BACKING_OUT, //Front obstruction but clear at back, so backup a bit and then rotate (if clear)
  ROTATING_REAR_BLOCKED_GO_FORWARD, //Rear obstruction but clear at front, so go forward a bit and then rotate (if clear)
};

enum Drive_State {
  STOPPED,
  TURNING,
  DRIVE_FORWARD,
  REVERSING
};

enum SWEEP_STATUS {
  CLEAR_TO_DRIVE, //We have a direction to drive
  BLOCKED_AHEAD, //No clear route ahead but have room to turn around
  CANNOT_TURN,  //Need to reverse straight back and re-sweep
};

enum Error_State {
  NO_ERROR, //0 = no current error
  I2C_DATA_TOO_LONG, //Note: first 5 fields match I2C error nums
  I2C_ADDR_NACK,
  I2C_DATA_NACK,
  I2C_ERROR,
  I2C_TIMEOUT, //I2C bus timeout
  I2C_PI_DATA_TOO_LONG, //Note: first 5 fields match I2C error nums
  I2C_PI_ADDR_NACK,
  I2C_PI_DATA_NACK,
  I2C_PI_ERROR,
  I2C_PI_TIMEOUT, //I2C bus timeout
  I2C_RX_TIMEOUT,
  I2C_PI_CRC_ERROR,
  I2C_NANO_CRC_ERROR,
  I2C_EXTRA_BYTES, //Received extra bytes when not expecting them (in flush())
  I2C_PI_RETRIED,
  I2C_NANO_RETRIED,
};