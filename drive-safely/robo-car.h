#ifndef ROBO_CAR_H
#define ROBO_CAR_H

#define DEBUG 0    // SET TO 0 OUT TO REMOVE TRACES

#if DEBUG
#define D_SerialBegin(...) Serial.begin(__VA_ARGS__)
#define D_print(...)    Serial.print(__VA_ARGS__)
#define D_write(...)    Serial.write(__VA_ARGS__)
#define D_println(...)  Serial.println(__VA_ARGS__)
#else
#define D_SerialBegin(...)
#define D_print(...)
#define D_write(...)
#define D_println(...)
#endif

#define LOOP_TIME 100
#define NUMBER_OF_ANGLES_IN_SWEEP 180  //Number of samples to take in a sweep
#define MAX_NUMBER_OF_OBJECTS_IN_SWEEP 12
#define MIN_DISTANCE_TO_MOVE 20  //Dont get any closer than this
#define MIN_DISTANCE_AHEAD 40  //Do point moving ahead if this is the furthest object
#define MIN_DISTANCE_TO_TURN 10  //If we have less than this around then we can't rotate
#define BACKOUT_TIME 1000 //Ms to backout of trouble

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
  I2C_RX_TIMEOUT,
  I2C_NANO_CRC_ERROR,
  I2C_NANO_RETRIED,
  I2C_EXTRA_BYTES, //Received extra bytes when not expecting them (in flush())
  PI_DATA_LEN_ERROR, 
  PI_SEQ_ERROR,
  PI_MSG_TYPE_ERROR,
  PI_HEADER_LEN_ERR,
  PI_ERROR,
  PI_TIMEOUT, //Msg timeout
  PI_CRC_ERROR,
  PI_RETRIED,
};


struct ObstacleData {
  uint8_t obstacleNo;
  uint16_t bearing;      //Compass heading of the centre of the obstacle
  uint16_t width;        //The width in degrees of view of the obstacle in front
  uint16_t avgDistance;  //How far away the obstacle is
};

struct ObstaclesCmd {
  int16_t currentCompassDirn;    //Current compass direction (i.e. straightahead)
  uint8_t numOfObstaclesToSend;  //Number of obstacles found that are to be sent for analysis
};

struct PiStatusStruct {
  uint8_t ready;              //0 == not ready
  uint8_t lidarStatus;        //Bits set according to proximity bits
  uint16_t directionToDrive;  //1000 = direction not set
};

struct SystemStatusStruct {
  unsigned long timestamp;  //This is the timestamp in milliseconds since boot up
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
  uint8_t errorField;  //Any errors (currently comms errors)
};

struct StatusStruct {
  uint8_t proximityState;      //Status of proximity sensors
  uint8_t currentLeftSpeed;    //Current speed of left wheel in cm/s
  uint8_t currentRightSpeed;   //Current speed of right wheel in cm/s
  uint8_t averageSpeed;        //Avg speed of current drive in cm/s
  uint16_t distanceTravelled;  //Distance travelled since motored started in cm (average of left and right)
  uint8_t checksum;
};

// Struct to hold arc info, which represent objects found in the forward field of view
struct Arc {
  uint8_t startIndex;  //0 is 90 right, 180 is 90 left
  uint8_t endIndex;
  uint8_t centreDirection;
  uint8_t width;         //of arc
  uint16_t avgDistance;  //distance
};

StatusStruct nanoStatus;
int8_t nanoCommsError = 1;  //0 means up and on the bus - anything else is an error

int8_t piCommsError = 1;    //0 means up and communicating, any other values are ERROR_STATE errors

ObstaclesCmd obstaclesCmd;
ObstacleData obstacle;
StatusStruct rdstatus;
PiStatusStruct piStatus;
SystemStatusStruct systemStatus;
Robot_State lastRobotState = INIT;

uint16_t distances[NUMBER_OF_ANGLES_IN_SWEEP];  //Gives a step size of 1 deg
Arc arcs[MAX_NUMBER_OF_OBJECTS_IN_SWEEP];       // up to 20 arcs
uint8_t furthestObjectIndex = 0;

#endif