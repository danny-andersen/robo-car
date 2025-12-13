#define UNO_PERIPHERAL_ADDR 0X08

//Commands
#define REQ_PROXIMITY_STATE_CMD 0x01  //Requesting the state of the proximity sensors attached to the uno
#define REQ_DIRECTION_TO_DRIVE_CMD 0x02 //Requesting the determination of which direction to drive next (based on obstacles in front)
#define SENDING_OBSTACLES_CMD 0x03 //About to start sending obstacles
#define NEXT_OBSTACLE_CMD 0x04 //The next obstacle in the list

//Data definitions
#define FRONT_LEFT_PROX_BIT 0
#define FRONT_RIGHT_PROX_BIT 1
#define REAR_LEFT_PROX_BIT 2
#define REAR_RIGHT_PROX_BIT 3

#define MAX_NUMBER_OF_OBJECTS_IN_SWEEP 20

struct DirectionData {
  uint16_t directionToDrive;
};

struct ObstacleData {
  int8_t relDirection;  //Direction relative to the current compass heading of the centre of the obstacle
  uint8_t width; //The width in degrees of view of the obstacle in front
  uint16_t avgDistance; //How far away the obstacle is 
};

struct ObstaclesCmd {
  uint16_t currentCompassDirn; //Current compass direction (i.e. straightahead)
  uint8_t numOfObstaclesToSend; //Number of obstacles found that are to be sent for analysis
};

