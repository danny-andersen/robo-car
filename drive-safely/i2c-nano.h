#define UNO_PERIPHERAL_ADDR 0x08
#define PI_ADDR 0x09

//Commands
#define REQ_DIRECTION_TO_DRIVE_CMD 0x02  //Requesting the determination of which direction to drive next (based on obstacles in front)
#define SENDING_OBSTACLES_CMD 0x03       //About to start sending obstacles
#define NEXT_OBSTACLE_CMD 0x04           //The next obstacle in the list
#define MOTOR_STARTING_CMD 0x05          //Motor is starting - reset wheel pulse counters
#define MOTOR_STOPPING_CMD 0x06          //Motor is stopping
#define REQ_STATUS_CMD 0x07              //Return proximity status, distance travelled, current speed
#define SEND_SYSTEM_STATUS_CMD 0x08      //Sends system status to PI

//Data definitions
#define FRONT_LEFT_PROX_BIT 0
#define FRONT_RIGHT_PROX_BIT 1
#define REAR_LEFT_PROX_BIT 2
#define REAR_RIGHT_PROX_BIT 3
#define TOP_FRONT_LEFT_PROX_BIT 4
#define TOP_FRONT_RIGHT_PROX_BIT 5
#define FRONT_LEFT_PROX_SET 0x01
#define FRONT_RIGHT_PROX_SET 0x02
#define REAR_LEFT_PROX_SET 0x04
#define REAR_RIGHT_PROX_SET 0x08
#define TOP_FRONT_LEFT_PROX_SET 0x10
#define TOP_FRONT_RIGHT_PROX_SET 0x20
#define REAR_REAR_PROX_SET 0x40 // Extra bit for rear-most proximity (lidar only)
#define FRONT_FRONT_PROX_SET 0x80 // Extra bit for rear-most proximity (lidar only)


#define MAX_NUMBER_OF_OBJECTS_IN_SWEEP 20


struct StatusStruct {
  uint8_t proximityState;      //Status of proximity sensors
  uint8_t currentLeftSpeed;    //Current speed of left wheel in cm/s
  uint8_t currentRightSpeed;   //Current speed of right wheel in cm/s
  uint8_t averageSpeed;        //Avg speed of current drive in cm/s
  uint16_t distanceTravelled;  //Distance travelled since motored started in cm (average of left and right)
  uint8_t checksum;
};

StatusStruct nanoStatus;

uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;        // initial value
    const uint8_t poly = 0x31; // polynomial x^8 + x^5 + x^4 + 1

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ poly;
            else
                crc <<= 1;
        }
    }
    return crc;
}


