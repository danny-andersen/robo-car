#include <Wire.h>
// #include "Arduino_RouterBridge.h"
#include "inter-i2c.h"


#define FRONT_LEFT_PROX_SENSOR 2
#define FRONT_RIGHT_PROX_SENSOR 3
#define REAR_LEFT_PROX_SENSOR 4
#define REAR_RIGHT_PROX_SENSOR 5
#define FRONT_LEFT_WHEEL_SENSOR 6
#define FRONT_RIGHT_WHEEL_SENSOR 7
#define TOP_FRONT_RIGHT_PROX_SENSOR 8
#define TOP_FRONT_LEFT_PROX_SENSOR 9


uint8_t numObstaclesToRx = 0;
uint8_t currentCommand = 0;

ObstacleData obstacles[MAX_NUMBER_OF_OBJECTS_IN_SWEEP];
uint8_t numObstaclesRx = 0;

void setup() {
  // Serial.begin(9600);
  // Serial.println("Starting...");
  // Monitor.begin();
  // Monitor.println("Starting...");
  // Bridge.begin();
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(FRONT_LEFT_PROX_SENSOR, INPUT_PULLUP);
  pinMode(FRONT_RIGHT_PROX_SENSOR, INPUT_PULLUP);
  pinMode(REAR_LEFT_PROX_SENSOR, INPUT_PULLUP);
  pinMode(REAR_RIGHT_PROX_SENSOR, INPUT_PULLUP);

  //Configure as I2C peripheral with address 0x8
  Wire.begin(UNO_PERIPHERAL_ADDR);
  // Monitor.println("I2C bus started...");
  // Serial.println("I2C bus started...");
  Wire.onReceive(command_handler);  //Rx command which tells us what to expect next
  Wire.onRequest(request_handler);
  // Monitor.println("Set up complete, waiting for command");
  // Serial.println("Set up complete, waiting for command");
}

void loop() {
  // Monitor.print("Proximity: ");
  // Monitor.println(readProximitySensors());
  // if (Serial) {
  //   Serial.print("Proximity: ");
  //   Serial.println(readProximitySensors());
  // }
  delay(1000);
}

uint8_t receiveNextByte() {
  //Rx a command from controller uno
  while (Wire.available() == 0) {
    // Serial.println("Waiting to rx next byte");
    delay(10);
    flickerLED();
  }
  return Wire.read();
}

uint8_t readProximitySensors() {
  //Return the state of the proximity sensors
  uint8_t sensors = 0;
  sensors = sensors | !digitalRead(FRONT_LEFT_PROX_SENSOR) << FRONT_LEFT_PROX_BIT;
  sensors = sensors | !digitalRead(FRONT_RIGHT_PROX_SENSOR) << FRONT_RIGHT_PROX_BIT;
  sensors = sensors | !digitalRead(REAR_LEFT_PROX_SENSOR) << REAR_LEFT_PROX_BIT;
  sensors = sensors | !digitalRead(REAR_RIGHT_PROX_SENSOR) << REAR_RIGHT_PROX_BIT;
  return sensors;
}

void request_handler() {
  // Serial.print("Received Request for data, current command: ");
  // Serial.println(currentCommand);
  switch (currentCommand) {
    case REQ_PROXIMITY_STATE_CMD:
      handleProximityRequest();
      break;
    case REQ_DIRECTION_TO_DRIVE_CMD:
      handleDirectionRequest();
      break;
    default:
      if (Serial) {
        Serial.print("Unexpected request state: ");
        Serial.println(currentCommand);
      }
      break;
      //     if (Serial) {
      //       Serial.print("Rx request event but command state not recognised: ");
      //       Serial.println(currentCommand);
      //     }
  }
  //Reset command state
  currentCommand = 0;
  flickerLED();

}

void handleProximityRequest() {
  //Return single byte digital state
  uint8_t digitalState = readProximitySensors();
  // if (Serial) {
  //   Serial.print("Sending Proximity state: ");
  //   Serial.println(digitalState);
  // }
  Wire.write(digitalState);
}

void handleDirectionRequest() {
  //Return compass direction to drive
  direction.directionToDrive = obstaclesCmd.currentCompassDirn + obstacles[10].relDirection;
  // if (Serial) {
  //   Serial.println("Rx Request for REQ_DIRECTION_TO_DRIVE_CMD");
  //   Serial.print("Sending direction: ");
  //   Serial.println(direction.directionToDrive);
  // }

  Wire.write((byte *)&direction, sizeof(direction));
}


void handleObstaclesCmd() {
  while (Wire.available() < sizeof(obstaclesCmd)) {
    // Serial.println("Waiting for rest of obstacle cmd");
    delay(10);
  }
  Wire.readBytes((byte *)&obstaclesCmd, sizeof(obstaclesCmd));
  // if (Serial) {
  //   Serial.print("Rx SENDING_OBSTACLES_CMD, total: ");
  //   Serial.print(obstaclesCmd.numOfObstaclesToSend);
  //   Serial.print(" Compass Dirn: ");
  //   Serial.println(obstaclesCmd.currentCompassDirn);
  // }
  numObstaclesRx = 0;  //Reset number received
  for (int i = 0; i < MAX_NUMBER_OF_OBJECTS_IN_SWEEP; i++) {
    obstacles[i].avgDistance = 0;  //Reset any previous obstacles to 0
  }
  // Bridge.notify("resetObstacles".obstaclesCmd.currentCompassDirn);
}

void handleNextObstacle() {
  Wire.readBytes((byte *)&obstacles[numObstaclesRx], sizeof(obstacles[numObstaclesRx]));
  // if (Serial) {
  //   Serial.print("Rx NEXT_OBSTACLE_CMD, number: ");
  //   Serial.print(numObstaclesRx);
  //   Serial.print(" Rel dirn: ");
  //   Serial.print(obstacles[numObstaclesRx].relDirection);
  //   Serial.print(" Width: ");
  //   Serial.print(obstacles[numObstaclesRx].width);
  //   Serial.print(" Distance: ");
  //   Serial.print(obstacles[numObstaclesRx].avgDistance);
  // }
  // Bridge.notify("obstacleInFront", )

  numObstaclesRx++;
}

void flickerLED()
{
  digitalWrite(LED_BUILTIN, HIGH);
  delay(10);
  digitalWrite(LED_BUILTIN, LOW);
}

void command_handler(int numRx) {
  //Rx a command from controller uno
  currentCommand = receiveNextByte();  // Command from controller
  flickerLED();
  // Monitor.print("Rx command: ");
  // Monitor.println(currentCommand);
  // if (Serial) {
  //   Serial.print("Rx command: ");
  //   Serial.println(currentCommand);
  // }
  switch (currentCommand) {
    case REQ_PROXIMITY_STATE_CMD:
      //Return digital state
      // Serial.println("Rx REQ_PROXIMITY_STATE_CMD");
      break;
    case SENDING_OBSTACLES_CMD:
      //Will receive multiple obstacles
      handleObstaclesCmd();
      break;
    case NEXT_OBSTACLE_CMD:
      handleNextObstacle();
      break;
    case REQ_DIRECTION_TO_DRIVE_CMD:
      // Serial.println("Rx REQ_DIRECTION_TO_DRIVE_CMD");
      break;
    default:
      // if (Serial) {
      //   Serial.print("Unrecognised command: ");
      //   Serial.println(currentCommand);
      // }
      currentCommand = 0;
  }
}