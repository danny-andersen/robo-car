#include <Wire.h>
// #include "Arduino_RouterBridge.h"
#include "i2c-nano.h"


#define FRONT_LEFT_PROX_SENSOR 6
#define FRONT_RIGHT_PROX_SENSOR 7
#define REAR_LEFT_PROX_SENSOR 5
#define REAR_RIGHT_PROX_SENSOR 4
#define FRONT_LEFT_WHEEL_SENSOR 3
#define FRONT_RIGHT_WHEEL_SENSOR 2
#define TOP_FRONT_RIGHT_PROX_SENSOR 8
#define TOP_FRONT_LEFT_PROX_SENSOR 9

#define WHEEL_DIAMETER 6.5
#define WHEEL_CIRCUMFERENCE 20.42
#define PULSE_COUNT_PER_ROTATION 40
#define DISTANCE_PER_PULSE WHEEL_CIRCUMFERENCE / PULSE_COUNT_PER_ROTATION
#define LOOP_DELAY 500
#define PULSES_PER_SECOND_SCALE 1000 / LOOP_DELAY
#define PULSE_COUNT_DISTANCE_PER_LOOP DISTANCE_PER_PULSE *PULSES_PER_SECOND_SCALE

uint8_t currentCommand = 0;
bool moving = false;

volatile unsigned long pulseCounterRight = 0;
volatile unsigned long pulseCounterLeft = 0;

unsigned long lastPulseCountLeft = 0;
unsigned long lastPulseCountRight = 0;
unsigned long movingTime = 0;
float distanceTravelled = 0.0;
float speed;

void setup() {
  // Serial.begin(9600);
  // Serial.println("Starting...");
  // Monitor.begin();
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(FRONT_LEFT_PROX_SENSOR, INPUT_PULLUP);
  pinMode(FRONT_RIGHT_PROX_SENSOR, INPUT_PULLUP);
  pinMode(REAR_LEFT_PROX_SENSOR, INPUT_PULLUP);
  pinMode(REAR_RIGHT_PROX_SENSOR, INPUT_PULLUP);
  pinMode(TOP_FRONT_RIGHT_PROX_SENSOR, INPUT_PULLUP);
  pinMode(TOP_FRONT_LEFT_PROX_SENSOR, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(FRONT_LEFT_WHEEL_SENSOR), pulseLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FRONT_RIGHT_WHEEL_SENSOR), pulseRight, CHANGE);

  resetStats();
  //Configure as I2C peripheral with address 0x8
  Wire.begin(UNO_PERIPHERAL_ADDR);
  // Serial.println("I2C bus started...");
  Wire.onReceive(command_handler);  //Rx command which tells us what to expect next
  Wire.onRequest(request_handler);
  // Serial.println("Set up complete, waiting for command");
}

void loop() {
  lastPulseCountLeft = pulseCounterLeft;
  lastPulseCountRight = pulseCounterRight;
  delay(LOOP_DELAY);
  if (moving) {
    movingTime += LOOP_DELAY;
    if (movingTime > LOOP_DELAY) {
      //Skip first loop
      speed = (pulseCounterLeft - lastPulseCountLeft) * 10.0 * PULSE_COUNT_DISTANCE_PER_LOOP;
      nanoStatus.currentLeftSpeed = int(speed / 10 + 0.5);
      speed = (pulseCounterRight - lastPulseCountRight) * 10.0 * PULSE_COUNT_DISTANCE_PER_LOOP;
      nanoStatus.currentRightSpeed = int(speed / 10 + 0.5);
      distanceTravelled = (pulseCounterRight + pulseCounterLeft) * DISTANCE_PER_PULSE / 2;
      nanoStatus.distanceTravelled = int(distanceTravelled + 0.5);
      nanoStatus.averageSpeed = int((1000 * distanceTravelled / movingTime) + 0.5);
      // if (Serial) {
      //   // Serial.print("Proximity: ");
      //   // Serial.println(readProximitySensors());
      // Serial.print(" Right Pulse: ");
      // Serial.print(pulseCounterRight);
      // Serial.print(" Speed: ");
      // Serial.print(periStatus.currentRightSpeed);
      // Serial.print(" Last cnt: ");
      // Serial.print(lastPulseCountRight);
      // Serial.print(" Left Pulse: ");
      // Serial.println(pulseCounterLeft);
      // }
    }
  }
}

void pulseRight() {
  pulseCounterRight++;
}

void pulseLeft() {
  pulseCounterLeft++;
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
  sensors = sensors | !digitalRead(TOP_FRONT_RIGHT_PROX_SENSOR) << TOP_FRONT_RIGHT_PROX_BIT;
  sensors = sensors | !digitalRead(TOP_FRONT_LEFT_PROX_SENSOR) << TOP_FRONT_LEFT_PROX_BIT;
  return sensors;
}

void returnStatus() {
  //Note: status is updated in each loop
  nanoStatus.proximityState = readProximitySensors();
  nanoStatus.checksum = nanoStatus.checksum = crc8((uint8_t*)&nanoStatus, sizeof(StatusStruct) - 1);

  Wire.write((byte *)&nanoStatus, sizeof(nanoStatus));
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

void flickerLED() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(10);
  digitalWrite(LED_BUILTIN, LOW);
}

void resetStats() {
  lastPulseCountLeft = 0;
  lastPulseCountRight = 0;
  pulseCounterRight = 0;
  pulseCounterLeft = 0;
  movingTime = 0;
  distanceTravelled = 0.0;
  moving = true;
  nanoStatus.distanceTravelled = 0;
  nanoStatus.averageSpeed = 0;
  nanoStatus.currentLeftSpeed = 0;
  nanoStatus.currentRightSpeed = 0;
}

void request_handler() {
  // Serial.print("Received Request for data, current command: ");
  // Serial.println(currentCommand);
  switch (currentCommand) {
    case MOTOR_STARTING_CMD:
    case MOTOR_STOPPING_CMD:
    case REQ_STATUS_CMD:
      //These commands expect a status to be returned
      returnStatus();
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
    case MOTOR_STARTING_CMD:
      resetStats();
      break;
    case MOTOR_STOPPING_CMD:
      moving = false;
      break;
    case REQ_STATUS_CMD:
      //Do nothing - next message will be a request for status
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