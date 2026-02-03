import struct


# -----------------------------
# I2C slave address
# -----------------------------
I2C_ADDR = 0x09


# -----------------------------
# Commands
# -----------------------------
REQ_PROXIMITY_STATE_CMD = 0x01
REQ_DIRECTION_TO_DRIVE_CMD = 0x02
SENDING_OBSTACLES_CMD = 0x03
NEXT_OBSTACLE_CMD = 0x04
MOTOR_STARTING_CMD = 0x05
MOTOR_STOPPING_CMD = 0x06
REQ_STATUS_CMD = 0x07
SEND_SYSTEM_STATUS_CMD = 0x08


# Proximity states (bitmask)
LIDAR_PROXIMITY_NONE = 0x00
FRONT_LEFT_PROX_BIT = 0
FRONT_RIGHT_PROX_BIT = 1
REAR_LEFT_PROX_BIT = 2
REAR_RIGHT_PROX_BIT = 3
FRONT_LEFT_PROX_SET = 0x01
FRONT_RIGHT_PROX_SET = 0x02
REAR_LEFT_PROX_SET = 0x04
REAR_RIGHT_PROX_SET = 0x08
TOP_FRONT_LEFT_PROX_SET = 0x10
TOP_FRONT_RIGHT_PROX_SET = 0x20
REAR_REAR_PROX_SET = 0x40  # Extra bit for rear-most proximity (lidar only)
FRONT_FRONT_PROX_SET = 0x80  # Extra bit for rear-most proximity (lidar only)


# Proximity thresholds (mm)
PROXIMITY_THRESHOLD_FRONT_MM = 200  # Anything closer than this is an obstacle directly in front
PROXIMITY_THRESHOLD_FRONT_SIDE_START_MM = 160
PROXIMITY_THRESHOLD_FRONT_SIDE_END_MM = 180
PROXIMITY_ANGLE_FRONT_LEFT_START = 320  # Angle (degrees) to left of front for proximity checking
PROXIMITY_ANGLE_FRONT_LEFT_END = 340
PROXIMITY_ANGLE_FRONT_RIGHT_START = 20  # Angle (degrees) to right of front for proximity checking
PROXIMITY_ANGLE_FRONT_RIGHT_END = 40
PROXIMITY_FRONT_SIDE_SCALE = (
    (PROXIMITY_THRESHOLD_FRONT_SIDE_END_MM - PROXIMITY_THRESHOLD_FRONT_SIDE_START_MM)
    / (PROXIMITY_ANGLE_FRONT_RIGHT_END - PROXIMITY_ANGLE_FRONT_RIGHT_START)
)

PROXIMITY_THRESHOLD_REAR_MM = 60  # Anything closer than this is an obstacle directly on the rear
PROXIMITY_THRESHOLD_REAR_SIDE_MM = 95  # Anything closer than this is an obstacle on the rear side
PROXIMITY_ANGLE_REAR_LEFT_START = 230  # Angle (degrees) limit on left side
PROXIMITY_ANGLE_REAR_LEFT_END = 260
PROXIMITY_ANGLE_REAR_RIGHT_START = 100  # Angle (degrees) on right side
PROXIMITY_ANGLE_REAR_RIGHT_END = 130


# -----------------------------
# Struct definitions
# -----------------------------
ObstacleData_struct = struct.Struct("<bHHHB")  # obstacleNum, relDir, width, avgDistance, crc
ObstaclesCmd_struct = struct.Struct("<hBB")  # currentCompassDirn, numToSend, crc
SystemStatusStruct = struct.Struct(
    "<hhHBBhbbBBBBBB"
)  # humidity, tempC, batteryVoltage, robotState, proximitySensors, currentBearing, pitch, roll, rightWheelSpeed, leftWheelSpeed, averageSpeed, distanceTravelled, errorField, CRC
PiStatusStruct = struct.Struct("<BBH")  # systemReady, lidarProximity, directionToDrive


MAX_OBS = 20

pi = None  # To be initialized in i2c_init() in i2c_slave.py


# -----------------------------
# State variables
# -----------------------------
currentCommand = 0
numObstaclesRx = 0

obstaclesCmd = {
    "currentCompassDirn": 0,
    "numOfObstaclesToSend": 0,
}

obstacles = [
    {"bearing": 0, "width": 0, "avgDistance": 0} for _ in range(MAX_OBS)
]


systemStatus = {
    "humidity": 0,
    "tempC": 0,
    "batteryVoltage": 0,
    "robotState": 0,
    "proximitySensors": 0,
    "currentBearing": 0,
    "pitch": 0,
    "roll": 0,
    "rightWheelSpeed": 0,
    "leftWheelSpeed": 0,
    "averageSpeed": 0,
    "distanceTravelled": 0,
    "errorField": 0,
}

piStatus = {
    "systemReady": 0,
    "lidarProximity": 0,  # LIDAR proximity state: bits are set as per LIDAR_PROXIMITY_xxx constants
    "directionToDrive": 1000,  # Value indicating no direction set
}

ROBOT_STATE_NAMES = [
    "INIT",
    "INIT_FAILED",
    "DRIVE",
    "ROTATING",
    "SWEEP",
    "UTURN_SWEEP",
    "BACK_OUT",
    "OFF_GROUND",
    "ROTATING_LEFT",
    "ROTATING_RIGHT",
    "ROTATING_LEFT_BLOCKED_RIGHT",  # Tried rotating right but blocked, so going left
    "ROTATING_RIGHT_BLOCKED_LEFT",  # Tried rotating left but blocked, so going right
    "ROTATING_FRONT_BLOCKED_BACKING_OUT",
    "ROTATING_REAR_BLOCKED_GO_FORWARD",
]

ERROR_FIELD_NAMES = [
  "NO_ERROR",
  "I2C_DATA_TOO_LONG", #Note: first 5 fields match I2C error nums
  "I2C_ADDR_NACK",
  "I2C_DATA_NACK",
  "I2C_ERROR",
  "I2C_TIMEOUT", #I2C bus timeout
  "I2C_PI_DATA_TOO_LONG", #Note: Following 5 fields match I2C error nums + 5
  "I2C_PI_ADDR_NACK",
  "I2C_PI_DATA_NACK",
  "I2C_PI_ERROR",
  "I2C_PI_TIMEOUT", #I2C bus timeout
  "I2C_RX_TIMEOUT",
  "I2C_PI_CRC_ERROR",
  "I2C_NANO_CRC_ERROR",
  "I2C_EXTRA_BYTES",
  "I2C_PI_RETRIED",
]

# LIDAR
SERIAL_PORT = "/dev/ttyS0"  # GPIO serial port on Raspberry Pi


# SLAM/map parameters
MAP_SIZE_PIXELS = 800  # Occupancy grid width/height
MAP_SIZE_METERS = 16.0  # Map width/height in meters


def printableProximity(proxStatus):
    parts = []
    if proxStatus & FRONT_LEFT_PROX_SET:
        parts.append("Front Left")
    if proxStatus & FRONT_RIGHT_PROX_SET:
        parts.append("Front Right")
    if proxStatus & REAR_LEFT_PROX_SET:
        parts.append("Rear Left")
    if proxStatus & REAR_RIGHT_PROX_SET:
        parts.append("Rear Right")
    if proxStatus & FRONT_FRONT_PROX_SET:
        parts.append("Front Front")
    if proxStatus & REAR_REAR_PROX_SET:
        parts.append("Rear Rear")
    return ", ".join(parts) if parts else "None"


smoothedScan = [0] * 360
