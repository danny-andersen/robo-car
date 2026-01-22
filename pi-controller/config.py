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

# -----------------------------
# Struct definitions
# -----------------------------
ObstacleData_struct = struct.Struct("<HHH")
ObstaclesCmd_struct = struct.Struct("<hB")
SystemStatusStruct = struct.Struct("<hhHBBhbbBBBB")
PiStatusStruct = struct.Struct("<BBH")

MAX_OBS = 20

# -----------------------------
# State variables
# -----------------------------
currentCommand = 0
numObstaclesRx = 0

obstaclesCmd = {
    "currentCompassDirn": 0,
    "numOfObstaclesToSend": 0
}

obstacles = [
    {"bearing": 0, "width": 0, "avgDistance": 0}
    for _ in range(MAX_OBS)
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
    # "checksum": 0
}

piStatus = {
    "systemReady": 0,
    "lidarRunning": 0,
    "directionToDrive": 1000 # Value indicating no direction set
}

