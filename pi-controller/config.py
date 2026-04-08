import struct
import math
import numpy as np


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
PI_STATUS_RESPONSE = 0x80 | REQ_STATUS_CMD  # Response to status request

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

NO_DIRECTION = 1000  #  Special value indicating no direction to drive has been specified by PI
NO_SAFE_DIRECTION = 1001  # Special value indicating no safe direction to drive from map or obstacles

# -----------------------------
# Struct definitions
# -----------------------------
ObstacleData_struct = struct.Struct("<bHHH")  # obstacleNum, bearing, width, avgDistance
ObstaclesCmd_struct = struct.Struct("<hB")  # currentCompassDirn, numToSend
SystemStatusStruct = struct.Struct(
    "<LhhHBBhbbBBBBB"
)  # timestamp, humidity, tempC, batteryVoltage, robotState, proximitySensors, currentBearing, pitch, roll, rightWheelSpeed, leftWheelSpeed, averageSpeed, distanceTravelled, errorField, CRC
PiStatusStruct = struct.Struct("<BBHB")  # systemReady, lidarProximity, directionToDrive, distanceToDrive


MAX_OBS = 20

# UART packet format:
# [START_BYTE][CMD][PAYLOAD...][CRC16]
START_BYTE = 0xAA
GPIO_SERIAL_PORT = "/dev/ttyS0"  # GPIO serial port on Raspberry Pi

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
    "timestamp": 0,
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
    "directionToDrive": NO_DIRECTION,  # Value indicating no direction set
    "distanceToDrive": 0,
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
    "WAITING_FOR_DIRECTION",
]

ERROR_FIELD_NAMES = [
  "NO_ERROR",
  "I2C_DATA_TOO_LONG", #Note: first 5 fields match I2C error nums
  "I2C_ADDR_NACK",
  "I2C_DATA_NACK",
  "I2C_ERROR",
  "I2C_TIMEOUT", #I2C bus timeout
  "I2C_RX_TIMEOUT",
  "I2C_NANO_CRC_ERROR",
  "I2C_NANO_RETRIED",
  "I2C_EXTRA_BYTES", #Received extra bytes when not expecting them (in flush())
  "PI_DATA_LEN_ERROR", #Note: Following 5 fields match I2C error nums + 5
  "PI_SEQ_ERROR",
  "PI_MSG_TYPE_ERROR",
  "PI_HEADER_LEN_ERR",
  "PI_ERROR",
  "PI_TIMEOUT", #Msg timeout
  "PI_CRC_ERROR",
  "PI_RETRIED",
]

lastBootTimeMs = 0

# LIDAR
USB_SERIAL_PORT = "/dev/ttyUSB0"  # USB serial port for LIDAR data

LIDAR_OFFSET_DEG = 0.0  # If LIDAR is not perfectly front-facing, set this to the angle offset in degrees (positive = CCW, negative = CW) to correct it in SLAM and proximity processing

# SLAM/map parameters
MAP_SIZE_PIXELS = 800  # Occupancy grid width/height
MAP_SIZE_METERS = 16.0  # Map width/height in meters

map_size = 12.0
map_resolution_m = 0.02
robot_radius_m = 0.1
robot_radius_cells = int(robot_radius_m / map_resolution_m)

# Log-odds parameters 
L_FREE = -0.4
L_OCC = +0.85
L_MIN = -4.0
L_MAX = +4.0

occ_threshold = 0.7

def is_free(I):
    p = I/255
    return p < occ_threshold          # prob that cell is free or not been mapped yet

def is_occupied(I):
    return I/255 >= occ_threshold          # fairly confident obstacle

def classify_cell(I):
    #Turn pixel intensity back to a probability that the cell on the map is occupied
    p = I / 255.0

    if p <= 0.1:
        return "unknown"  # not seen on LIDAR

    if p < occ_threshold:
        return "free"

    if p >= occ_threshold:
        return "occupied"
    
    return "unknown"


def printableProximity(proxStatus):
    parts = []
    if proxStatus & TOP_FRONT_LEFT_PROX_SET:
        parts.append("Top Front Left")
    if proxStatus & FRONT_LEFT_PROX_SET:
        parts.append("Front Left")
    if proxStatus & TOP_FRONT_RIGHT_PROX_SET:
        parts.append("Top Front Right")
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
poses = [] # list of (timestamp, x, y, theta)
explorerManager = None
save_index = 0
output_dir = "./slam_logs"

# ---------------------------------------------------------
# World mm → grid indices
# ---------------------------------------------------------
# def world_to_grid(wx, wy, map_pixels, resolution_m=0.02):
#     x_m = wx / 1000.0
#     y_m = wy / 1000.0
#     ix = (x_m / resolution_m).astype(int)
#     iy = (y_m / resolution_m).astype(int)
#     iy = map_pixels - 1 - iy  # flip Y so +Y world is up in the image
#     return ix, iy
    
def map_rads_to_world(rads):
    degs = math.degrees(rads)
    world_degs = 90 - degs
    return (360 + world_degs) % 360         

def world_to_grid(wx, wy, map_pixels, resolution_m=0.02):
    # Convert to numpy arrays without copying if already arrays
    wx = np.asarray(wx)
    wy = np.asarray(wy)

    # Convert mm → m
    x_m = wx * 0.001
    y_m = wy * 0.001

    # Convert to grid indices
    ix = np.floor_divide(x_m, resolution_m).astype(np.int32)
    iy = np.floor_divide(y_m, resolution_m).astype(np.int32)

    # Flip Y axis for image coordinates
    iy = map_pixels - 1 - iy

    # Return scalars if scalars were passed
    if ix.ndim == 0:
        return int(ix), int(iy)

    return ix, iy

def average_heading(deg1, deg2):
    # Convert degrees to radians
    r1 = math.radians(deg1)
    r2 = math.radians(deg2)

    # Convert to unit vectors
    x = math.cos(r1) + math.cos(r2)
    y = math.sin(r1) + math.sin(r2)

    # Compute average angle
    avg = math.atan2(y, x)

    # Convert back to degrees
    avg_deg = math.degrees(avg)

    # Normalize to 0–360
    return (avg_deg + 360) % 360

# Bresenham's line algorithm
# Used to determine which points on a 2D raster (pixel grid) should be selected to form a close approximation of a straight line 
# between two points
def bresenham(x0, y0, x1, y1):
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy

    while True:
        yield (x0, y0)
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


