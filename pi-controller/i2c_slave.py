import pigpio
# import RPi.GPIO as GPIO
import time

import config
import log_changes

# GPIO.setmode(GPIO.BCM)

# -----------------------------
# I2C Slave Handlers
# -----------------------------

#Note: cmd is the first byte of data received
#Note: The way the pigpio bsc_i2c works is that it will transmit any data sat in the FIFO buffer when the master does a read.
# So we need to set the FIFO buffer to the response BEFORE the master sends a command. Therefore the PI will always send 
# back the piStatus when the master sends any data, as an acknowledgement of receipt.

def on_receive(cmd, data):
    global numObstaclesRx, obstaclesCmd, obstacles, systemStatus

    if len(data) == 0:
        return

    # -----------------------------
    # SENDING_OBSTACLES_CMD
    # -----------------------------
    if cmd == config.SENDING_OBSTACLES_CMD:
        if len(data) >= 1 + config.ObstaclesCmd_struct.size:
            payload = data[1:1 + config.ObstaclesCmd_struct.size]
            currentCompassDirn, numToSend = config.ObstaclesCmd_struct.unpack(payload)
            config.obstaclesCmd["currentCompassDirn"] = currentCompassDirn
            config.obstaclesCmd["numOfObstaclesToSend"] = numToSend
            numObstaclesRx = 0
            obstacles = [{"bearing": 0, "width": 0, "avgDistance": 0} for _ in range(config.MAX_OBS)]


    # -----------------------------
    # NEXT_OBSTACLE_CMD
    # -----------------------------
    elif cmd == config.NEXT_OBSTACLE_CMD:
        if len(data) >= 1 + config.ObstacleData_struct.size:
            payload = data[1:1 + config.ObstacleData_struct.size]
            relDir, width, dist = config.ObstacleData_struct.unpack(payload)
            config.obstacles[numObstaclesRx]["bearing"] = relDir
            config.obstacles[numObstaclesRx]["width"] = width
            config.obstacles[numObstaclesRx]["avgDistance"] = dist
            numObstaclesRx += 1
        
    elif cmd == config.SEND_SYSTEM_STATUS_CMD:
        if len(data) >= 1+config.SystemStatusStruct.size:
            payload = data[1:1+config.SystemStatusStruct.size]
            unpacked = config.SystemStatusStruct.unpack(payload)
            (config.systemStatus["humidity"],
             config.systemStatus["tempC"],
             config.systemStatus["batteryVoltage"],
             config.systemStatus["robotState"],
             config.systemStatus["proximitySensors"],
             config.systemStatus["currentBearing"],
             config.systemStatus["pitch"],
             config.systemStatus["roll"],
             config.systemStatus["rightWheelSpeed"],
             config.systemStatus["leftWheelSpeed"],
             config.systemStatus["averageSpeed"],
             config.systemStatus["distanceTravelled"]) = unpacked
            # print("System status updated:", systemStatus)
    elif cmd == config.REQ_STATUS_CMD:
        pass  # No additional data to process

    else:
        print("Unknown command received:", cmd)
        pass  # Unknown command


def writeStatusToFIFO():
    response = config.PiStatusStruct.pack(config.piStatus["systemReady"], 
                                          config.piStatus["lidarProximity"], 
                                          config.piStatus["directionToDrive"])
    config.pi.bsc_i2c(config.I2C_ADDR, response)


def i2cEvent(id, tick):
   status, bytesRead, data = config.pi.bsc_i2c(config.I2C_ADDR)
#    print ("I2C Event: status={}, bytesRead={}, data={}".format(status, bytesRead, data))
   if bytesRead:
    cmd = data[0]
    # Always write status to FIFO first so master gets it as ack on next message
    writeStatusToFIFO()
    on_receive(cmd, data)
    if cmd == config.SEND_SYSTEM_STATUS_CMD or cmd == config.REQ_STATUS_CMD:
        log_changes.record_status_change()
    # if (cmd == SEND_SYSTEM_STATUS_CMD):
    #     print(systemStatus)
    # elif (cmd == NEXT_OBSTACLE_CMD):
    #     print(obstacles[numObstaclesRx - 1])
    # elif (cmd == SENDING_OBSTACLES_CMD):
    #     print(obstaclesCmd)
    # print(f"Req status command received {cmd}")

def i2c_init():
    # -----------------------------
    # pigpio I2C slave setup
    # -----------------------------
    config.pi = pigpio.pi()
    if not config.pi.connected:
        raise RuntimeError("pigpio daemon not running")

    eventHandle = config.pi.event_callback(pigpio.EVENT_BSC, i2cEvent)

    config.pi.bsc_i2c(config.I2C_ADDR)
    print(f"I2C slave active at address {config.I2C_ADDR}")
    config.piStatus["systemReady"] = 1
    writeStatusToFIFO()
    return eventHandle

# -----------------------------
# pigpio I2C slave setup
# -----------------------------
if __name__ == '__main__':

    eventHandle = i2c_init()
    # -----------------------------
    # Main loop
    # -----------------------------
    try:
        while True:
            # directionToDrive = (directionToDrive + 10) % 360
            # for i in range(0,MAX_OBS):
            #     if obstacles[i]["avgDistance"] != 0:
            #         print(f"Obstacle {i}: {obstacles[i]}")
            time.sleep(15)

    except KeyboardInterrupt:
        pass

    finally:
        eventHandle.cancel()
        config.pi.stop()
