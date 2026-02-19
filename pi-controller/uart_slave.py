import serial
from datetime import datetime
import time
import queue

#Local imports
import config
import log_changes


# Thread-safe FIFO queue
status_fifo = queue.Queue(maxsize=200)   # adjust size as needed

ser = serial.Serial(config.GPIO_SERIAL_PORT, 115200, timeout=0.01)

def crc16_update(crc, data):
    crc ^= data
    for _ in range(8):
        if crc & 1:
            crc = (crc >> 1) ^ 0xA001
        else:
            crc >>= 1
    return crc

def crc16(buf):
    crc = 0xFFFF
    for b in buf:
        crc = crc16_update(crc, b)
    return crc


def on_receive(cmd, data):

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
            config.obstacles = [{"bearing": 0, "width": 0, "avgDistance": 0} for _ in range(config.MAX_OBS)]

    # -----------------------------
    # NEXT_OBSTACLE_CMD
    # -----------------------------
    elif cmd == config.NEXT_OBSTACLE_CMD:
        if len(data) >= 1 + config.ObstacleData_struct.size:
            payload = data[1:1 + config.ObstacleData_struct.size]
            obstacleNum, relDir, width, dist = config.ObstacleData_struct.unpack(payload)
            config.obstacles[obstacleNum]["bearing"] = relDir
            config.obstacles[obstacleNum]["width"] = width
            config.obstacles[obstacleNum]["avgDistance"] = dist
        
    elif cmd == config.SEND_SYSTEM_STATUS_CMD:
        if len(data) >= 1+config.SystemStatusStruct.size:
            payload = data[1:1+config.SystemStatusStruct.size]
            unpacked = config.SystemStatusStruct.unpack(payload)
            (config.systemStatus["timestamp"],
            config.systemStatus["humidity"],
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
            config.systemStatus["distanceTravelled"],
            config.systemStatus["errorField"],
            ) = unpacked
            # print("System status updated:", config.systemStatus)
            # print(f"Payload data: {len(data)} bytes, {data} {unpacked}")
            if (config.systemStatus["robotState"] == 0):
                config.lastBootTime = datetime.now().timestamp()
                # print("Robot state: INIT\r")
    elif cmd == config.REQ_STATUS_CMD:
        pass  # No additional data to process

    else:
        print("Unknown command received:", cmd)
        pass  # Unknown command

   
def msg_process_thread():

   while True:
       data = status_fifo.get()   # blocks until data available
       cmd  = data[0]
       on_receive(cmd, data)
       if (cmd == config.SEND_SYSTEM_STATUS_CMD):
           #Only log changes when system status updates, not for obstacles which can be very noisy
           log_changes.record_status_change()
        #    print(config.systemStatus)
    #    elif (cmd == config.NEXT_OBSTACLE_CMD):
    #         print(config.obstacles[config.numObstaclesRx - 1])
    #    elif (cmd == config.SENDING_OBSTACLES_CMD):
    #         print(config.obstaclesCmd)
    #    print(f"Received {cmd}\n\r")


def read_exact(n, timeout=0.05):
    buf = bytearray()
    start = time.time()
    while len(buf) < n and (time.time() - start) < timeout:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
    return bytes(buf) if len(buf) == n else None

def uart_rx_thread():
    while True:
        # 1) Find start byte
        b = ser.read(1)
        if not b:
            continue
        if b[0] != config.START_BYTE:
            # print(f"{b[0]} != {config.START_BYTE}")
            continue

        # print("Start byte detected, reading frame...")
        # 2) Read cmd, seq
        hdr = read_exact(2)
        if hdr is None:
            print("Failed to read cmd and seq bytes")
            continue
        cmd, seq = hdr[0], hdr[1]
        
        #Determine payload length based on cmd
        match cmd:
            case config.SENDING_OBSTACLES_CMD:
                payload_len = config.ObstaclesCmd_struct.size
            case config.NEXT_OBSTACLE_CMD:
                payload_len = config.ObstacleData_struct.size
            case config.SEND_SYSTEM_STATUS_CMD:
                payload_len = config.SystemStatusStruct.size
            case config.REQ_STATUS_CMD: 
                payload_len = 0

        payload = b''
        if payload_len > 0:
            payload = read_exact(payload_len)
            if payload is None:
                print(f"Failed to read full payload {payload_len} for cmd {cmd}")
                continue

        # 3) Read CRC
        crc_bytes = read_exact(2)
        if crc_bytes is None:
            print("Failed to read CRC bytes")
            continue
        recv_crc = crc_bytes[0] | (crc_bytes[1] << 8)

        # 4) Check CRC over cmd + seq + payload
        crc_buf = bytes([cmd, seq]) + payload
        calc_crc = crc16(crc_buf)
        if calc_crc != recv_crc:
            print(f"CRC mismatch: cmd {cmd} payload {payload} calculated {calc_crc}, received {recv_crc}")
            continue  # bad frame, drop

        # 5) Add payload to FIFO for processing by main thread
        status_fifo.put(bytes([cmd]) + payload)  # include cmd for processing thread
        
        # 6) Build response
        resp_cmd = config.PI_STATUS_RESPONSE
        status_bytes = config.PiStatusStruct.pack(config.piStatus["systemReady"], 
                                            config.piStatus["lidarProximity"], 
                                            config.piStatus["directionToDrive"])
        resp_crc_buf = bytes([resp_cmd, seq]) + status_bytes
        resp_crc = crc16(resp_crc_buf)

        frame = bytearray()
        frame.append(config.START_BYTE)
        frame.append(resp_cmd)
        frame.append(seq)
        frame.extend(status_bytes)
        frame.append(resp_crc & 0xFF)
        frame.append((resp_crc >> 8) & 0xFF)

        ser.write(frame)
        # print(f"Sent response for cmd {cmd} with seq {seq} {config.piStatus}")