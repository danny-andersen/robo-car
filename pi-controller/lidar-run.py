import threading
import time
import numpy as np
import math

import config
from uart_slave import msg_process_thread, uart_rx_thread
from ld19_reader import LD19Reader
from icp_slam_scan_to_map import ICP_SLAM
from proximity_scan import processProximityScan
from frontiers import Explorer, ExplorationManager

def processLidarScan(current_bearing, robotState, slam, reader):
    current_bearingRads = current_bearing * (math.pi / 180.0)
    if (robotState == config.ROBOT_STATE_NAMES.index("SWEEP")): # SWEEP
        # We are stationary during a sweep, so update SLAM
        scan = reader.latest_scan
        if scan is None:
            return
        reader.latest_scan = None
        config.explorerManager.update_map(scan, current_bearingRads)
        # Save pose 
        t = time.time()
        x, y, th = slam.get_pose()
        config.poses.append((t, x, y, th))        
        # Periodically save map + poses 
        if len(config.poses) % 50 == 0:
            np.save("./slam_logs/map.npy", slam.get_map())
            np.savetxt("./slam_logs/poses.csv", config.poses, delimiter=",")    
    elif (robotState == config.ROBOT_STATE_NAMES.index("WAITING_FOR_DIRECTION")):
        # Robot is waiting for direction - process map to determine next move
        #Firstly give it the obstacles from ultrasonic sweep to inform move veto
        config.explorerManager.receive_obstacles(config.obstacles)
        config.piStatus["directionToDrive"] = config.NO_DIRECTION #Default to no direction, in case explorer manager fails to give one
        #Set PI status for direction and distance to move, which will be sent to robot on the next status response 
        config.piStatus["directionToDrive"], config.piStatus["distanceToDrive"] = config.explorerManager.get_next_move()
        print("Move:", config.piStatus["directionToDrive"], "deg for", config.piStatus["distanceToDrive"]   , "mm")
    
    else:
        # Moving or rotating - process scan for obstacle avoidance only
        processProximityScan(scan, robotState)

        # grid = slam.get_map()
        # x, y, th = slam.get_pose()
        # self.map_widget.update_map_and_pose(grid, (x, y, th))
    
if __name__ == '__main__':
    slam = ICP_SLAM(map_size_m=16.0, resolution=0.02)
    reader = LD19Reader(config.USB_SERIAL_PORT)
    reader.start()

    processing_thread = threading.Thread( target=msg_process_thread, daemon=True ) 
    processing_thread.start()    
    uart_thread = threading.Thread( target=uart_rx_thread, daemon=True ) 
    uart_thread.start()
        
    config.explorerManager = ExplorationManager(slam=slam, resolution_m=0.02)
    config.piStatus["systemReady"] = 1

    interval = 0.050 # 50 ms 
    next_run = time.perf_counter()
    try:
        # Main loop
        while True:
            now = time.perf_counter()
            if now >= next_run: 
                processLidarScan(config.systemStatus["currentBearing"], config.systemStatus["robotState"], slam, reader) 
                next_run += interval 
            # Tiny sleep to avoid 100% CPU 
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        pass

    finally:
        reader.stop()
