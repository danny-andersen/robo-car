import time
import math
import threading

import numpy as np

import config
from i2c_slave import i2c_init, msg_process_thread
from ld19_reader import LD19Reader
from icp_slam_scan_to_map import ICP_SLAM
from proximity_scan import processProximityScan

poses = [] # list of (timestamp, x, y, theta)

        
def process_lidar(current_bearing, robotState, slam, reader):
    scan = reader.latest_scan
    if scan is None:
        return

    reader.latest_scan = None
    current_bearingRads = current_bearing * (math.pi / 180.0)
    if (robotState == 4): # SWEEP
        # We are stationary during a sweep, so update SLAM
        slam.update(scan, current_bearingRads)
        # Save pose 
        t = time.time()
        x, y, th = slam.get_pose()
        poses.append((t, x, y, th))        
        # Periodically save map + poses 
        if len(poses) % 50 == 0:
            np.save("./slam_logs/map.npy", slam.get_map())
            np.savetxt("./slam_logs/poses.csv", poses, delimiter=",")
    else:
        # Moving or rotating - process scan for obstacle avoidance only
        processProximityScan(scan, robotState)

    # grid = slam.get_map()
    # x, y, th = slam.get_pose()
    # self.map_widget.update_map_and_pose(grid, (x, y, th))
    
if __name__ == '__main__':
    slam = ICP_SLAM(map_size_m=16.0, resolution=0.02)
    reader = LD19Reader(config.SERIAL_PORT)
    reader.start()

    processing_thread = threading.Thread( target=msg_process_thread, daemon=True ) 
    processing_thread.start()    
    
    eventHandler = i2c_init()
    
    interval = 0.050 # 50 ms 
    next_run = time.perf_counter()
    try:
        # Main loop
        while True:
            now = time.perf_counter()
            if now >= next_run: 
                process_lidar(config.systemStatus["currentBearing"], config.systemStatus["robotState"], slam, reader) 
                next_run += interval 
            # Tiny sleep to avoid 100% CPU 
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        pass

    finally:
        eventHandler.cancel()
        config.pi.stop()

