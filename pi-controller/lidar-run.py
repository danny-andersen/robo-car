import queue
import threading
import time
import numpy as np
import math

import config
from robot_state_monitor import RobotStateMonitor
from uart_slave import msg_process_thread, uart_rx_thread
from ld19_reader import LD19Reader
from icp_slam_scan_to_map import ICP_SLAM
from proximity_scan import processProximityScan
from frontiers import Explorer, ExplorationManager

def processLidarScan(current_bearing, robotState, reader):
    scan = reader.latest_scan
    if scan is None:
        return
    reader.latest_scan = None
    if (robotState == config.ROBOT_STATE_NAMES.index("SWEEP")):
        # We are stationary during a sweep, so update SLAM
        current_bearingRads = current_bearing * (math.pi / 180.0)
        config.explorerManager.update_map(scan, current_bearingRads)
    else:
        # Moving or rotating - process scan for obstacle avoidance only
        processProximityScan(scan, robotState)
   

def save_worker_thread(slam, save_queue):
    while True:
        task = save_queue.get()   # blocks until needed

        if task == "save":
            print("Saving SLAM map and poses...")
            # Extract map + pose safely
            map = slam.get_map().copy()
            # Save pose 
            t = time.time()
            x, y, th = slam.get_pose()
            config.poses.append((t, x, y, th))        

            # Perform slow disk writes
            np.save("./slam_logs/map.npy", map)
            # Append pose to CSV 
            with open("./slam_logs/poses.csv", "a") as f:
                f.write(f"{t},{x},{y},{th}\n")  

        save_queue.task_done()


if __name__ == '__main__':
    slam = ICP_SLAM(map_size_m=16.0, resolution=0.02)
    reader = LD19Reader(config.USB_SERIAL_PORT)
    reader.start()

    processing_thread = threading.Thread( target=msg_process_thread, daemon=True ) 
    processing_thread.start()    
    uart_thread = threading.Thread( target=uart_rx_thread, daemon=True ) 
    uart_thread.start()
 
    last_state = None
    save_queue = queue.Queue()

    with open("./slam_logs/poses.csv", "w") as f:
        f.write("")
 
    state_monitor = RobotStateMonitor(save_queue)
    
    save_thread = threading.Thread(
        target=save_worker_thread,
        args=(slam, save_queue),
        daemon=True
    )
    save_thread.start()
        
    config.explorerManager = ExplorationManager(slam=slam, resolution_m=0.02)
    config.piStatus["systemReady"] = 1

    interval = 0.050 # 50 ms 
    next_run = time.perf_counter()
    try:
        # Main loop
        while True:
            now = time.perf_counter()
            if now >= next_run: 
                robotState = config.systemStatus["robotState"]
                state_monitor.update_state(robotState)
                # print("Robot State:", config.ROBOT_STATE_NAMES[robotState])
                processLidarScan(config.systemStatus["currentBearing"], robotState, reader) 
                next_run += interval 
            # Tiny sleep to avoid 100% CPU 
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        pass

    finally:
        reader.stop()
