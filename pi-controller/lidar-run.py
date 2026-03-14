import queue
import threading
import time
import numpy as np
import math
import json
import os

import config
from robot_state_monitor import RobotStateMonitor
from uart_slave import msg_process_thread, uart_rx_thread
from ld19_reader import LD19Reader
from icp_slam_scan_to_map import ICP_SLAM
from frontiers import ExplorationManager

def save_worker_thread(slam, save_queue):
    while True:
        task = save_queue.get()   # blocks until needed

        if task == "save":
            print("Saving SLAM map and poses...")
            # Extract map + pose safely
            map = slam.get_map().copy()
            explorationManager: ExplorationManager = config.explorerManager
            clusters = explorationManager.clusters
            chosen_target = explorationManager.target
            next_waypoint = explorationManager.next_waypoint
            pose = slam.get_pose()  # (x, y, theta)
            # Perform slow disk writes
            np.save(f"{config.output_dir}/map_{config.save_index:04d}.npy", map)

            # Save pose and frontier data with timestamp
            t = time.time()

            data = {
                "timestamp": t,
                "pose_index": config.save_index,
                "robot_pose": pose,
                "frontier_clusters": clusters,
                "chosen_target": chosen_target,
                "next_waypoint": next_waypoint,
                "status": (config.systemStatus["tempC"],config.systemStatus["humidity"], config.systemStatus["batteryVoltage"]),
                "obstacles": config.obstacles,
            }

            fname = f"clusters_{config.save_index:05d}.json"
            path = os.path.join(config.output_dir, fname)

            with open(path, "w") as f:
                json.dump(data, f, indent=2)

            # config.poses.append((t, x, y, th))        

            # # Append pose to CSV 
            # with open("./slam_logs/poses.csv", "a") as f:
            #     f.write(f"{t},{x},{y},{th}\n")  

            # np.save("./slam_logs/scans.npy", np.array(slam.scan_log, dtype=object))
            # np.save(f"./slam_logs/clusters_{config.save_index:04d}.npy", np.array(config.explorerManager.clusters), dtype=object)            
            # with open("./slam_logs/targets.csv", "a") as f:
            #     f.write(f"{config.explorerManager.target[0]},{config.explorerManager.target[1]}\n")  

            
            config.save_index += 1
            
        save_queue.task_done()


if __name__ == '__main__':
    slam = ICP_SLAM(map_size_m=6.0, resolution=0.02)
    reader = LD19Reader(config.USB_SERIAL_PORT)
    reader.start()

    processing_thread = threading.Thread( target=msg_process_thread, daemon=True ) 
    processing_thread.start()    
    uart_thread = threading.Thread( target=uart_rx_thread, daemon=True ) 
    uart_thread.start()
 
    last_state = None
    save_queue = queue.Queue()
    config.save_index = 0

    with open("./slam_logs/poses.csv", "w") as f:
        f.write("")
 
    save_thread = threading.Thread(
        target=save_worker_thread,
        args=(slam, save_queue),
        daemon=True
    )
    save_thread.start()
        
    config.explorerManager = ExplorationManager(slam=slam, resolution_m=0.02)
    state_monitor = RobotStateMonitor(save_queue, reader=reader, explorer_manager=config.explorerManager)
    
    config.piStatus["systemReady"] = 1

    interval = 0.020 # Update robot state every 20 ms and check if a LIDAR scan is available to process for SLAM and obstacle avoidance - this is a good balance between keeping the map and frontier information fresh for decision making, and not overloading the CPU with too frequent updates and SLAM processing 
    next_run = time.perf_counter()
    try:
        # Main loop
        while True:
            now = time.perf_counter()
            if now >= next_run: 
                state_monitor.update_state(config.systemStatus["robotState"], config.systemStatus["currentBearing"], config.systemStatus["distanceTravelled"],)
                # print("Robot State:", config.ROBOT_STATE_NAMES[robotState])
                next_run += interval 
            # Tiny sleep to avoid 100% CPU 
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        pass

    finally:
        reader.stop()
