import config
from datetime import datetime

from explorer import ExplorationManager
from proximity_scan import processProximityScan


# This class is the upper brain of the robot that determines what to do based on what the robot is currently doing (robotState) and the latest LIDAR scan. It is responsible for:
# - Deciding the next move for the robot when it is waiting for direction, by processing the latest map and frontier information from the explorer manager
# - Updating the explorer manager with new LIDAR scans when the robot is sweeping or moving, so that the map and frontier information is kept
class RobotStateMonitor:
    def __init__(self, save_queue, reader, explorer_manager):
        self.last_state = None
        self.save_queue = save_queue
        self.reader = reader
        self.explorer_manager : ExplorationManager = explorer_manager
        self.statesSinceLastSweep = []
        self.startDriveHeading = None
        self.move_confidence = -1.0
        self.distance_travelled_since_last_sweep = 0.0
        self.current_bearing = 0.0
        self.driveStarted = False
        self.has_reset = False

    def update_state(self, robotState, current_bearing, distance_travelled):

        self.avg_moving_bearing = current_bearing
        self.current_bearing = current_bearing

        if robotState != self.last_state:
            self.statesSinceLastSweep.append(robotState)
            if (robotState == config.ROBOT_STATE_NAMES.index("SWEEP")):
                self.has_reset = False
                # We have just started a sweep, so we can use the distance travelled since the last sweep to update SLAM with the movement since the last sweep, which will help to keep the map and frontier information accurate for decision making during the sweep, but we need to determine how much confidence to place in the distance travelled measurement based on the sequence of states since the last sweep
                self.distance_travelled_since_last_sweep = distance_travelled * 10.0  # convert cm to mm for SLAM and explorer manager updates
                self.move_confidence = 1.0
                # We are starting a sweep - determine whether distance travelled since last sweep can be used to update SLAM, 
                # or whether we should discard it as unreliable (e.g. due to wheel slip)
                # Basically if the sequence of states is something like ROTATE, DRIVE then SWEEP, then we can use the distance travelled to update SLAM, but if it is something like ROTATE -> SWEEP or multiple ROTATEs -> SWEEP, 
                # then we have good confidence that the distance measured is accurate
                move_while_rotating = False
                rotated_during_drive = False
                for state in self.statesSinceLastSweep:
                    if not self.driveStarted and state == config.ROBOT_STATE_NAMES.index("DRIVE"):
                        self.driveStarted = True
                        self.move_confidence = 0.9  # Default confidence in distance travelled measurement
                    if self.driveStarted and ((state == config.ROBOT_STATE_NAMES.index("ROTATING") or \
                        state == config.ROBOT_STATE_NAMES.index("ROTATING_LEFT") or \
                            state == config.ROBOT_STATE_NAMES.index("ROTATING_RIGHT") or \
                                state == config.ROBOT_STATE_NAMES.index("ROTATING_LEFT_BLOCKED_RIGHT") or \
                                    state == config.ROBOT_STATE_NAMES.index("ROTATING_RIGHT_BLOCKED_LEFT") or \
                                        state == config.ROBOT_STATE_NAMES.index("ROTATING_FRONT_BLOCKED_BACKING_OUT") or \
                                            state == config.ROBOT_STATE_NAMES.index("ROTATING_REAR_BLOCKED_GO_FORWARD"))) \
                          and self.driveStarted:
                        # We have started driving and then rotated, so distance travelled is likely to be inaccurate due to wheel slip during rotation, so discard it
                        print(f"{datetime.now()}: Low confidence in distance travelled due to rotation in drive")
                        if self.move_confidence > 0.2:
                            self.move_confidence = 0.2
                        rotated_during_drive = True
                        break
                    if not self.driveStarted and (state == config.ROBOT_STATE_NAMES.index("ROTATING_FRONT_BLOCKED_BACKING_OUT") or \
                                state == config.ROBOT_STATE_NAMES.index("ROTATING_LEFT_BLOCKED_RIGHT") or \
                                    state == config.ROBOT_STATE_NAMES.index("ROTATING_RIGHT_BLOCKED_LEFT") or \
                                        state == config.ROBOT_STATE_NAMES.index("ROTATING_FRONT_BLOCKED_BACKING_OUT") or \
                                            state == config.ROBOT_STATE_NAMES.index("ROTATING_REAR_BLOCKED_GO_FORWARD")):
                        #We didnt rotate on the spot, so starting location is less reliable
                        move_while_rotating = True
                        if self.move_confidence > 0.5:
                            self.move_confidence = 0.5
                        self.startDriveHeading = None # Clear the start drive heading as we can no longer rely on it for SLAM updates during the sweep if we rotated during the drive
                        print(f"{datetime.now()}: Didnt rotate on the spot so cant trust move distance")
                        break
                    
                if not self.driveStarted and not move_while_rotating:
                    # We never started driving, we just probably rotated on the spot, so we can be confident that we didnt actually move, but we can still use the current bearing to update SLAM during the sweep
                    print(f"{datetime.now()}: Did not start driving - no distance travelled")
                    self.distance_travelled_since_last_sweep = 0
                    if self.move_confidence > 0.7:
                        self.move_confidence = 0.7 #High degree of confidence that we did not actually move, but we can still use the current bearing to update SLAM during the sweep
                    self.startDriveHeading = None # Clear the start drive heading as we can no longer rely on it for SLAM updates during the sweep if we rotated during the drive
                if self.startDriveHeading is not None:
                    bearing_diff = abs(current_bearing - self.startDriveHeading)
                    if bearing_diff > 30 and bearing_diff < 330:
                        print(f"{datetime.now()}: Poor confidence in distance travelled since last sweep due to large change in bearing during drive. Start bearing:", self.startDriveHeading, "Current bearing:", current_bearing)
                        if self.move_confidence > 0.6:
                            self.move_confidence = 0.6
                    # Calculate an average bearing during the drive to use for SLAM updates, which may be more accurate than just using the current bearing at the end of the drive if there was some rotation during the drive
                    self.avg_moving_bearing = config.average_heading(self.startDriveHeading, current_bearing)
                    if self.move_confidence > 0.9:
                        self.move_confidence = 0.9  # Reduce confidence slightly as we are now relying on the average bearing calculation which may be inaccurate if there was significant rotation during the drive, but it is likely to still be more accurate than just using the current bearing at the end of the drive in this case
                    self.startDriveHeading = None
                    # print("Average moving bearing during drive:", self.avg_moving_bearing)
                if rotated_during_drive:
                    if self.driveStarted:
                        print(f"{datetime.now()}: Moved while rotating - low confidence in distance travelled due to rotation in drive")
                        if self.move_confidence > 0.2:
                            self.move_confidence = 0.2
                    else:
                        #Zero distance travelled
                        self.distance_travelled_since_last_sweep = 0
                        self.startDriveHeading = None # Clear the start drive heading as we can no longer rely on it for SLAM updates during the sweep if we rotated during the drive
                        if self.move_confidence > 0.6:
                            self.move_confidence = 0.6 
                        print(f"{datetime.now()}: Moved while rotating without starting drive - low confidence in distance travelled due to rotation without drive:", [config.ROBOT_STATE_NAMES[s] for s in self.statesSinceLastSweep])
                # If the sequence contains BACK_OUT or OFF_GROUND, then we should discard the distance travelled since last sweep, as it is likely to be very inaccurate
                if config.ROBOT_STATE_NAMES.index("BACK_OUT") in self.statesSinceLastSweep or \
                        config.ROBOT_STATE_NAMES.index("OFF_GROUND") in self.statesSinceLastSweep or \
                        config.ROBOT_STATE_NAMES.index("UTURN_SWEEP") in self.statesSinceLastSweep:
                    print(f"{datetime.now()}: Low confidence in distance travelled since last sweep due to BACK_OUT, OFF_GROUND or UTURN_SWEEP in state sequence")
                    self.startDriveHeading = None # Clear the start drive heading as we can no longer rely on it for SLAM updates during the sweep if we rotated during the drive
                    self.distance_travelled_since_last_sweep = 0
                    if self.move_confidence > 0.1:
                        self.move_confidence = 0.1
                if config.ROBOT_STATE_NAMES.index("INIT") in self.statesSinceLastSweep:
                    # Robot has been restarted (probably watchdog failure since last sweep, so reset everything 
                    print(f"{datetime.now()}: ***** Robot has RESET *****")
                    self.move_confidence = 0.0
                    self.distance_travelled_since_last_sweep = 0
                    self.has_reset = True
                        
                # Finally check the final bearing at the end of the drive is reasonably close to the bearing at the start of the drive (within 30 degrees), otherwise we may have had significant rotation during the drive which would make the distance travelled measurement inaccurate, so discard it
                print(f"{datetime.now()}: Distance travelled since last sweep: {self.distance_travelled_since_last_sweep:.0f} mm on avg bearing {self.avg_moving_bearing:.0f} with confidence {self.move_confidence}")
            if robotState == config.ROBOT_STATE_NAMES.index("DRIVE"):
                if self.startDriveHeading is None:
                    self.startDriveHeading = current_bearing
            if self.last_state == config.ROBOT_STATE_NAMES.index("WAITING_FOR_DIRECTION"):
                # No longer waiting for direction, so clear any stored direction to drive
                # This is important to avoid accidentally sending an old direction to drive
                # whilst the explorer manager is processing the new map and determining the next move
                config.piStatus["directionToDrive"] = config.NO_DIRECTION
                config.piStatus["distanceToDrive"] = 0
            if (robotState == config.ROBOT_STATE_NAMES.index("WAITING_FOR_DIRECTION")):
                # Robot is waiting for direction - process map to determine next move
                # Use the scans received during the last sweep to update SLAM 
                print(f"{datetime.now()}: Refining pose at bearing {self.current_bearing} with micro map and merging before determining next move for sweep")
                map_updated = self.explorer_manager.slam.refine_pose_with_micro_map_and_merge(self.current_bearing, \
                    self.avg_moving_bearing, self.distance_travelled_since_last_sweep, \
                    self.move_confidence, self.has_reset)
                print(f"Map updated? {map_updated}")
                # if map_updated:
                #Even if we havent updated the map, proceed on the best guess basis
                #Firstly give it the obstacles from ultrasonic sweep to inform move veto
                print(f"{datetime.now()}: Sending obstacles to explorer manager")
                self.explorer_manager.receive_obstacles(config.obstacles)
                # config.piStatus["directionToDrive"] = config.NO_SAFE_DIRECTION #Default to no safedirection, in case explorer manager fails to give one
                #Set PI status for direction and distance to move, which will be sent to robot on the next status response 
                print(f"{datetime.now()}: Determining next move for robot based on current map and obstacle information...")
                (bearing, distance) = self.explorer_manager.get_next_move(map_updated)
                config.piStatus["directionToDrive"] = int(bearing)
                config.piStatus["distanceToDrive"] = int(distance)
                print(f"{datetime.now()}: Move: {config.piStatus['directionToDrive']} deg for {config.piStatus['distanceToDrive']} cm")
                # Trigger a save of the map and targets for debugging and analysis
                self.save_queue.put("save")
                # else:
                #     print(f"{datetime.now()}: Failed to update map as we appear to be lost - move and then retry")
                #     config.piStatus["directionToDrive"] = config.NO_SAFE_DIRECTION
                #     config.piStatus["distanceToDrive"] = 0
                self.startDriveHeading = None
                self.statesSinceLastSweep = []
                self.move_confidence = 0.0
                self.distance_travelled_since_last_sweep = 0.0
                self.driveStarted = False
            self.last_state = robotState

        #Process the latest LIDAR scan for SLAM and obstacle avoidance    
        scan = self.reader.latest_scan
        if scan is not None:
            self.reader.latest_scan = None
            if (robotState == config.ROBOT_STATE_NAMES.index("SWEEP")):
                # We are stationary during a sweep, so update SLAM
                # print("Updating map with new scan during sweep. Current bearing:", current_bearing, "Distance travelled since last sweep:", distance_travelled, "mm with confidence", distance_confidence)
                self.explorer_manager.slam.update(scan)
            else:
                # Moving or rotating - process scan for obstacle avoidance only
                processProximityScan(scan, robotState)
