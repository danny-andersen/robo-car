import config

from proximity_scan import processProximityScan


# This class is the upper brain of the robot that determines what to do based on what the robot is currently doing (robotState) and the latest LIDAR scan. It is responsible for:
# - Deciding the next move for the robot when it is waiting for direction, by processing the latest map and frontier information from the explorer manager
# - Updating the explorer manager with new LIDAR scans when the robot is sweeping or moving, so that the map and frontier information is kept
class RobotStateMonitor:
    def __init__(self, save_queue, reader, explorer_manager):
        self.last_state = None
        self.save_queue = save_queue
        self.reader = reader
        self.explorer_manager = explorer_manager
        self.statesSinceLastSweep = []
        self.startDriveHeading = None

    def update_state(self, robotState, current_bearing, distance_travelled):

        avg_moving_bearing = current_bearing
        if robotState != self.last_state:
            if (self.last_state == config.ROBOT_STATE_NAMES.index("SWEEP")):
                self.statesSinceLastSweep = []
            self.statesSinceLastSweep.append(robotState)
            if (robotState == config.ROBOT_STATE_NAMES.index("SWEEP")):
                # We have just started a sweep, so we can use the distance travelled since the last sweep to update SLAM with the movement since the last sweep, which will help to keep the map and frontier information accurate for decision making during the sweep, but we need to determine how much confidence to place in the distance travelled measurement based on the sequence of states since the last sweep
                distance_travelled = distance_travelled * 10.0  # convert cm to mm for SLAM and explorer manager updates
                move_confidence = 1.0
                # We are starting a sweep - determine whether distance travelled since last sweep can be used to update SLAM, 
                # or whether we should discard it as unreliable (e.g. due to wheel slip)
                # Basically if the sequence of states is something like ROTATE, DRIVE then SWEEP, then we can use the distance travelled to update SLAM, but if it is something like ROTATE -> SWEEP or multiple ROTATEs -> SWEEP, 
                # then we have good confidence that the distance measured is accurate
                driveStarted = False
                move_while_rotating = False
                rotated_during_drive = False
                for state in self.statesSinceLastSweep:
                    if not driveStarted and state == config.ROBOT_STATE_NAMES.index("DRIVE"):
                        driveStarted = True
                        move_confidence = 0.9  # Default confidence in distance travelled measurement
                    elif (state == config.ROBOT_STATE_NAMES.index("ROTATING") or \
                        state == config.ROBOT_STATE_NAMES.index("ROTATING_LEFT") or \
                            state == config.ROBOT_STATE_NAMES.index("ROTATING_RIGHT") or \
                                state == config.ROBOT_STATE_NAMES.index("ROTATING_LEFT_BLOCKED_RIGHT") or \
                                    state == config.ROBOT_STATE_NAMES.index("ROTATING_RIGHT_BLOCKED_LEFT") or \
                                        state == config.ROBOT_STATE_NAMES.index("ROTATING_FRONT_BLOCKED_BACKING_OUT") or \
                                            state == config.ROBOT_STATE_NAMES.index("ROTATING_REAR_BLOCKED_GO_FORWARD")) \
                          and driveStarted:
                        # We have started driving and then rotated, so distance travelled is likely to be inaccurate due to wheel slip during rotation, so discard it
                        print("Low confidence in distance travelled due to rotation in drive:", [config.ROBOT_STATE_NAMES[s] for s in self.statesSinceLastSweep])
                        move_confidence = 0.2
                        rotated_during_drive = True
                        break
                    if state == config.ROBOT_STATE_NAMES.index("ROTATING_FRONT_BLOCKED_BACKING_OUT") or \
                                            state == config.ROBOT_STATE_NAMES.index("ROTATING_REAR_BLOCKED_GO_FORWARD"):
                        #We didnt rotate on the spot, so cant trust the location
                        move_while_rotating = True
                if not driveStarted and not move_while_rotating:
                    # We never started driving, we just probably rotated on the spot, so we can be confident that we didnt actually move, but we can still use the current bearing to update SLAM during the sweep
                    print("Did not start driving - no distance travelled")
                    distance_travelled = 0
                    move_confidence = 0.9 #High degree of confidence that we did not actually move, but we can still use the current bearing to update SLAM during the sweep
                if self.startDriveHeading is not None and move_confidence == 1.0:
                    bearing_diff = abs(current_bearing - self.startDriveHeading)
                    if bearing_diff > 30 and bearing_diff < 330:
                        print("Poor confidence in distance travelled since last sweep due to large change in bearing during drive. Start bearing:", self.startDriveHeading, "Current bearing:", current_bearing)
                        move_confidence = 0.6
                    # Calculate an average bearing during the drive to use for SLAM updates, which may be more accurate than just using the current bearing at the end of the drive if there was some rotation during the drive
                    avg_moving_bearing = config.average_heading(self.startDriveHeading, current_bearing)
                    move_confidence = 0.9 # Reduce confidence slightly as we are now relying on the average bearing calculation which may be inaccurate if there was significant rotation during the drive, but it is likely to still be more accurate than just using the current bearing at the end of the drive in this case
                    self.startDriveHeading = None
                    print("Average moving bearing during drive:", avg_moving_bearing)
                if rotated_during_drive:
                    if driveStarted:
                        print("Moved while rotating - low confidence in distance travelled due to rotation in drive:", [config.ROBOT_STATE_NAMES[s] for s in self.statesSinceLastSweep])
                        move_confidence = 0.1
                    else:
                        #Zero distance travelled
                        distance_travelled = 0
                        move_confidence = 0.3
                        print("Moved while rotating without starting drive - low confidence in distance travelled due to rotation without drive:", [config.ROBOT_STATE_NAMES[s] for s in self.statesSinceLastSweep])
                # If the sequence contains BACK_OUT or OFF_GROUND, then we should discard the distance travelled since last sweep, as it is likely to be very inaccurate
                if config.ROBOT_STATE_NAMES.index("BACK_OUT") in self.statesSinceLastSweep or config.ROBOT_STATE_NAMES.index("OFF_GROUND") in self.statesSinceLastSweep:
                    print("Low confidence in distance travelled since last sweep due to state sequence:", [config.ROBOT_STATE_NAMES[s] for s in self.statesSinceLastSweep])
                    move_confidence = 0.2
                if config.ROBOT_STATE_NAMES.index("INIT") in self.statesSinceLastSweep:
                    move_confidence = 1.0
                    distance_travelled = 0
                # Finally check the final bearing at the end of the drive is reasonably close to the bearing at the start of the drive (within 30 degrees), otherwise we may have had significant rotation during the drive which would make the distance travelled measurement inaccurate, so discard it
                print("Distance travelled since last sweep:", distance_travelled, "mm with confidence", move_confidence)
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
                #Firstly give it the obstacles from ultrasonic sweep to inform move veto
                self.explorer_manager.receive_obstacles(config.obstacles)
                # config.piStatus["directionToDrive"] = config.NO_SAFE_DIRECTION #Default to no safedirection, in case explorer manager fails to give one
                #Set PI status for direction and distance to move, which will be sent to robot on the next status response 
                (bearing, distance) = self.explorer_manager.get_next_move()
                config.piStatus["directionToDrive"] = int(bearing)
                config.piStatus["distanceToDrive"] = int(distance)
                self.startDriveHeading = None
                print("Move:", config.piStatus["directionToDrive"], "deg for", config.piStatus["distanceToDrive"]   , "mm")
                # Trigger a save of the map and targets for debugging and analysis
                self.save_queue.put("save")
            self.last_state = robotState
        else:
            if (robotState == config.ROBOT_STATE_NAMES.index("SWEEP")):
                # State hasn't changed, but if we are in a sweep state then the robot is stationary and we can keep updating SLAM with new LIDAR scans to keep the map and frontier information fresh for decision making
                distance_travelled = 0
                move_confidence = 1.0

        #Process the latest LIDAR scan for SLAM and obstacle avoidance    
        scan = self.reader.latest_scan
        if scan is not None:
            self.reader.latest_scan = None
            if (robotState == config.ROBOT_STATE_NAMES.index("SWEEP")):
                # We are stationary during a sweep, so update SLAM
                # print("Updating map with new scan during sweep. Current bearing:", current_bearing, "Distance travelled since last sweep:", distance_travelled, "mm with confidence", distance_confidence)
                config.explorerManager.update_map(scan, current_bearing, avg_moving_bearing, distance_travelled, move_confidence)
            else:
                # Moving or rotating - process scan for obstacle avoidance only
                processProximityScan(scan, robotState)
