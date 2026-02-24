import config


class RobotStateMonitor:
    def __init__(self, save_queue):
        self.last_state = None
        self.save_queue = save_queue

    def update_state(self, robotState):

        if robotState != self.last_state:
            if self.last_state == config.ROBOT_STATE_NAMES.index("SWEEP"):
                # No longer updating the map so trigger a save
                self.save_queue.put("save")
            elif self.last_state == config.ROBOT_STATE_NAMES.index("WAITING_FOR_DIRECTION"):
                # No longer waiting for direction, so clear any stored direction to drive
                config.piStatus["directionToDrive"] = config.NO_DIRECTION
                config.piStatus["distanceToDrive"] = 0
            if (robotState == config.ROBOT_STATE_NAMES.index("WAITING_FOR_DIRECTION")):
                # Robot is waiting for direction - process map to determine next move
                #Firstly give it the obstacles from ultrasonic sweep to inform move veto
                config.explorerManager.receive_obstacles(config.obstacles)
                # config.piStatus["directionToDrive"] = config.NO_SAFE_DIRECTION #Default to no safedirection, in case explorer manager fails to give one
                #Set PI status for direction and distance to move, which will be sent to robot on the next status response 
                (bearing, distance) = config.explorerManager.get_next_move()
                config.piStatus["directionToDrive"] = int(bearing)
                config.piStatus["distanceToDrive"] = int(distance)
                print("Move:", config.piStatus["directionToDrive"], "deg for", config.piStatus["distanceToDrive"]   , "mm")
            self.last_state = robotState

    
