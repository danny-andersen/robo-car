import time

from arduino.app_utils import App, Bridge

MAX_NUMBER_OF_OBJECTS_IN_SWEEP=20
MAX_DISTANCE_CAN_MEASURE=200 

class Obstacle:
  relDirection = 0 
  width = 0
  distance = 0
  
  
obstacles = []
numOfObstacles = 0
currentHeading = 0
print("App started")

def resetObstacles(heading: int):
  print(f"Received obstacle reset, with current heading: {heading}")
  currentHeading = heading
  obstacles = []

def obstacleInFront(relDirection, width, distance):
  print(f"Rx new obstacle {numOfObstacles}, direction {relDirection}, width {width} distance {distance}")
  obstacle = Obstacle()
  obstacle.relDirection = relDirection
  obstacle.width = width
  obstacle.distance = distance
  obstacles.append(obstacle)

def getDirectionToDrive() -> int:
  #Find arc with the furthest distance. For those greater than the max, then treat them as equal
  #Of these select the one that is the widest, as this reflects the largest gap to aim for
  furthestObject = obstacles[0]
  closestObject = obstacles[0]
  maxDistanceObjects = []
  retDirection = currentHeading; #default to straightahead
  for obstacle in obstacles:
    if (obstacle.avgDistance >= MAX_DISTANCE_CAN_MEASURE):
      #We have a distant object - save
      maxDistanceObjects.append(obstacle) 
    elif (obstacle.avgDistance > furthestObject.avgDistance):
      furthestObject = obstacle
    if (obstacle.avgDistance < closestObject.avgDistance):
      closestObject = obstacle
  if len(maxDistanceObjects) > 0:
    #Have several objects (gaps) that are too far to measure - choose the widest
    maxWidth = 0;
    widestObject = None
    for obj in maxDistanceObjects:
      if obj.width > maxWidth:
        widestObject = obj
    retDirection = currentHeading + widestObject.relDirection
  else:
    retDirection = currentHeading + furthestObject.relDirection
  print("Direction to drive request returns {retDirection}")
  return retDirection

def mcuSetupComplete() -> int:
  print("MCU set up complete, waiting for data")
  return 0
  
Bridge.provide("getDirectionToDrive", getDirectionToDrive)
Bridge.provide("resetObstacles", resetObstacles)
Bridge.provide("obstacleInFront", obstacleInFront)
Bridge.provide("mcuSetupComplete", mcuSetupComplete)

def loop():
    """This function is called repeatedly by the App framework."""
    # You can replace this with any code you want your App to run repeatedly.
    print("In loop")
    time.sleep(15)


# See: https://docs.arduino.cc/software/app-lab/tutorials/getting-started/#app-run
App.run(user_loop=loop)
