import math
import config




def processProximityScan(scan, robotState):
    # Process LIDAR scan for proximity detection
    # Process each point in the scan to determine proximity around the robot
    # Note that scan is a list of 360 distance measurements in mm
    # Update config.piStatus["lidarProximity"] based on objects detected
    # Proximity threshold varies by robot state and angle
    
    scan_filtered = median_filter(scan)
    scan_filtered = temporal_smooth(scan_filtered)

    config.piStatus["lidarProximity"] = 0 # Clear previous proximity status
    for angle in range(360):
        distance = scan_filtered[angle]
        if distance == 0:
            continue  # No reading
            
        threshold = 0
        bitSet = 0
        
        # Determine if this angle is in front, side, or rear
        # if robotState == 3 or robotState == 5: # ROTATING or UTURN_SWEEP
        #     threshold = config.PROXIMITY_THRESHOLD_ROTATING_MM
        # else:
        #     # Normal driving
        if ((angle <= config.PROXIMITY_ANGLE_REAR_LEFT) and (angle >= 180)): 
            threshold = config.PROXIMITY_THRESHOLD_REAR_MM /math.cos(math.radians(angle - 180))
            bitSet = config.REAR_LEFT_PROX_SET
        elif (angle >= config.PROXIMITY_ANGLE_REAR_RIGHT) and (angle < 180):
            threshold = config.PROXIMITY_THRESHOLD_REAR_MM/math.cos(math.radians(180 - angle))
            bitSet = config.REAR_RIGHT_PROX_SET
        elif ((angle > 0) and (angle < config.PROXIMITY_ANGLE_FRONT_RIGHT)):
            threshold = config.PROXIMITY_THRESHOLD_FRONT_MM/math.cos(math.radians(angle))
            bitSet = config.FRONT_RIGHT_PROX_SET
        elif ((angle >= config.PROXIMITY_ANGLE_FRONT_LEFT) and (angle <= 359)):
            threshold = config.PROXIMITY_THRESHOLD_FRONT_MM/math.cos(math.radians(359 - angle))
            bitSet = config.FRONT_LEFT_PROX_SET

        # Require at least 3 consecutive angles below threshold
        if (distance <= threshold and
            scan_filtered[(angle-1)%360] <= threshold and
            scan_filtered[(angle+1)%360] <= threshold):
            # treat as real obstacle
            config.piStatus["lidarProximity"] |= bitSet 
            
            
            # if ((angle >= config.PROXIMITY_ANGLE_FRONT_LEFT) and (angle <= 359)):
            #     # Object too close in front left
            #     config.piStatus["lidarProximity"] |= config.FRONT_LEFT_PROX_SET
            #     # print("LIDAR Proximity Front Left:", distance, "mm at angle", angle)
            # elif (angle <= config.PROXIMITY_ANGLE_FRONT_RIGHT):
            #     # Object too close in front right
            #     config.piStatus["lidarProximity"] |= config.FRONT_RIGHT_PROX_SET
            #     # print("LIDAR Proximity Front Right:", distance, "mm at angle", angle)
            # elif (angle > config.PROXIMITY_ANGLE_FRONT_RIGHT) and (angle < 180):
            #     # Right rear side
            #     config.piStatus["lidarProximity"] |= config.REAR_RIGHT_PROX_SET
            #     # print("LIDAR Proximity Rear Right Side:", distance, "mm at angle", angle)
            # else:
            #     # Left rear side
            #     config.piStatus["lidarProximity"] |= config.REAR_LEFT_PROX_SET
            #     # print("LIDAR Proximity Rear Left Side:", distance, "mm at angle", angle)
        
    if (config.piStatus["lidarProximity"] != 0):
        print(f"LIDAR Proximity Status: {hex(config.piStatus["lidarProximity"])} {config.printableProximity(config.piStatus["lidarProximity"])}")

# --- 1. Spatial median filter (3-point window) ---
def median_filter(scan):
    filtered = scan.copy()
    for i in range(360):
        a = scan[(i - 1) % 360]
        b = scan[i]
        c = scan[(i + 1) % 360]
        filtered[i] = sorted([a, b, c])[1]  # median of 3
    return filtered

# --- 2. Temporal smoothing (EMA per angle) ---
def temporal_smooth(scan, alpha=0.6):
    for i in range(360):
        if scan[i] > 0:
            config.smoothedScan[i] = int(alpha * scan[i] + (1 - alpha) * config.smoothedScan[i])
    return config.smoothedScan
