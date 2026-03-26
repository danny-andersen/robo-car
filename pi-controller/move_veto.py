import math
import numpy as np

class UltrasonicAdjustment:
    def __init__(self, clearance_deg=5):
        self.clearance = clearance_deg

    def adjust_bearing(self, obstacles, bearing):
        """
        Adjust a bearing so it does not fall inside any arc.

        arcs: list of tuples (center_deg, width_deg)
        bearing: angle in degrees

        Returns adjusted bearing.
        """

        angle = self._normalize_angle(bearing)

        for obs in obstacles:
            center = obs["bearing"]
            width = obs["width"]
            if width <= 0:
                continue

            # expand arc by safety clearance on both sides
            width_with_clearance = width + (2 * self.clearance)

            if self._angle_in_arc(angle, center, width_with_clearance):
                angle = self._nearest_outside(angle, center, width_with_clearance)
                
        return angle


    def _normalize_angle(self, angle):
        """Normalize angle to 0–360."""
        return angle % 360


    def _angle_diff(self, a, b):
        """
        Smallest signed angular difference from a to b.
        Result in range [-180, 180].
        """
        return (b - a + 180) % 360 - 180


    def _angle_in_arc(self, angle, center, width):
        """Return True if angle lies within the arc."""
        half = width / 2
        diff = self._angle_diff(center, angle)
        return abs(diff) <= half


    def _nearest_outside(self, angle, center, width):
        """Return the nearest boundary angle outside the arc."""
        half = width / 2

        diff = self._angle_diff(center, angle)

        if abs(diff) > half:
            return self._normalize_angle(angle)

        lower = self._normalize_angle(center - half)
        upper = self._normalize_angle(center + half)

        d_lower = abs(self._angle_diff(angle, lower))
        d_upper = abs(self._angle_diff(angle, upper))

        return lower if d_lower < d_upper else upper
    
class MoveVeto:
    def __init__(self, slam, resolution_m=0.02):
        self.slam = slam
        self.res = resolution_m
        self.ultrasonicAdjustment = UltrasonicAdjustment()

    # ---------------------------------------------------------
    # 1. Veto if ultrasonic obstacles block the move
    # ---------------------------------------------------------
    def adjust_ultrasonic(self, move, obstacles):
        bearing, distance = move
        # Adjust bearing to avoid ultrasonic obstacles
        adjusted_bearing = self.ultrasonicAdjustment.adjust_bearing(
            obstacles, bearing
        )
        return adjusted_bearing, distance


    # ---------------------------------------------------------
    # 2. Veto if LIDAR map shows obstacle along path
    # ---------------------------------------------------------
    def veto_lidar_path(self, move):
        bearing, distance = move
        x0, y0, th = self.slam.get_pose()

        # Convert bearing to radians
        br = math.radians(bearing)

        # Step along the path in small increments
        step_mm = 50  # 5 cm resolution
        steps = int(distance / step_mm)

        for i in range(1, steps + 1):
            px = x0 + math.cos(br) * (i * step_mm)
            py = y0 + math.sin(br) * (i * step_mm)

            # Convert to grid
            ix = int((px / 1000.0) / self.res)
            iy = int((py / 1000.0) / self.res)

            grid = self.slam.get_map()
            h, w = grid.shape

            if ix < 0 or iy < 0 or ix >= w or iy >= h:
                return True  # outside map

            if grid[iy, ix] == 255:
                return True  # obstacle

        return False

    # ---------------------------------------------------------
    # 3. Veto if move is too long
    # ---------------------------------------------------------
    def veto_length(self, move, max_distance_mm=800):
        _, distance = move
        return distance > max_distance_mm

    # ---------------------------------------------------------
    # 4. Veto if move enters unknown space
    # ---------------------------------------------------------
    def veto_unknown(self, move):
        bearing, distance = move
        x0, y0, th = self.slam.get_pose()

        br = math.radians(bearing)
        px = x0 + math.cos(br) * distance
        py = y0 + math.sin(br) * distance

        ix = int((px / 1000.0) / self.res)
        iy = int((py / 1000.0) / self.res)

        grid = self.slam.get_map()
        h, w = grid.shape

        if ix < 0 or iy < 0 or ix >= w or iy >= h:
            return True

        return grid[iy, ix] == 0  # unknown

    # ---------------------------------------------------------
    # Main API: check if move is safe
    # ---------------------------------------------------------
    def is_safe(self, move, obstacles):
        if self.veto_length(move):
            return False
        if self.veto_lidar_path(move):
            return False
        if self.veto_unknown(move):
            return False
        return True

    def adjust_move(self, move, obstacles, max_distance_mm=1000):
        bearing, distance  = move

        print("Original move:", move)
        # 1. Clip long moves
        if distance > max_distance_mm:
            distance = max_distance_mm

        # 2. Adjust bearing to avoid ultrasonic obstacles (shouldnt need to as candidate moves should already be filtered)
        bearing, distance = self.adjust_ultrasonic(move, obstacles)
        
        # 2. Clip moves that enter unknown space
        x0, y0, th = self.slam.get_pose()
        br = math.radians(bearing)

        step_mm = 50.0
        steps = int(distance / step_mm)

        grid = self.slam.get_map()
        h, w = grid.shape

        safe_distance = distance

        for i in range(1, steps + 1):
            px = x0 + math.cos(br) * (i * step_mm)
            py = y0 + math.sin(br) * (i * step_mm)

            ix = int((px / 1000.0) / self.res)
            iy = int((py / 1000.0) / self.res)

            if ix < 0 or iy < 0 or ix >= w or iy >= h:
                safe_distance = (i - 1) * step_mm
                break

            if grid[iy, ix] == 0:  # unknown
                safe_distance = (i - 1) * step_mm
                break

        print("Adjusted move:", (bearing, safe_distance))

        # 4. If LIDAR obstacle blocks the adjusted move, veto
        if self.veto_lidar_path((bearing, safe_distance)):
            print("Move vetoed due to LIDAR obstacle")
            return None

        return (bearing, safe_distance)
