import math
import numpy as np

class MoveVeto:
    def __init__(self, slam, resolution_m=0.02):
        self.slam = slam
        self.res = resolution_m

    # ---------------------------------------------------------
    # 1. Veto if ultrasonic obstacles block the move
    # ---------------------------------------------------------
    def veto_ultrasonic(self, move, obstacles):
        bearing, distance = move

        for obs in obstacles:
            # Check angular overlap
            if abs(obs["bearing"] - bearing) <= obs["width"] / 2:
                # Check distance conflict
                if obs["avgDistance"] < distance:
                    print("Ultrasonic veto for move:", move)
                    return True
        return False

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
        if self.veto_ultrasonic(move, obstacles):
            return False
        if self.veto_lidar_path(move):
            return False
        if self.veto_unknown(move):
            return False
        return True

    def adjust_move(self, move, obstacles, max_distance_mm=1000):
        bearing, distance  = move

        # 1. Clip long moves
        if distance > max_distance_mm:
            distance = max_distance_mm

        # 2. Clip moves that enter unknown space
        x0, y0, th = self.slam.get_pose()
        br = math.radians(bearing)

        step_mm = 50
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

        # 3. If ultrasonic obstacle blocks the adjusted move, veto
        if self.veto_ultrasonic((bearing, safe_distance), obstacles):
            return None

        # 4. If LIDAR obstacle blocks the adjusted move, veto
        if self.veto_lidar_path((bearing, safe_distance)):
            return None

        return (bearing, safe_distance)
