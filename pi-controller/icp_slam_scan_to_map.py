import numpy as np
import math
from scipy.ndimage import distance_transform_edt

import config


class ICP_SLAM:
    def __init__(self, map_size_m=16.0, resolution=0.02):
        """
        map_size_m: physical width/height of map in meters
        resolution: meters per cell
        """
        self.res = resolution
        self.size = int(map_size_m / resolution)  # pixels per side
        self.map = np.zeros((self.size, self.size), dtype=np.uint8)

        # Distance field
        self.dist = np.zeros_like(self.map, dtype=float)

        # Pose in world frame (mm, mm, rad)
        map_width_mm = (self.size * self.res) * 1000.0
        self.x = map_width_mm / 2.0
        self.y = map_width_mm / 2.0
        self.theta = 0.0

    # ---------------------------------------------------------
    # Polar scan → Cartesian points in robot frame (mm)
    # ---------------------------------------------------------
    def scan_to_points(self, scan_mm):
        angles = np.deg2rad(np.arange(360))
        d = np.array(scan_mm, dtype=float)
        mask = (d > 50) & (d < 12000)
        if not np.any(mask):
            return np.zeros((0, 2))
        x = d[mask] * np.cos(angles[mask])
        y = d[mask] * np.sin(angles[mask])
        return np.vstack((x, y)).T

    # ---------------------------------------------------------
    # Distance field from occupancy grid
    # ---------------------------------------------------------
    def update_distance_field(self):
        # Obstacles = 0, free = 1
        occ = (self.map == 0).astype(float)
        self.dist = distance_transform_edt(occ) * self.res  # meters

    # ---------------------------------------------------------
    # Robot frame → world frame (mm)
    # ---------------------------------------------------------
    def transform_points_world(self, pts, x_mm, y_mm, theta):
        c = math.cos(theta)
        s = math.sin(theta)
        wx = x_mm + (c * pts[:, 0] - s * pts[:, 1])
        wy = y_mm + (s * pts[:, 0] + c * pts[:, 1])
        return wx, wy

    # ---------------------------------------------------------
    # World mm → grid indices
    # ---------------------------------------------------------
    def world_to_grid(self, wx, wy):
        x_m = wx / 1000.0
        y_m = wy / 1000.0
        ix = (x_m / self.res).astype(int)
        iy = (y_m / self.res).astype(int)
        iy = self.size - 1 - iy  # flip Y
        return ix, iy

    # ---------------------------------------------------------
    # Pose scoring using distance field (lower distance = better)
    # ---------------------------------------------------------
    def score_pose(self, pts, x_mm, y_mm, theta):
        if pts.shape[0] == 0:
            return -np.inf

        wx, wy = self.transform_points_world(pts, x_mm, y_mm, theta)
        ix, iy = self.world_to_grid(wx, wy)

        mask = (ix >= 0) & (ix < self.size) & (iy >= 0) & (iy < self.size)
        if not np.any(mask):
            return -np.inf

        ix = ix[mask]
        iy = iy[mask]

        # Negative sum of distances → higher score = better alignment
        return -np.sum(self.dist[iy, ix])

    # ---------------------------------------------------------
    # Scan-to-map refinement: x,y only (theta from IMU)
    # ---------------------------------------------------------
    def refine_pose_scan_to_map(self, pts):
        trans_step_mm = 20.0  # 2 cm
        trans_offsets = [-trans_step_mm, 0.0, trans_step_mm]

        # No rotation search: theta fixed from IMU
        rot_offsets = [0.0]

        base_score = self.score_pose(pts, self.x, self.y, self.theta)
        best_score = base_score
        best_pose = (self.x, self.y, self.theta)

        for dx in trans_offsets:
            for dy in trans_offsets:
                for dth in rot_offsets:  # dth is always 0.0 here
                    cx = self.x + dx
                    cy = self.y + dy
                    cth = self.theta  # fixed

                    score = self.score_pose(pts, cx, cy, cth)
                    if score > best_score:
                        best_score = score
                        best_pose = (cx, cy, cth)

        return best_pose, base_score, best_score

    # ---------------------------------------------------------
    # Map update at current pose
    # ---------------------------------------------------------
    def update_map(self, pts):
        if pts.shape[0] == 0:
            return

        wx, wy = self.transform_points_world(pts, self.x, self.y, self.theta)
        ix, iy = self.world_to_grid(wx, wy)

        # Robot position in grid coords
        rx, ry = self.world_to_grid(
            np.array([self.x]), 
            np.array([self.y])
        )
        rx = int(rx[0])
        ry = int(ry[0])

        for hx, hy in zip(ix, iy):
            if not (0 <= hx < self.size and 0 <= hy < self.size):
                continue

            # Bresenham ray from robot to hit
            for cx, cy in self.bresenham(rx, ry, hx, hy)[:-1]:
                if self.map[cy, cx] == config.UNKNOWN:   # only overwrite unknown
                    self.map[cy, cx] = config.FREE   # FREE

            # Mark hit cell as occupied
            self.map[hy, hx] = config.OCCUPIED


    def bresenham(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy

        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
        return points

    # ---------------------------------------------------------
    # Main update: scan + IMU heading
    # ---------------------------------------------------------
    def update(self, scan_mm, imu_theta):
        pts = self.scan_to_points(scan_mm)
        if pts.shape[0] == 0:
            return

        # 1. Use IMU heading as primary theta
        self.theta = math.radians(90.0 - imu_theta)
        
        # 2. Build distance field from current map
        self.update_distance_field()

        # 3. Refine x,y only using scan-to-map
        (new_x, new_y, new_th), old_score, new_score = \
            self.refine_pose_scan_to_map(pts)

        # Simple gating: accept only if score improves
        if new_score > old_score:
            self.x = new_x
            self.y = new_y
            # theta remains IMU-driven

        # 4. Update map with scan at (x,y,theta)
        self.update_map(pts)

    # ---------------------------------------------------------
    # Accessors
    # ---------------------------------------------------------
    def get_pose(self):
        return self.x, self.y, self.theta

    def get_map(self):
        return self.map

 