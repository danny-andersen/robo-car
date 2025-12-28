import numpy as np
import math
from scipy.ndimage import distance_transform_edt


class ICP_SLAM:
    def __init__(self, map_size_m=16.0, resolution=0.02):
        self.res = resolution
        self.size = int(map_size_m / resolution)
        self.map = np.zeros((self.size, self.size), dtype=np.uint8)

        # Distance field
        self.dist = np.zeros_like(self.map, dtype=float)

        # Pose in mm / rad
        map_width_mm = (self.size * self.res) * 1000.0
        self.x = map_width_mm / 2.0
        self.y = map_width_mm / 2.0
        self.theta = 0.0

    def scan_to_points(self, scan_mm):
        angles = np.deg2rad(np.arange(360))
        d = np.array(scan_mm, dtype=float)
        mask = (d > 50) & (d < 12000)
        if not np.any(mask):
            return np.zeros((0, 2))
        x = d[mask] * np.cos(angles[mask])
        y = d[mask] * np.sin(angles[mask])
        return np.vstack((x, y)).T

    def update_distance_field(self):
        # Obstacles = 0, free = 1
        occ = (self.map == 0).astype(float)
        self.dist = distance_transform_edt(occ) * self.res  # meters

    def transform_points_world(self, pts, x_mm, y_mm, theta):
        c = math.cos(theta)
        s = math.sin(theta)
        wx = x_mm + (c * pts[:, 0] - s * pts[:, 1])
        wy = y_mm + (s * pts[:, 0] + c * pts[:, 1])
        return wx, wy

    def world_to_grid(self, wx, wy):
        x_m = wx / 1000.0
        y_m = wy / 1000.0
        ix = (x_m / self.res).astype(int)
        iy = (y_m / self.res).astype(int)
        iy = self.size - 1 - iy
        return ix, iy

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

        # Lower distance = better match → use negative sum as score
        return -np.sum(self.dist[iy, ix])

    def refine_pose_scan_to_map(self, pts):
        # Small search around current pose
        trans_step_mm = 20.0  # 2 cm
        rot_step_rad = math.radians(0.5)  # smaller angular step

        trans_offsets = [-trans_step_mm, 0.0, trans_step_mm]
        rot_offsets = [-rot_step_rad, 0.0, rot_step_rad]

        base_score = self.score_pose(pts, self.x, self.y, self.theta)
        best_score = base_score
        best_pose = (self.x, self.y, self.theta)

        for dx in trans_offsets:
            for dy in trans_offsets:
                for dth in rot_offsets:
                    cx = self.x + dx
                    cy = self.y + dy
                    cth = self.theta + dth

                    score = self.score_pose(pts, cx, cy, cth)
                    if score > best_score:
                        best_score = score
                        best_pose = (cx, cy, cth)

        return best_pose, base_score, best_score

    def update(self, scan_mm):
        pts = self.scan_to_points(scan_mm)
        if pts.shape[0] == 0:
            return

        # 1. Build distance field from current map
        self.update_distance_field()

        # 2. Refine pose with scan-to-map
        (new_x, new_y, new_th), old_score, new_score = self.refine_pose_scan_to_map(pts)

        dx = new_x - self.x
        dy = new_y - self.y
        dth = new_th - self.theta

        score_improvement = new_score - old_score

        # Gating thresholds
        max_trans_step_mm = 100.0
        max_rot_step_rad = math.radians(5.0)
        min_score_improvement = 0.0  # you can raise this

        # Reject crazy rotations and tiny "fake" improvements
        if (
            score_improvement > min_score_improvement and
            abs(dx) < max_trans_step_mm and
            abs(dy) < max_trans_step_mm
        ):
            # Rotation is much more fragile – gate it harder
            if abs(dth) < max_rot_step_rad:
                # Accept full update
                self.x, self.y, self.theta = new_x, new_y, new_th
            else:
                # Accept translation, reject rotation
                self.x, self.y = new_x, new_y
                # theta unchanged
        # else: keep previous pose

        # 3. Update map at (possibly updated) pose
        self.update_map(pts)


    def update_map(self, pts):
        if pts.shape[0] == 0:
            return
        wx, wy = self.transform_points_world(pts, self.x, self.y, self.theta)
        ix, iy = self.world_to_grid(wx, wy)
        mask = (ix >= 0) & (ix < self.size) & (iy >= 0) & (iy < self.size)
        ix = ix[mask]
        iy = iy[mask]
        self.map[iy, ix] = 255

    def get_pose(self):
        return self.x, self.y, self.theta

    def get_map(self):
        return self.map
