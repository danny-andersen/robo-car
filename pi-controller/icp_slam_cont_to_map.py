import numpy as np
import math
from scipy.ndimage import distance_transform_edt


class ICP_SLAM:
    def __init__(self, map_size_m=16.0, resolution=0.02):
        self.res = resolution
        self.size = int(map_size_m / resolution)
        self.map = np.zeros((self.size, self.size), dtype=np.uint8)

        # Distance field + gradients
        self.dist = np.zeros_like(self.map, dtype=float)
        self.grad_x = np.zeros_like(self.map, dtype=float)
        self.grad_y = np.zeros_like(self.map, dtype=float)

        # Start in centre
        map_width_mm = (self.size * self.res) * 1000.0
        self.x = map_width_mm / 2
        self.y = map_width_mm / 2
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
    # Transform points from robot frame to world frame (mm)
    # for a given candidate pose
    # ---------------------------------------------------------
    def transform_points_world(self, pts, x_mm, y_mm, theta):
        c = math.cos(theta)
        s = math.sin(theta)
        wx = x_mm + (c * pts[:, 0] - s * pts[:, 1])
        wy = y_mm + (s * pts[:, 0] + c * pts[:, 1])
        return wx, wy  # arrays (N,), (N,)

    # ---------------------------------------------------------
    # Build distance field from occupancy grid
    # ---------------------------------------------------------
    def update_distance_field(self):
        # Obstacles = 0, free = 1
        occ = (self.map == 0).astype(float)
        self.dist = distance_transform_edt(occ) * self.res  # meters

        # Compute gradients
        self.grad_y, self.grad_x = np.gradient(self.dist)

    # ---------------------------------------------------------
    # Transform scan points into world frame
    # ---------------------------------------------------------
    def transform_points(self, pts, x, y, th):
        c = math.cos(th)
        s = math.sin(th)
        wx = x + (c * pts[:, 0] - s * pts[:, 1])
        wy = y + (s * pts[:, 0] + c * pts[:, 1])
        return wx, wy

    # ---------------------------------------------------------
    # World mm → grid index
    # ---------------------------------------------------------
    def world_to_grid(self, wx, wy):
        x_m = wx / 1000.0
        y_m = wy / 1000.0
        ix = (x_m / self.res).astype(int)
        iy = (y_m / self.res).astype(int)
        iy = self.size - 1 - iy
        return ix, iy

    # ---------------------------------------------------------
    # Continuous scan-to-map optimisation (Gauss-Newton)
    # ---------------------------------------------------------
    def refine_pose(self, pts):
        x = self.x
        y = self.y
        th = self.theta

        for _ in range(8):  # 8 GN iterations
            wx, wy = self.transform_points(pts, x, y, th)
            ix, iy = self.world_to_grid(wx, wy)

            mask = (ix >= 0) & (ix < self.size) & (iy >= 0) & (iy < self.size)
            if not np.any(mask):
                break

            ix = ix[mask]
            iy = iy[mask]

            # Distance and gradients at these points
            d = self.dist[iy, ix]
            gx = self.grad_x[iy, ix]
            gy = self.grad_y[iy, ix]

            # Compute Jacobians wrt x, y, theta
            c = math.cos(th)
            s = math.sin(th)

            px = pts[mask, 0]
            py = pts[mask, 1]

            # Derivatives of world coords wrt theta
            d_wx_dth = -s * px - c * py
            d_wy_dth =  c * px - s * py

            # Chain rule: d(distance)/d(x,y,theta)
            Jx = gx
            Jy = gy
            Jth = gx * d_wx_dth + gy * d_wy_dth

            # Gauss-Newton update
            J = np.vstack((Jx, Jy, Jth)).T
            H = J.T @ J
            b = -J.T @ d

            try:
                delta = np.linalg.solve(H, b)
            except np.linalg.LinAlgError:
                break

            x += delta[0]
            y += delta[1]
            th += delta[2]

        return x, y, th

    # ---------------------------------------------------------
    # Update SLAM
    # ---------------------------------------------------------
    def update(self, scan_mm):
        pts = self.scan_to_points(scan_mm)
        if pts.shape[0] == 0:
            return

        # 1. Update distance field from current map
        self.update_distance_field()

        # 2. Refine pose using continuous scan-to-map
        self.x, self.y, self.theta = self.refine_pose(pts)

        # 3. Update map with scan at refined pose
        self.update_map(pts)

    # ---------------------------------------------------------
    # Occupancy update (global map, fixed frame)
    # ---------------------------------------------------------
    def update_map(self, pts):
        if pts.shape[0] == 0:
            return

        wx, wy = self.transform_points_world(pts, self.x, self.y, self.theta)
        ix, iy = self.world_to_grid(wx, wy)

        mask = (ix >= 0) & (ix < self.size) & (iy >= 0) & (iy < self.size)
        ix = ix[mask]
        iy = iy[mask]

        self.map[iy, ix] = 255  # simple marking; can be made probabilistic

    # ---------------------------------------------------------
    # Accessors
    # ---------------------------------------------------------
    def get_pose(self):
        return self.x, self.y, self.theta

    def get_map(self):
        return self.map
