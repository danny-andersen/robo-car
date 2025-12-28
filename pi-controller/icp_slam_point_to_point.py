import numpy as np
import math

class ICP_SLAM:
    def __init__(self, map_size_m=16.0, resolution=0.02):
        """
        map_size_m: physical size of map (meters)
        resolution: meters per cell
        """
        self.res = resolution
        self.size = int(map_size_m / resolution)   # number of cells per side
        self.map = np.zeros((self.size, self.size), dtype=np.uint8)

        # Start in the center of the map (world coordinates in mm)
        map_width_m = self.size * self.res
        map_width_mm = map_width_m * 1000.0
        self.x = map_width_mm / 2.0   # mm
        self.y = map_width_mm / 2.0   # mm
        self.theta = 0.0              # radians

        self.prev_points = None

    # ---------------------------------------------------------
    # Utility: convert polar scan to Cartesian point cloud
    # ---------------------------------------------------------
    def scan_to_points(self, scan_mm):
        angles = np.deg2rad(np.arange(360))
        d = np.array(scan_mm, dtype=float)

        # Keep reasonable distances
        mask = (d > 50) & (d < 12000)
        if not np.any(mask):
            return np.zeros((0, 2))

        x = d[mask] * np.cos(angles[mask])
        y = d[mask] * np.sin(angles[mask])
        return np.vstack((x, y)).T    # shape (N, 2), in mm

    # ---------------------------------------------------------
    # ICP: align two point clouds (previous → current)
    # ---------------------------------------------------------
    def icp(self, A, B, max_iter=20, tol=1e-4):
        """
        A = previous scan (Nx2)
        B = current scan  (Nx2)
        Returns rotation R (2x2), translation t (2,)
        """
        if len(A) < 20 or len(B) < 20:
            return np.eye(2), np.zeros(2)

        R = np.eye(2)
        t = np.zeros(2)

        for _ in range(max_iter):
            # Transform A by current estimate
            A2 = (A @ R.T) + t

            # Nearest neighbours (naive)
            idx = np.argmin(
                ((A2[:, None, :] - B[None, :, :]) ** 2).sum(axis=2),
                axis=1
            )
            Bcorr = B[idx]

            # Centroids
            Ac = A2.mean(axis=0)
            Bc = Bcorr.mean(axis=0)

            # Subtract centroids
            A0 = A2 - Ac
            B0 = Bcorr - Bc

            # SVD for rotation
            H = A0.T @ B0
            U, _, Vt = np.linalg.svd(H)
            R_new = Vt.T @ U.T

            # Fix reflection
            if np.linalg.det(R_new) < 0:
                Vt[1, :] *= -1
                R_new = Vt.T @ U.T

            t_new = Bc - Ac @ R_new.T

            # Convergence check
            if np.linalg.norm(t_new - t) < tol:
                R, t = R_new, t_new
                break

            R, t = R_new, t_new

        return R, t

    # ---------------------------------------------
    # Update SLAM with a new 360° scan
    # ---------------------------------------------
    def update(self, scan_mm):
        pts = self.scan_to_points(scan_mm)
        if pts.shape[0] == 0:
            return

        if self.prev_points is not None:
            R, t = self.icp(self.prev_points, pts)

            # Extract rotation angle from R
            dtheta = math.atan2(R[1, 0], R[0, 0])
            dx, dy = t   # in mm (same units as points)

            # Transform ICP translation from local frame → world frame
            c = math.cos(self.theta)
            s = math.sin(self.theta)
            wx = c * dx - s * dy
            wy = s * dx + c * dy

            # Integrate pose in world coordinates (if robot has moved)
            # Compute ICP confidence
            A2 = (self.prev_points @ R.T) + t
            idx = np.argmin(((A2[:, None, :] - pts[None, :, :]) ** 2).sum(axis=2), axis=1)
            Bcorr = pts[idx]
            error = np.mean(np.sum((A2 - Bcorr)**2, axis=1))

            # Reject motion if ICP is unreliable
            if error < 200:   # tune threshold
                self.x += wx
                self.y += wy
                self.theta += dtheta
            else:
                # ignore ICP motion
                pass

        self.prev_points = pts
        self.update_map(pts)

    # ---------------------------------------------
    # Occupancy grid update (global map)
    # ---------------------------------------------
    def update_map(self, pts):
        """
        pts: Nx2 array of lidar points in the robot frame (mm)
        Uses current pose (self.x, self.y, self.theta) in world frame.
        """

        c = math.cos(self.theta)
        s = math.sin(self.theta)

        for px, py in pts:
            # Robot frame → world frame (mm)
            wx = self.x + (c * px - s * py)
            wy = self.y + (s * px + c * py)

            # World mm → meters
            x_m = wx / 1000.0
            y_m = wy / 1000.0

            # Meters → grid index
            ix = int(x_m / self.res)
            iy = int(y_m / self.res)

            # Flip Y so (0,0) in world is bottom-left, but map[0,0] is top-left
            iy = self.size - 1 - iy

            # Bounds check
            if 0 <= ix < self.size and 0 <= iy < self.size:
                self.map[iy, ix] = 255

    # ---------------------------------------------
    # Get map + pose
    # ---------------------------------------------
    def get_pose(self):
        return self.x, self.y, self.theta

    def get_map(self):
        return self.map
