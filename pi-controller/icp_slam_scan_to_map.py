import math
import numpy as np
from scipy.ndimage import distance_transform_edt

import config


def bresenham(x0, y0, x1, y1):
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

# ------------------------------------------------------------
# Local micro-map structure
# ------------------------------------------------------------

class LocalMap:
    def __init__(self, size_m=4.0, resolution_m=0.02):
        self.resolution_m = resolution_m
        self.size_m = size_m
        self.pixels = int(size_m / resolution_m)
        self.L = np.zeros((self.pixels, self.pixels), dtype=np.float32)

    def world_from_local(self, lx_mm, ly_mm, pose):
        """Transform local (mm) to world (mm) using pose (x,y,theta)."""
        x, y, th = pose
        c = math.cos(th)
        s = math.sin(th)
        wx = x + c * lx_mm - s * ly_mm
        wy = y + s * lx_mm + c * ly_mm
        return wx, wy

    def cell_from_local_mm(self, lx_mm, ly_mm):
        """Local mm → local grid indices."""
        cx = int(lx_mm / (self.resolution_m * 1000) + self.pixels / 2)
        cy = int(self.pixels / 2 - ly_mm / (self.resolution_m * 1000))
        return cx, cy




class ICP_SLAM:
    def __init__(self, map_size_m=6.0, resolution=0.02):
        """
        map_size_m: physical width/height of map in meters
        resolution: meters per cell
        """
        self.resolution_m = resolution
        self.map_pixels = int(map_size_m / resolution)  # pixels per side
        
        # Log-odds grid
        self.L = np.zeros((self.map_pixels, self.map_pixels),dtype=np.float32)
        # self.map = np.zeros((self.map_pixels, self.map_pixels), dtype=np.uint8)
        
        # Scan log for offline viewer
        # self.scan_log = []

        # Pose in world frame (mm, mm, rad)
        map_width_mm = (self.map_pixels * self.resolution_m) * 1000.0
        self.x = map_width_mm / 2.0
        self.y = map_width_mm / 2.0
        self.theta = 0.0
        self.theta_offset = None
         # Thresholds
        self.p_occ = 0.7
        self.p_free = 0.3
        self.icp_good_threshold = 5000.0  # ICP error threshold for deciding when to trust ICP vs do global relocalisation
        self.L_min = -8.0
        self.L_max = 8.0

         # Cached views
        self.prob = None
        self.occ_mask = None
        # Distance field (meters)
        self.dist_field = None
        #Init internal maps
        self.update_internal_maps()
        self.first_scan = True 
        self.first_ever_scan = True
        self.scans_mm_stationary = []  # Store recent scans for building local micro-maps during stationary phases
        self.heading_deg = 0  # Initialize heading degree
        self.avg_move_heading_deg = 0  # Initialize average move heading degree
        self.distance_moved_mm = 0  # Initialize distance moved
        self.move_confidence = 0.0  # Initialize move confidence
        
    # ---------------------------------------------------------
    # Polar scan → Cartesian points in robot frame (mm)
    # ---------------------------------------------------------
    def scan_to_points(self, scan_mm):
        angles = np.deg2rad(np.arange(360) + config.LIDAR_OFFSET_DEG)
        angles = -angles  # Convert to CW+ from CCW+
        d = np.array(scan_mm, dtype=float)
        mask = (d > 50) & (d < 8000)
        if not np.any(mask):
            return np.zeros((0, 2))

        # Downsample to reduce noise 
        mask_idx = np.where(mask)[0][::2]
        d = d[mask_idx]
        a = angles[mask_idx]        
        
        x = d * np.cos(a)
        y = d * np.sin(a)
        return np.vstack((x, y)).T

    # ------------------------------------------------------------ 
    # Map update helpers
    # ------------------------------------------------------------
    def update_internal_maps(self):
        """
        Rebuild probability grid, occupancy mask, and distance field.
        Assumes log-odds are already clamped.
        """

        # Convert to probability safely
        prob = 1.0 / (1.0 + np.exp(-self.L))

        # Occupancy mask
        self.occ_mask = (prob >= self.p_occ)

        # Distance field (Euclidean)
        self.dist_field = distance_transform_edt(~self.occ_mask) * self.resolution_m
    
    # def update_internal_maps(self):
    #     """Recompute probability, occupancy mask, and distance field."""
    #     self.prob = 1.0 / (1.0 + np.exp(-self.L))
    #     self.occ_mask = self.prob >= self.p_occ
    #     # Distance field from occupancy grid
    #     self.dist_field = distance_transform_edt(~self.occ_mask) * (self.resolution_m * 1000)
        
    # ---------------------------------------------------------
    # Accessors
    # ---------------------------------------------------------
    def get_pose(self):
        return self.x, self.y, self.theta

    def get_map(self):
        p = 1.0 / (1.0 + np.exp(-self.L))
        return (p * 255).astype(np.uint8)
    
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
        ix = (x_m / self.resolution_m).astype(int)
        iy = (y_m / self.resolution_m).astype(int)
        iy = self.map_pixels - 1 - iy  # flip Y so +Y world is up in the image
        return ix, iy
    
    # # ---------------------------------------------------------
    # # Pose scoring using distance field (lower distance = better)
    # # ---------------------------------------------------------
    # def score_pose(self, pts, x_mm, y_mm, theta):
    #     if pts.shape[0] == 0:
    #         return -np.inf

    #     wx, wy = self.transform_points_world(pts, x_mm, y_mm, theta)
    #     ix, iy = self.world_to_grid(wx, wy)

    #     mask = (ix >= 0) & (ix < self.size) & (iy >= 0) & (iy < self.size)
    #     if not np.any(mask):
    #         return -np.inf

    #     ix = ix[mask]
    #     iy = iy[mask]

    #     # Negative sum of distances → higher score = better alignment
    #     return -np.sum(self.dist[iy, ix])

    # ------------------------------------------------------------
    # Scan scoring using distance field
    # ------------------------------------------------------------
    def score_scan_at_pose(self, scan_xy, x_mm, y_mm, theta):
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        score = 0.0
        h, w = self.dist_field.shape

        for sx, sy in scan_xy:
            wx = x_mm + cos_t * sx - sin_t * sy
            wy = y_mm + sin_t * sx + cos_t * sy

            gx = int(wx / (self.resolution_m * 1000))
            gy = int(self.map_pixels - 1 - (wy / (self.resolution_m * 1000)))

            if 0 <= gx < w and 0 <= gy < h:
                score -= self.dist_field[gy, gx]  # lower distance = better score

        return score
 
    # ---------------------------------------------------------
    # Scan-to-map refinement: x, y, theta
    # ---------------------------------------------------------
    def refine_pose_scan_to_map(self, pts):

        trans_step = 20.0
        rot_step = math.radians(0.5)

        trans_offsets = [-trans_step, 0.0, trans_step]
        rot_offsets = [k * rot_step for k in range(-2, 3)]  # ±1°

        base_score = self.score_scan_at_pose(pts, self.x, self.y, self.theta)
        best_score = base_score
        best_pose = (self.x, self.y, self.theta)

        for dx in trans_offsets:
            for dy in trans_offsets:
                for dth in rot_offsets:
                    cx = self.x + dx
                    cy = self.y + dy
                    cth = self.theta + dth

                    score = self.score_scan_at_pose(pts, cx, cy, cth)
                    if score > best_score:
                        best_score = score
                        best_pose = (cx, cy, cth)

        return best_pose, base_score, best_score


    # ------------------------------------------------------------
    # 1. Build local micro-map from stationary scans
    # ------------------------------------------------------------

    # def build_local_map_from_points(
    #         self,
    #         pose_seed,
    #         size_m=4.0,
    #         resolution_m=0.02,
    #         occ_logodds=2.0,
    #         free_logodds=-0.5
    #     ):
    #     """
    #     Build a local micro-map using the SAME logic as update_map():
    #     - robot at center cell
    #     - pts in robot frame (x=forward, y=right)
    #     - use transform_points_world
    #     - use world_to_grid
    #     - use Bresenham
    #     """

    #     local = LocalMap(size_m=size_m, resolution_m=resolution_m)
    #     cell_mm = resolution_m * 1000

    #     # Robot is at the center of the local map
    #     rx = local.pixels // 2
    #     ry = local.pixels // 2

    #     x0, y0, th0 = pose_seed

    #     for pts in self.scans_mm_stationary:

    #         # Convert robot-frame → world-frame using the SAME function as old pipeline
    #         wx, wy = self.transform_points_world(pts, x0, y0, th0)

    #         # Convert world → local grid using SAME logic as world_to_grid
    #         lx = (wx / 1000.0) / local.resolution_m
    #         ly = (wy / 1000.0) / local.resolution_m
    #         hx = lx.astype(int)
    #         hy = (local.pixels - 1 - ly).astype(int)

    #         for cx, cy in zip(hx, hy):
    #             if not (0 <= cx < local.pixels and 0 <= cy < local.pixels):
    #                 continue

    #             # Free cells
    #             for fx, fy in bresenham(rx, ry, cx, cy)[:-1]:
    #                 local.L[fy, fx] += free_logodds

    #             # Occupied cell
    #             local.L[cy, cx] += occ_logodds

    #         np.clip(local.L, self.L_min, self.L_max, out=local.L)

    #     return local

    def build_local_map_from_points(
            self,
            pose_seed,
            size_m=4.0,
            resolution_m=0.02,
            occ_logodds=2.0,
            free_logodds=-0.5
        ):
        local = LocalMap(size_m=size_m, resolution_m=resolution_m)

        # Robot is at the center of the local map
        rx = local.pixels // 2
        ry = local.pixels // 2

        for pts in self.scans_mm_stationary:
            # pts are in robot frame (x=forward, y=right)
            # local map is also robot-centric: no global pose, no theta
            cell_mm = local.resolution_m * 1000
            lx = pts[:, 0] / cell_mm
            ly = pts[:, 1] / cell_mm

            hx = (lx + local.pixels / 2).astype(int)
            hy = (local.pixels / 2 - ly).astype(int)

            for cx, cy in zip(hx, hy):
                if not (0 <= cx < local.pixels and 0 <= cy < local.pixels):
                    continue

                for fx, fy in bresenham(rx, ry, cx, cy)[:-1]:
                    local.L[fy, fx] += free_logodds

                local.L[cy, cx] += occ_logodds

            np.clip(local.L, self.L_min, self.L_max, out=local.L)

        return local

    # ------------------------------------------------------------
    # 2. Align local micro-map to global map using distance field
    # ------------------------------------------------------------

    def align_local_map_to_global(self, local_map, pose_guess,
                                trans_range_mm=200.0, trans_step_mm=20.0,
                                rot_range_deg=2.0, rot_step_deg=0.5):

        self.update_internal_maps()

        x0, y0, th0 = pose_guess
        best_score = -1e18
        best_pose = pose_guess

        # Extract occupied cells
        local_map.L = np.clip(local_map.L, -10.0, +10.0)
        prob_local = 1.0 / (1.0 + np.exp(-local_map.L))
        occ_mask_local = prob_local >= 0.7
        occ_indices = np.argwhere(occ_mask_local)

        if occ_indices.shape[0] == 0:
            return pose_guess

        # Local grid → local mm (same convention as cell_from_local_mm)
        cell_mm = local_map.resolution_m * 1000
        lx_mm = (occ_indices[:, 1] - local_map.pixels / 2) * cell_mm
        ly_mm = (local_map.pixels / 2 - occ_indices[:, 0]) * cell_mm

        # Search window
        dx_vals = np.arange(-trans_range_mm, trans_range_mm + 1e-3, trans_step_mm)
        dy_vals = np.arange(-trans_range_mm, trans_range_mm + 1e-3, trans_step_mm)
        dth_vals = np.arange(-rot_range_deg, rot_range_deg + 1e-3, rot_step_deg)

        h, w = self.dist_field.shape

        for dth_deg in dth_vals:
            th = th0 + math.radians(dth_deg)
            c = math.cos(th)
            s = math.sin(th)

            # local → world (same as transform_points_world)
            wx_base = x0 + (c * lx_mm - s * ly_mm)
            wy_base = y0 + (s * lx_mm + c * ly_mm)

            for dx in dx_vals:
                for dy in dy_vals:
                    wx = wx_base + dx
                    wy = wy_base + dy

                    gx, gy = self.world_to_grid(wx, wy)

                    valid = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
                    if not np.any(valid):
                        continue

                    df_vals = self.dist_field[gy[valid], gx[valid]]
                    score = -np.mean(df_vals)

                    if score > best_score:
                        best_score = score
                        best_pose = (x0 + dx, y0 + dy, th)

        return best_pose


    # ------------------------------------------------------------
    # 3. Merge local micro-map into global log-odds map
    # ------------------------------------------------------------

    def merge_local_map_into_global(self, local_map, pose):
        prob_local = 1.0 / (1.0 + np.exp(-local_map.L))
        occ_mask_local = (prob_local >= 0.5)

        ys, xs = np.nonzero(occ_mask_local)
        if xs.size == 0:
            return

        cell_mm_local = local_map.resolution_m * 1000

        # local grid → local mm
        lx_mm = (xs - local_map.pixels / 2) * cell_mm_local
        ly_mm = (local_map.pixels / 2 - ys) * cell_mm_local

        x, y, th = pose
        c = math.cos(th)
        s = math.sin(th)

        # local → world (identical to transform_points_world)
        wx = x + (c * lx_mm - s * ly_mm)
        wy = y + (s * lx_mm + c * ly_mm)

        # world → map grid (identical to world_to_grid)
        gx, gy = self.world_to_grid(wx, wy)

        valid = (gx >= 0) & (gx < self.map_pixels) & (gy >= 0) & (gy < self.map_pixels)
        gx = gx[valid]
        gy = gy[valid]
        ys = ys[valid]
        xs = xs[valid]

        self.L[gy, gx] += local_map.L[ys, xs]
        self.L = np.clip(self.L, self.L_min, self.L_max)


     # ---------------------------------------------------------
    # Map update at current pose (ray-cast free + mark hits)
    # ---------------------------------------------------------
    # def update_map(self, pts):
    #     if pts.shape[0] == 0:
    #         return

    #     # Robot position in grid coords
    #     rx, ry = self.world_to_grid(
    #         np.array([self.x]),
    #         np.array([self.y])
    #     )
    #     rx = int(rx[0])
    #     ry = int(ry[0])

    #     wx, wy = self.transform_points_world(pts, self.x, self.y, self.theta)
    #     ix, iy = self.world_to_grid(wx, wy)

    #     for hx, hy in zip(ix, iy):
    #         if not (0 <= hx < self.size and 0 <= hy < self.size):
    #             continue

    #         # Ray from robot to hit
    #         ray = bresenham(rx, ry, hx, hy)

    #         # Free cells along the ray (except the last)
    #         for cx, cy in ray[:-1]:
    #             if 0 <= cx < self.size and 0 <= cy < self.size:
    #                 if self.map[cy, cx] == config.UNKNOWN:
    #                     self.map[cy, cx] = config.FREE

    #         # Hit cell = occupied
    #         self.map[hy, hx] = config.OCCUPIED

    # --------------------------------------------------------- 
    # Log-odds map update 
    # --------------------------------------------------------- 
    def update_map(self, pts):
        rx, ry = self.world_to_grid(np.array([self.x]), np.array([self.y]))
        rx, ry = int(rx[0]), int(ry[0])
        wx, wy = self.transform_points_world(pts, self.x, self.y, self.theta)
        ix, iy = self.world_to_grid(wx, wy)
        for hx, hy in zip(ix, iy): 
            if not (0 <= hx < self.map_pixels and 0 <= hy < self.map_pixels):
                continue
            # Free cells
            for cx, cy in bresenham(rx, ry, hx, hy)[:-1]:
                self.L[cy, cx] += config.L_FREE 
                # Occupied cell
                self.L[hy, hx] += config.L_OCC
        np.clip(self.L, config.L_MIN, config.L_MAX, out=self.L)
        
    def angle_diff(self, a, b):
        d = a - b
        return (d + math.pi) % (2 * math.pi) - math.pi


    def convert_bearing_from_slam_frame(self, bearing_deg):
        # Convert compass bearing (0°=north, CW+) from SLAM frame (0°=east, CCW+)
        # Adjust for theta_offset to keep SLAM world frame locked to initial robot orientation
        if (bearing_deg < 0):
            bearing_deg += 360.0
        return bearing_deg % 360.0
    
    # ------------------------------------------------------------
    # Translation-only ICP using distance field
    # ------------------------------------------------------------
    def icp_scan_to_map(self, scan_xy, pose_guess, move_conf, max_iters=20):
        x, y, theta = pose_guess

        last_error = 1e18

        for _ in range(max_iters):
            A = 0.0
            B = 0.0
            C = 0.0
            count = 0

            cos_t = math.cos(theta)
            sin_t = math.sin(theta)

            for sx, sy in scan_xy:
                wx = x + cos_t * sx - sin_t * sy
                wy = y + sin_t * sx + cos_t * sy

                gx = int(wx / (self.resolution_m * 1000))
                gy = int(self.map_pixels - 1 - (wy / (self.resolution_m * 1000)))

                if 0 <= gx < self.map_pixels and 0 <= gy < self.map_pixels:
                    d = self.dist_field[gy, gx]
                    C += d * d
                    count += 1

                    # Gradient approximation: move opposite the distance gradient
                    # (simple finite difference)
                    if gx > 0 and gx < self.map_pixels - 1:
                        dx = self.dist_field[gy, gx+1] - self.dist_field[gy, gx-1]
                    else:
                        dx = 0

                    if gy > 0 and gy < self.map_pixels - 1:
                        dy = self.dist_field[gy+1, gx] - self.dist_field[gy-1, gx]
                    else:
                        dy = 0

                    A -= dx
                    B -= dy

            if count == 0:
                return x, y, theta, 1e18

            # Confidence-weighted correction
            dx_corr = (A / count) * (1.0 - move_conf)
            dy_corr = (B / count) * (1.0 - move_conf)
            error = C / count

            x += dx_corr
            y += dy_corr

            if abs(last_error - error) < 1e-3:
                break

            last_error = error

        return x, y, theta, error
    
    # def relocalise_translation(self, scan_xy, occ_img, theta, guess_xy_mm,
    #                         search_radius_mm=800, step_mm=40):
    #     """
    #     scan_points: list/array of (range_mm, angle_rad) or (x_mm, y_mm) in robot frame
    #     occ_img: occupancy uint8 (0=occ, 255=free, mid=unknown)
    #     theta: world-frame heading (rad), trusted from IMU
    #     guess_xy_mm: (x_mm, y_mm) last known pose
    #     """

    #     best_score = -1e18
    #     best_x, best_y = guess_xy_mm

    #     # Search a window around the last pose
    #     for dx_mm in range(-search_radius_mm, search_radius_mm + 1, step_mm):
    #         for dy_mm in range(-search_radius_mm, search_radius_mm + 1, step_mm):
    #             x_mm = guess_xy_mm[0] + dx_mm
    #             y_mm = guess_xy_mm[1] + dy_mm

    #             score = self.score_scan_at_pose(scan_xy, x_mm, y_mm, theta)

    #             if score > best_score:
    #                 best_score = score
    #                 best_x, best_y = x_mm, y_mm

    #     return best_x, best_y

    def local_relocalise(self, pts, pose_guess,
                        trans_range_mm=200.0,
                        trans_step_mm=20.0,
                        rot_range_deg=3.0,
                        rot_step_deg=0.5):
        """
        Local relocalisation around pose_guess using distance-field scoring.
        pts: Nx2 scan points in robot frame (mm)
        pose_guess: (x_mm, y_mm, theta_rad)
        """

        self.update_internal_maps()  # ensure dist_field is fresh

        x0, y0, th0 = pose_guess
        best_score = -1e18
        best_pose = pose_guess

        # Precompute scan points in mm
        px = pts[:, 0]
        py = pts[:, 1]

        # Global map info
        h, w = self.dist_field.shape
        cell_mm = self.resolution_m * 1000

        # Search windows
        dx_vals = np.arange(-trans_range_mm, trans_range_mm + 1e-6, trans_step_mm)
        dy_vals = np.arange(-trans_range_mm, trans_range_mm + 1e-6, trans_step_mm)
        dth_vals = np.arange(-rot_range_deg, rot_range_deg + 1e-6, rot_step_deg)

        for dth_deg in dth_vals:
            th = th0 + math.radians(dth_deg)
            c = math.cos(th)
            s = math.sin(th)

            # Rotate scan points once per angle
            wx_base = c * px - s * py
            wy_base = s * px + c * py

            for dx in dx_vals:
                for dy in dy_vals:
                    wx = x0 + dx + wx_base
                    wy = y0 + dy + wy_base

                    gx = (wx / cell_mm).astype(np.int32)
                    gy = (self.map_pixels - 1 - (wy / cell_mm)).astype(np.int32)

                    valid = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
                    if not np.any(valid):
                        continue

                    df_vals = self.dist_field[gy[valid], gx[valid]]
                    score = -np.mean(df_vals)

                    if score > best_score:
                        best_score = score
                        best_pose = (x0 + dx, y0 + dy, th)

        return (*best_pose, best_score)

    # def global_relocalise(self, scan_xy, occ_img, imu_theta, last_good_xy):
    #     best = None
    #     best_score = -1e18

    #     for dtheta in np.linspace(-math.radians(20), math.radians(20), 9):
    #         theta = imu_theta + dtheta

    #         for dx_mm in range(-1200, 1201, 80):
    #             for dy_mm in range(-1200, 1201, 80):
    #                 x = last_good_xy[0] + dx_mm
    #                 y = last_good_xy[1] + dy_mm

    #                 score = self.score_scan_at_pose(scan_xy, occ_img, x, y, theta)

    #                 if score > best_score:
    #                     best_score = score
    #                     best = (x, y, theta)

    #     return best  # (x, y, theta)

    def global_relocalise(self, pts,
                        rot_step_deg=2.0,
                        trans_step_cells=10):
        """
        Global relocalisation using distance-field scoring.
        Searches the entire map at coarse resolution.
        pts: Nx2 scan points in robot frame (mm)
        """

        self.update_internal_maps()

        h, w = self.dist_field.shape
        cell_mm = self.resolution_m * 1000

        px = pts[:, 0]
        py = pts[:, 1]

        best_score = -1e18
        best_pose = (self.x, self.y, self.theta)

        # Search headings 0..360
        for th_deg in range(0, 360, int(rot_step_deg)):
            th = math.radians(th_deg)
            c = math.cos(th)
            s = math.sin(th)

            # Rotate scan once per heading
            wx_base = c * px - s * py
            wy_base = s * px + c * py

            # Search translation over entire map
            for gy in range(0, h, trans_step_cells):
                for gx in range(0, w, trans_step_cells):

                    # Convert grid cell to world mm
                    wx = gx * cell_mm
                    wy = (h - 1 - gy) * cell_mm

                    # Apply translation
                    wx_pts = wx + wx_base
                    wy_pts = wy + wy_base

                    gx_pts = (wx_pts / cell_mm).astype(np.int32)
                    gy_pts = (h - 1 - (wy_pts / cell_mm)).astype(np.int32)

                    valid = (gx_pts >= 0) & (gx_pts < w) & (gy_pts >= 0) & (gy_pts < h)
                    if not np.any(valid):
                        continue

                    df_vals = self.dist_field[gy_pts[valid], gx_pts[valid]]
                    score = -np.mean(df_vals)

                    if score > best_score:
                        best_score = score
                        best_pose = (wx, wy, th)

        return (*best_pose, best_score)

    def refine_pose_with_micro_map_and_merge(self):
        """
        Full stationary refinement pipeline:

        1. Motion prior from odometry + move_confidence + IMU heading
        2. ICP refinement (if move_confidence high enough)
        - if ICP fails → local/global relocalisation
        3. Build local micro-map from stationary scans using refined pose
        4. Align micro-map to global map (distance-field based)
        5. Merge micro-map into global log-odds map
        6. Update internal map structures (prob, dist_field, etc.)
        """


        if len(self.scans_mm_stationary) == 0:
            print("No scans to process for micro-map refinement.")
            return

        print("Number of scans in stationary buffer:", len(self.scans_mm_stationary))
        
        if not self.first_ever_scan:
            # We cannot do any of this on the very first scan since we have no prior pose or map, so just skip to building the initial map. After the first scan, we have a pose guess and can start refining with micro-maps.
            print("Refining pose with micro-map and merging into global map...")
            # ------------------------------------------------------------
            # 1. Motion prior
            # ------------------------------------------------------------

            dx = self.move_confidence * self.distance_moved_mm * math.cos(math.radians(self.avg_move_heading_deg))
            dy = self.move_confidence * self.distance_moved_mm * math.sin(math.radians(self.avg_move_heading_deg))

            x_pred = self.x + dx
            y_pred = self.y + dy
            pose_guess = (x_pred, y_pred, self.theta)

            # First scan points (for ICP / relocalisation)
            first_scan_pts = self.scans_mm_stationary[0]

            # ------------------------------------------------------------
            # 2. ICP refinement + fallback relocalisation
            # ------------------------------------------------------------
            pose_seed = pose_guess
            print("Pose guess from motion prior:", pose_guess, "with move confidence:", self.move_confidence)

            relocalise = False
            if self.move_confidence > 0.3:
                # Try ICP around motion prior
                x_icp, y_icp, theta_icp, icp_error = self.icp_scan_to_map(
                    first_scan_pts,
                    pose_guess,
                    self.move_confidence
                )

                icp_threshold = self.icp_good_threshold * (0.5 + 0.5 * self.move_confidence)

                if icp_error < icp_threshold:
                    # ICP succeeded → use it as seed
                    print("ICP refinement successful with error:", icp_error)
                    pose_seed = (x_icp, y_icp, theta_icp)
                else:
                    # ICP failed → fall back to relocalisation
                    print("ICP refinement failed with error:", icp_error, "exceeding threshold:", icp_threshold)
                    relocalise = True
            else:
                relocalise = True
        
            if relocalise:        
                # Low confidence in pose guess → skip ICP, go straight to relocalisation
                if self.distance_moved_mm < 500.0:
                    print("Performing local relocalisation around predicted pose with in movement:", self.move_confidence)
                    x_rel, y_rel, th_rel, score_rel = self.local_relocalise(
                        first_scan_pts,
                        pose_guess
                    )
                else:
                    print("Performing global relocalisation with no confidence in movement:", self.move_confidence)
                    x_rel, y_rel, th_rel, score_rel = self.global_relocalise(
                        first_scan_pts
                    )
                    pose_seed = (x_rel, y_rel, th_rel)

            # At this point, pose_seed is our best estimate before micro-map
        else:
            pose_seed = (self.x, self.y, self.theta)
            self.first_ever_scan = False
            print("First ever scan - using initial pose guess without refinement:", pose_seed)
            
        # ------------------------------------------------------------
        # 3. Build local micro-map from stationary scans
        # ------------------------------------------------------------
        local_map = self.build_local_map_from_points(
            pose_seed,
            size_m=4.0,
            resolution_m=self.resolution_m
        )

        # ------------------------------------------------------------
        # 4. Align local micro-map to global map
        # ------------------------------------------------------------
        refined_pose = self.align_local_map_to_global(
            local_map,
            pose_seed,
            trans_range_mm=200.0,
            trans_step_mm=20.0,
            rot_range_deg=2.0,
            rot_step_deg=0.5
        )

        self.x, self.y, self.theta = refined_pose

        # ------------------------------------------------------------
        # 5. Merge local micro-map into global log-odds map
        # ------------------------------------------------------------
        self.merge_local_map_into_global(local_map, refined_pose)

        # ------------------------------------------------------------
        # 6. Rebuild probability, occupancy mask, distance field, etc.
        # ------------------------------------------------------------
        self.update_internal_maps()
        
        self.scans_mm_stationary.clear()  # Clear stored scans after processing
        self.first_scan = True  # Reset for next stationary phase

    def update(self, scan_mm, heading_deg, avg_move_heading_deg, distance_mm, move_confidence=1.0):
        pts = self.scan_to_points(scan_mm)
        if pts.shape[0] == 0:
            return

        # Store scan for building local micro-maps during stationary phases
        self.scans_mm_stationary.append(pts)
        if self.first_scan:
            self.first_scan = False
            self.distance_moved_mm = distance_mm
            # Convert compass heading to SLAM frame: 0°=north, CCW+
            self.heading_deg = 90.0 - heading_deg
            self.theta = math.radians(self.heading_deg)
            self.avg_move_heading_deg = 90.0 - avg_move_heading_deg
            self.move_confidence = move_confidence

    # ---------------------------------------------------------
    # Main update: scan + IMU heading (degrees, 0°=north, CW+)
    # ---------------------------------------------------------
    # def update(self, scan_mm, heading_deg, avg_move_heading_deg, distance_mm, move_confidence=1.0):
    #     pts = self.scan_to_points(scan_mm)
    #     if pts.shape[0] == 0:
    #         return

    #     # print("Raw compass:", heading_deg)
    #     # Convert compass heading to SLAM frame: 0°=north, CCW+
    #     bearing_deg = 90.0 - heading_deg
    #     avg_move_bearing_deg = 90.0 - avg_move_heading_deg
    #     self.theta = math.radians(bearing_deg)
    #     theta_pred = self.theta

    #     if self.first_scan_count == 0:
    #         # Skip pose update for the first few scans to allow map to build up, as ICP is likely to fail when the map is very sparse and not very accurate yet
    #         relocalise = False
    #         # if distance_mm > 0:
    #             # Robot has moved since last scan - predict new pose based on heading + distance
    #         dx = move_confidence * distance_mm * math.cos(math.radians(avg_move_bearing_deg))
    #         dy = move_confidence * distance_mm * math.sin(math.radians(avg_move_bearing_deg))

    #         x_pred = self.x + dx
    #         y_pred = self.y + dy

    #         # if move_confidence > 0.2:
    #         # Try ICP refinement of the predicted pose based on the new scan and current map, to correct for any drift in heading or position during the drive. We can trust ICP more when we have high confidence in movement, and less when we have low confidence in movement (e.g. due to wheel slip), by adjusting the ICP error threshold for accepting ICP results accordingly.
    #         x_icp, y_icp, theta_icp, icp_error = self.icp_scan_to_map( pts, (x_pred, y_pred, theta_pred), move_confidence )
    #         icp_threshold = self.icp_good_threshold * (0.5 + 0.5 * move_confidence)  # Adjust ICP threshold based on move confidence (lower confidence → be more lenient with ICP)
        
    #         if icp_error < icp_threshold: 
    #             # Normal case 
    #             self.x, self.y, self.theta = x_icp, y_icp, theta_icp 
    #         else:
    #             # ICP failed → global relocalisation (translation only) 
    #             if move_confidence < 0.5:
    #                 print("ICP failed with error", icp_error, "exceeding threshold", icp_threshold, "- doing relocalisation with low confidence in movement:", move_confidence)
    #                 relocalise = True
    #             else:
    #                 #We were confident in the move so this must be a dodgy scan - ignore it and keep previous pose, rather than risk a bad relocalisation based on a bad scan when we have reasonably good confidence in movement since last scan
    #                 print("Dodgy SCAN - ICP failed with error", icp_error, "exceeding threshold", icp_threshold, "- ignoring scan and keeping previous pose with reasonably good confidence in movement:", move_confidence)
    #                 return
    #         # else:
    #         #     # Force a relocalisation when we have no confidence in movement, as the pose guess is likely to be very inaccurate
    #         #     relocalise = True

    #         if relocalise:
    #             if distance_mm > 500:
    #                 print("Performing global relocalisation with no confidence in movement:", move_confidence)
    #                 best_x, best_y, best_theta = self.global_relocalise(pts, self.L, theta_pred, (self.x, self.y))
    #             else:
    #                 print("Performing local relocalisation around predicted pose with low confidence in movement:", move_confidence)
    #                 search_radius = 300  + int(2.0 * distance_mm)  # Increase search radius by the distance travelled since last scan, to account for increased uncertainty in pose
    #                 best_x, best_y = self.relocalise_translation( pts, self.L, theta_pred, (x_pred, y_pred), search_radius_mm=search_radius, step_mm=40 ) 
    #                 best_theta = theta_pred
    #             # Final ICP refine after relocalisation
    #             x_ref, y_ref, theta_ref, _ = self.icp_scan_to_map(
    #                 pts, (best_x, best_y, best_theta), 0.0)
    #             self.x, self.y, self.theta = x_ref, y_ref, theta_ref
    #         # else:
    #         #     # print("Robot is stationary - performing ICP-based pose refinement with high confidence in heading and no movement information")
    #         #     (new_x, new_y, new_th), old_score, new_score = \
    #         #         self.refine_pose_scan_to_map(pts)
    #         #     if new_score > old_score:
    #         #         self.x = new_x
    #         #         self.y = new_y
    #         #         self.theta = new_th

    #     else:
    #         self.first_scan_count -= 1    
    #     # Update map with scan at refined pose
    #     self.update_map(pts)
    #     self.update_internal_maps()

