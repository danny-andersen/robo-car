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

         # Cached views
        self.prob = None
        self.occ_mask = None
        # Distance field (meters)
        self.dist_field = None
        #Init internal maps
        self.update_internal_maps()
        self.first_scan_count = 10 # Number of initial scans to use for building up an initial map before we start doing ICP-based pose refinement, to give ICP a better chance of success when we do start using it for pose refinement
        
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
        """Recompute probability, occupancy mask, and distance field."""
        self.prob = 1.0 / (1.0 + np.exp(-self.L))
        self.occ_mask = self.prob >= self.p_occ
        # Distance field from occupancy grid
        self.dist_field = distance_transform_edt(~self.occ_mask) * (self.resolution_m * 1000)
        
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
    
    def relocalise_translation(self, scan_xy, occ_img, theta, guess_xy_mm,
                            search_radius_mm=800, step_mm=40):
        """
        scan_points: list/array of (range_mm, angle_rad) or (x_mm, y_mm) in robot frame
        occ_img: occupancy uint8 (0=occ, 255=free, mid=unknown)
        theta: world-frame heading (rad), trusted from IMU
        guess_xy_mm: (x_mm, y_mm) last known pose
        """

        best_score = -1e18
        best_x, best_y = guess_xy_mm

        # Search a window around the last pose
        for dx_mm in range(-search_radius_mm, search_radius_mm + 1, step_mm):
            for dy_mm in range(-search_radius_mm, search_radius_mm + 1, step_mm):
                x_mm = guess_xy_mm[0] + dx_mm
                y_mm = guess_xy_mm[1] + dy_mm

                score = self.score_scan_at_pose(scan_xy, x_mm, y_mm, theta)

                if score > best_score:
                    best_score = score
                    best_x, best_y = x_mm, y_mm

        return best_x, best_y

    def global_relocalise(self, scan_xy, occ_img, imu_theta, last_good_xy):
        best = None
        best_score = -1e18

        for dtheta in np.linspace(-math.radians(20), math.radians(20), 9):
            theta = imu_theta + dtheta

            for dx_mm in range(-1200, 1201, 80):
                for dy_mm in range(-1200, 1201, 80):
                    x = last_good_xy[0] + dx_mm
                    y = last_good_xy[1] + dy_mm

                    score = self.score_scan_at_pose(scan_xy, occ_img, x, y, theta)

                    if score > best_score:
                        best_score = score
                        best = (x, y, theta)

        return best  # (x, y, theta)

    # ---------------------------------------------------------
    # Main update: scan + IMU heading (degrees, 0°=north, CW+)
    # ---------------------------------------------------------
    def update(self, scan_mm, heading_deg, avg_move_heading_deg, distance_mm, move_confidence=1.0):
        pts = self.scan_to_points(scan_mm)
        if pts.shape[0] == 0:
            return

        # print("Raw compass:", heading_deg)
        # Convert compass heading to SLAM frame: 0°=north, CCW+
        bearing_deg = 90.0 - heading_deg
        avg_move_bearing_deg = 90.0 - avg_move_heading_deg
        self.theta = math.radians(bearing_deg)
        theta_pred = self.theta

        if self.first_scan_count == 0:
            # Skip pose update for the first few scans to allow map to build up, as ICP is likely to fail when the map is very sparse and not very accurate yet
            relocalise = False
            # if distance_mm > 0:
                # Robot has moved since last scan - predict new pose based on heading + distance
            dx = move_confidence * distance_mm * math.cos(math.radians(avg_move_bearing_deg))
            dy = move_confidence * distance_mm * math.sin(math.radians(avg_move_bearing_deg))

            x_pred = self.x + dx
            y_pred = self.y + dy

            # if move_confidence > 0.2:
            # Try ICP refinement of the predicted pose based on the new scan and current map, to correct for any drift in heading or position during the drive. We can trust ICP more when we have high confidence in movement, and less when we have low confidence in movement (e.g. due to wheel slip), by adjusting the ICP error threshold for accepting ICP results accordingly.
            x_icp, y_icp, theta_icp, icp_error = self.icp_scan_to_map( pts, (x_pred, y_pred, theta_pred), move_confidence )
            icp_threshold = self.icp_good_threshold * (0.5 + 0.5 * move_confidence)  # Adjust ICP threshold based on move confidence (lower confidence → be more lenient with ICP)
        
            if icp_error < icp_threshold: 
                # Normal case 
                self.x, self.y, self.theta = x_icp, y_icp, theta_icp 
            else:
                # ICP failed → global relocalisation (translation only) 
                if move_confidence < 0.5:
                    print("ICP failed with error", icp_error, "exceeding threshold", icp_threshold, "- doing relocalisation with low confidence in movement:", move_confidence)
                    relocalise = True
                else:
                    #We were confident in the move so this must be a dodgy scan - ignore it and keep previous pose, rather than risk a bad relocalisation based on a bad scan when we have reasonably good confidence in movement since last scan
                    print("Dodgy SCAN - ICP failed with error", icp_error, "exceeding threshold", icp_threshold, "- ignoring scan and keeping previous pose with reasonably good confidence in movement:", move_confidence)
                    return
            # else:
            #     # Force a relocalisation when we have no confidence in movement, as the pose guess is likely to be very inaccurate
            #     relocalise = True

            if relocalise:
                if distance_mm > 500:
                    print("Performing global relocalisation with no confidence in movement:", move_confidence)
                    best_x, best_y, best_theta = self.global_relocalise(pts, self.L, theta_pred, (self.x, self.y))
                else:
                    print("Performing local relocalisation around predicted pose with low confidence in movement:", move_confidence)
                    search_radius = 300  + int(2.0 * distance_mm)  # Increase search radius by the distance travelled since last scan, to account for increased uncertainty in pose
                    best_x, best_y = self.relocalise_translation( pts, self.L, theta_pred, (x_pred, y_pred), search_radius_mm=search_radius, step_mm=40 ) 
                    best_theta = theta_pred
                # Final ICP refine after relocalisation
                x_ref, y_ref, theta_ref, _ = self.icp_scan_to_map(
                    pts, (best_x, best_y, best_theta), 0.0)
                self.x, self.y, self.theta = x_ref, y_ref, theta_ref
            # else:
            #     # print("Robot is stationary - performing ICP-based pose refinement with high confidence in heading and no movement information")
            #     (new_x, new_y, new_th), old_score, new_score = \
            #         self.refine_pose_scan_to_map(pts)
            #     if new_score > old_score:
            #         self.x = new_x
            #         self.y = new_y
            #         self.theta = new_th

        else:
            self.first_scan_count -= 1    
        # Update map with scan at refined pose
        self.update_map(pts)
        self.update_internal_maps()

