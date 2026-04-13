from datetime import datetime
import math
import os
import numpy as np
from scipy.ndimage import distance_transform_edt, binary_closing, binary_dilation

import config


# ------------------------------------------------------------
# Local micro-map structure
# ------------------------------------------------------------

class LocalMap:
    def __init__(self, size_m):
        self.resolution_m = config.map_resolution_m
        self.size_m = size_m
        self.pixels = int(size_m / self.resolution_m)
        self.L = np.zeros((self.pixels, self.pixels), dtype=np.float32)
        self.slam_scores = (0, 0 ,0)


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
    def __init__(self):
        """
        map_size_m: physical width/height of map in meters
        resolution: meters per cell
        """
        self.init_data()
        self.scans_mm_stationary = []  # Store recent scans for building local micro-maps during stationary phases
        self._ray_cache = {}

    
    def init_data(self):
        self.map_size_m = config.map_size
        self.resolution_m = config.map_resolution_m
        self.map_pixels = int(self.map_size_m / self.resolution_m)  # pixels per side
        # print(f"ICP SLAM map pixels {self.map_pixels}")
        
        # Load pre-generated Log-odds grid
        if os.path.isfile("./global_logodds.npy"):
            print(f"{datetime.now()}: Loading Log odds global map")
            self.L = np.load("./global_logodds.npy")
            self.first_ever_scan = False
        else:
            print(f"{datetime.now()}: Log odds global map doesnt exist - starting with blank map")
            self.L = np.zeros((self.map_pixels, self.map_pixels),dtype=np.float32)
            self.first_ever_scan = True
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
        self.icp_good_threshold = 0.5  # ICP error threshold for deciding when to trust ICP vs failing

         # Cached views
        self.prob = None
        self.occ_mask = None
        # Distance field (meters)
        self.dist_field = None
        #Init internal maps
        self.update_internal_maps()
        self.heading_deg = 0  # Initialize heading degree
        self.avg_move_heading_deg = 0  # Initialize average move heading degree
        self.distance_moved_mm = 0  # Initialize distance moved
        self.move_confidence = 0.0  # Initialize move confidence
        self.lost = False  #Whether we are a bit lost on the global map (localisation failed)
        self.slam_scores = [0, 0, 0, 0, 0]
        
    
    def clean_logodds(self, logodds):
        prob = 1 / (1 + np.exp(-logodds))
        occ = prob >= 0.55       # strong occupancy
        # free = prob <= 0.44      # strong free
        # unknown = ~(occ | free)

        # occ_clean = binary_dilation(occ_clean, iterations=1)        
        occ = binary_dilation(occ, iterations=1)        
        occ = binary_closing(occ, structure=np.ones((2,2)))
        # occ = binary_dilation(occ, iterations=1)        
    
        dist = distance_transform_edt(~occ)
        dist_free = distance_transform_edt(occ)
        
        occ_clean = dist <= 1.0
        free_clean = dist_free <= 1.0
        unknown = ~(occ_clean | free_clean)

        L_clean = np.zeros_like(logodds)
        L_clean[occ_clean] = +2.0     # strong occupied
        L_clean[free_clean] = -2.0    # strong free
        L_clean[unknown] = 0.0        # unknown
        
        return L_clean
    
    # Create Bresenham ray cache    
    def get_ray(self, rx, ry, cx, cy):
        key = (cx - rx, cy - ry)

        # If cached, return it
        if key in self._ray_cache:
            return self._ray_cache[key]

        # Compute ray (may be a generator)
        ray = config.bresenham(rx, ry, cx, cy)

        # Convert generator → list safely
        ray_list = list(ray) if ray is not None else []

        # Cache empty rays too (so we don't recompute)
        self._ray_cache[key] = ray_list
        return ray_list            
    
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
        self.occ_mask = (prob >= config.occ_threshold)

        occ_ratio = np.mean(self.occ_mask)
        # print("Occupied ratio:", occ_ratio)

        # Distance field (Euclidean)
        self.dist_field = distance_transform_edt(~self.occ_mask) * self.resolution_m
    
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
        c = np.cos(theta)
        s = np.sin(theta)

        lx = pts[:, 0]
        ly = pts[:, 1]

        gx_mm = x_mm + c * lx - s * ly
        gy_mm = y_mm + s * lx + c * ly        
        return gx_mm, gy_mm

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
 
    def build_local_map_from_points(
            self,
            pose_seed,
            size_m=4.0,
            resolution_m=0.02,
            occ_logodds=2.0,
            free_logodds=-0.5
        ):
        local = LocalMap(size_m=size_m)

        # Robot is at the center of the local map
        rx = local.pixels // 2
        ry = local.pixels // 2
        cell_mm = local.resolution_m * 1000

        free_x = []
        free_y = []
        for pts in self.scans_mm_stationary:
            # pts are in robotn frame (x=forward, y=right)
            # local map is also robot-centric: no global pose, no theta
            lx = pts[:, 0] / cell_mm
            ly = pts[:, 1] / cell_mm

            hx = (lx + local.pixels / 2).astype(int)
            hy = (local.pixels / 2 - ly).astype(int)

            # for cx, cy in zip(hx, hy):
            #     if not (0 <= cx < local.pixels and 0 <= cy < local.pixels):
            #         continue

            #     for fx, fy in config.bresenham(rx, ry, cx, cy):
            #         local.L[fy, fx] += free_logodds

            #     local.L[cy, cx] += occ_logodds

            # np.clip(local.L, config.L_MIN, config.L_MAX, out=local.L)

            inb = (hx >= 0) & (hx < local.pixels) & (hy >= 0) & (hy < local.pixels)
            hx = hx[inb]
            hy = hy[inb]

            # Occupied cells in one shot
            local.L[hy, hx] += occ_logodds

            # Free cells: collect all ray cells, then apply in one vectorised update
            for cx, cy in zip(hx, hy):
                ray = self.get_ray(rx, ry, cx, cy)
                if not ray:
                    continue
                fx, fy = zip(*ray)
                free_x.extend(fx)
                free_y.extend(fy)
                
        if free_x:
            local.L[free_y, free_x] += free_logodds

        # Single clip at the end, not per scan
        np.clip(local.L, config.L_MIN, config.L_MAX, out=local.L)
        return local

        return local

    # ------------------------------------------------------------
    # 2. Align local micro-map to global map using distance field
    # ------------------------------------------------------------

    def score_rotation_only(self, local_pts_mm, pose, theta_rad):
        x_mm, y_mm, _ = pose
        h, w = self.dist_field.shape
        cell_mm = self.resolution_m * 1000

        px = local_pts_mm[:, 0]
        py = local_pts_mm[:, 1]

        c = math.cos(theta_rad)
        s = math.sin(theta_rad)
        wx = x_mm + c * px - s * py
        wy = y_mm + s * px + c * py

        gx = (wx / cell_mm).astype(np.int32)
        gy = (h - 1 - (wy / cell_mm)).astype(np.int32)

        valid = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
        if not np.any(valid):
            return -1e18

        df_vals = self.dist_field[gy[valid], gx[valid]]
        return -np.mean(df_vals)

    def rotation_search(self, local_pts_mm, pose, window_deg, step_deg):
        x_mm, y_mm, theta_guess = pose
        theta_guess_deg = math.degrees(theta_guess)

        candidates = []
        for k in range(-int(window_deg/step_deg), int(window_deg/step_deg)+1):
            th_deg = (theta_guess_deg + k * step_deg) % 360
            th_rad = math.radians(th_deg)
            score = self.score_rotation_only(local_pts_mm, pose, th_rad)
            candidates.append((th_rad, score))

        best = max(candidates, key=lambda t: t[1])
        return best, candidates


    def rotation_ambiguous(self, candidates):
        # Sort by score
        sorted_scores = sorted(candidates, key=lambda t: t[1], reverse=True)
        best = sorted_scores[0][1]
        second = sorted_scores[1][1]

        # Normalise
        denom = max(abs(best), abs(second), 1e-9)
        diff = abs(best - second) / denom

        # Ambiguous if peak is not at least 2% better
        print(f"{datetime.now()}: Rotation ambiguous score: {diff}")
        return diff < 0.002

    def translation_refine(self, pts_mm, x_guess_mm, y_guess_mm, theta_rad,
                        win_mm=150, step_mm=20):

        best_x = x_guess_mm
        best_y = y_guess_mm
        best_score = -1e18

        offsets = np.arange(-win_mm, win_mm + 1e-6, step_mm)

        for dx in offsets:
            for dy in offsets:
                x = x_guess_mm + dx
                y = y_guess_mm + dy
                score = self.score_rotation_only(pts_mm, (x, y, theta_rad), theta_rad)
                if score > best_score:
                    best_score = score
                    best_x, best_y = x, y

        return best_x, best_y, best_score


    def score_rotation_only(self, local_pts_mm, pose, theta_rad):
        x_mm, y_mm, _ = pose
        h, w = self.dist_field.shape
        cell_mm = self.resolution_m * 1000

        px = local_pts_mm[:, 0]
        py = local_pts_mm[:, 1]

        c = math.cos(theta_rad)
        s = math.sin(theta_rad)
        wx = x_mm + c * px - s * py
        wy = y_mm + s * px + c * py

        gx = (wx / cell_mm).astype(np.int32)
        gy = (h - 1 - (wy / cell_mm)).astype(np.int32)

        valid = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
        if not np.any(valid):
            return -1e18

        df_vals = self.dist_field[gy[valid], gx[valid]]
        return -np.mean(df_vals)

    def align_map_pts_to_global(self, local_pts_mm, pose_guess):
        x_mm, y_mm, th_guess = pose_guess

        # --- ROTATION ONLY ---
        # Coarse
        (th1, sc1), c1 = self.rotation_search(local_pts_mm, pose_guess, 6.0, 1.0)

        # for th, sc in c1:
        #     print(f"{math.degrees(th):.1f}° → {sc:.6f}")

        # Ambiguity check
        if self.rotation_ambiguous(c1):
            # Fallback: rotation unreliable → do NOT align
            # print(f"Map alignment potentially wrong?? but ignoring")
            return (*pose_guess, sc1, False)

        # Medium
        (th2, sc2), c2 = self.rotation_search(local_pts_mm, (x_mm, y_mm, th1), 1.0, 0.25)
        # Fine
        (th_final, sc_final), c3 = self.rotation_search(local_pts_mm, (x_mm, y_mm, th2), 0.5, 0.1)

        # --- TRANSLATION REFINEMENT ---
        x_ref, y_ref, trans_score = self.translation_refine(
            local_pts_mm, x_mm, y_mm, th_final
        )

        final_score = min(sc_final, trans_score)
        return (x_ref, y_ref, th_final, final_score, True)


    def align_local_map_to_global(self, local_map, pose_guess,
                                trans_range_mm=200.0, trans_step_mm=20.0,
                                rot_range_deg=2.0, rot_step_deg=0.5):

        # self.update_internal_maps()

        x0, y0, th0 = pose_guess
        best_score = -1e18
        best_pose = pose_guess

        # Extract occupied cells
        local_map.L = np.clip(local_map.L, -10.0, +10.0)
        prob_local = 1.0 / (1.0 + np.exp(-local_map.L))
        occ_mask_local = prob_local >= config.occ_threshold
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

                    gx, gy = config.world_to_grid(wx, wy, self.map_pixels, self.resolution_m)

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
        gx, gy = config.world_to_grid(wx, wy, self.map_pixels)

        valid = (gx >= 0) & (gx < self.map_pixels) & (gy >= 0) & (gy < self.map_pixels)
        gx = gx[valid]
        gy = gy[valid]
        ys = ys[valid]
        xs = xs[valid]

        self.L[gy, gx] += local_map.L[ys, xs]
        self.L = np.clip(self.L, config.L_MIN, config.L_MAX)


    # --------------------------------------------------------- 
    # Log-odds map update 
    # --------------------------------------------------------- 
    def update_map(self, pts):
        rx, ry = config.world_to_grid(np.array([self.x]), np.array([self.y]), self.map_pixels)
        rx, ry = int(rx[0]), int(ry[0])
        wx, wy = self.transform_points_world(pts, self.x, self.y, self.theta, self.map_pixels)
        ix, iy = config.world_to_grid(wx, wy)
        for hx, hy in zip(ix, iy): 
            if not (0 <= hx < self.map_pixels and 0 <= hy < self.map_pixels):
                continue
            # Free cells
            for cx, cy in config.bresenham(rx, ry, hx, hy):
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
    
    def compute_confidence(self, overlap, mean_dist_m):
        # Overlap term: 0 at 0.1, 1 at 0.7
        o = np.clip((overlap - 0.1) / (0.7 - 0.1), 0.0, 1.0)
        # Distance term: 1 at 0.05m, 0 at 0.4m
        d = np.clip((0.4 - mean_dist_m) / (0.4 - 0.05), 0.0, 1.0)
        return float(np.sqrt(o * d))

    def local_relocalise(self, local_pts_mm,
                        pose_odom,
                        win_xy=1.0,
                        win_th_deg=12.0,
                        step_xy=0.10,
                        step_th_deg=3.0,
                        good_dist_m=0.15,
                        early_reject_dist_m=0.30,
                        early_reject_overlap=0.05,
                        use_coarse=True):
        """
        Fast local relocalisation around odom pose using a distance field.

        local_pts_mm: (N,2) local points in mm (robot frame)
        pose_odom: (x_mm, y_mm, th_rad)
        dist_field_m: HxW distance field in metres
        resolution_m: map resolution in metres
        """

        x0_mm, y0_mm, th0 = pose_odom

        # Optionally use a coarse distance field (2x downsample)
        if use_coarse:
            df = self.dist_field[::2, ::2]
            res_m = self.resolution_m * 2.0
        else:
            df = self.dist_field
            res_m = self.dist_field

        H, W = df.shape
        cell_mm = res_m * 1000.0

        # -----------------------------
        # 1. Build search grids
        # -----------------------------
        win_mm = win_xy * 1000.0

        xs = np.arange(-win_mm, win_mm + 1e-6, step_xy * 1000.0)
        ys = np.arange(-win_mm, win_mm + 1e-6, step_xy * 1000.0)

        th_range = np.deg2rad(win_th_deg)
        th_step = np.deg2rad(step_th_deg)
        dths = np.arange(-th_range, th_range + 1e-9, th_step)

        lx = local_pts_mm[:, 0]
        ly = local_pts_mm[:, 1]

        best_score = np.inf
        best_pose = pose_odom
        best_overlap = 0.0

        # -----------------------------
        # 2. Pre‑rotate local points for all θ
        # -----------------------------
        rotated_sets = []
        for dth in dths:
            th = th0 + dth
            c = np.cos(th)
            s = np.sin(th)
            gx_rot = c * lx - s * ly
            gy_rot = s * lx + c * ly
            rotated_sets.append((th, gx_rot, gy_rot))

        # -----------------------------
        # 3. Search over θ and (x,y)
        # -----------------------------
        for (th, gx_rot, gy_rot) in rotated_sets:
            # Pre‑compute global base (odom) in mm
            base_x = x0_mm + gx_rot
            base_y = y0_mm + gy_rot

            for dx in xs:
                gx_mm = base_x + dx
                gx = (gx_mm / cell_mm).astype(int)

                # Quick in‑bounds check on x
                valid_x = (gx >= 0) & (gx < W)
                if not np.any(valid_x):
                    continue

                for dy in ys:
                    gy_mm = base_y + dy
                    gy = (gy_mm / cell_mm).astype(int)

                    valid = valid_x & (gy >= 0) & (gy < H)
                    if not np.any(valid):
                        continue

                    df_vals = df[gy[valid], gx[valid]]  # metres

                    # Early reject: if almost nothing is within early_reject_dist_m
                    if np.mean(df_vals < early_reject_dist_m) < early_reject_overlap:
                        continue

                    mean_dist = float(np.mean(df_vals))
                    overlap = float(np.mean(df_vals < good_dist_m))

                    # Combined score: lower distance, higher overlap
                    score = mean_dist - 0.1 * overlap

                    if score < best_score:
                        best_score = score
                        best_pose = (x0_mm + dx, y0_mm + dy, th)
                        best_overlap = overlap

        # -----------------------------
        # 4. Confidence
        # -----------------------------
        confidence = self.compute_confidence(best_overlap, best_score)

        return best_pose, best_score, best_overlap, confidence


    def global_relocalise(self, pts, rot_step_deg=5.0, coarse=10):
        # self.update_internal_maps()

        h, w = self.dist_field.shape
        cell_mm = self.resolution_m * 1000

        px = pts[:, 0]
        py = pts[:, 1]

        # Downsample distance field
        df = self.dist_field[::coarse, ::coarse]
        hc, wc = df.shape

        rot_window_deg=10.0   # +/- around current heading
        rot_step_deg=1.0

        # Current heading in degrees
        theta_deg = math.degrees(self.theta) % 360

        # Build a small set of candidate headings around theta
        half = int(rot_window_deg // rot_step_deg)
        rot_candidates = []
        for k in range(-half, half + 1):
            th_deg = (theta_deg + k * rot_step_deg) % 360
            rot_candidates.append(th_deg)
            
        # Precompute rotated scans
        rotations = []
        for th_deg in rot_candidates:
            th = math.radians(th_deg)
            c, s = math.cos(th), math.sin(th)
            wx = c * px - s * py
            wy = s * px + c * py
            rotations.append((th_deg, wx, wy))

        best_score = -1e18
        best_pose = None

        # Vectorised translation grid
        gx = np.arange(wc) * coarse
        gy = np.arange(hc) * coarse
        GX, GY = np.meshgrid(gx, gy)

        WX = GX * cell_mm
        WY = (h - 1 - GY) * cell_mm

        for th_deg, wx_base, wy_base in rotations:
            # Broadcast scan points over translation grid
            wx_pts = WX[..., None] + wx_base
            wy_pts = WY[..., None] + wy_base

            gx_pts = (wx_pts / cell_mm).astype(np.int32)
            gy_pts = (h - 1 - (wy_pts / cell_mm)).astype(np.int32)

            valid = (gx_pts >= 0) & (gx_pts < w) & (gy_pts >= 0) & (gy_pts < h)

            # Score all translations at once
            # df_vals = np.where(valid, self.dist_field[gy_pts, gx_pts], 255)
            # Allocate output
            df_vals = np.full(gx_pts.shape, 255, dtype=np.float32)

            # Extract only valid indices
            gyv = gy_pts[valid]
            gxv = gx_pts[valid]

            # Safe indexing
            df_vals[valid] = self.dist_field[gyv, gxv]
 
            scores = -np.mean(df_vals, axis=2)

            # Find best
            idx = np.unravel_index(np.argmax(scores), scores.shape)
            score = scores[idx]

            if score > best_score:
                gyc, gxc = idx
                wx = GX[gyc, gxc] * cell_mm
                wy = (h - 1 - GY[gyc, gxc]) * cell_mm
                best_score = score
                best_pose = (wx, wy, math.radians(th_deg))

        return (*best_pose, best_score)

    def relocalise_coarse_retry(self, pts, pred_pose):

        # BAD coarse result → retry around predicted pose
        pred_x = pred_pose[0]
        pred_y = pred_pose[1]
        pred_th = pred_pose[2]

        h, w = self.dist_field.shape
        cell_mm = self.resolution_m * 1000

        gx0 = int(pred_x / cell_mm)
        gy0 = int((h - 1) - pred_y / cell_mm)

        # Local coarse window
        gx_range = range(gx0 - 80, gx0 + 80, 10)
        gy_range = range(gy0 - 80, gy0 + 80, 10)

        # Rotation window around predicted heading
        th_deg0 = math.degrees(pred_th)
        rot_candidates = [(th_deg0 + k) % 360 for k in range(-20, 21, 5)]

        retry_pose = self.global_relocalise_local_window(
            pts,
            gx_range,
            gy_range,
            rot_candidates
        )

        rx, ry, rth, retry_score = retry_pose

        return (rx, ry, rth), retry_score

    def relocalise_medium(self, pts, coarse_pose,
                        rot_window_deg=20,
                        rot_step_deg=2,
                        trans_window_cells=20,
                        trans_step_cells=5):

        cx, cy, cth = coarse_pose

        # Convert coarse pose to grid
        gx0 = int(cx / (self.resolution_m * 1000))
        gy0 = int((self.dist_field.shape[0] - 1) - cy / (self.resolution_m * 1000))

        # Build local search ranges
        gx_range = range(gx0 - trans_window_cells,
                        gx0 + trans_window_cells + 1,
                        trans_step_cells)

        gy_range = range(gy0 - trans_window_cells,
                        gy0 + trans_window_cells + 1,
                        trans_step_cells)

        # Build rotation window
        th_deg0 = math.degrees(cth)
        rot_candidates = [
            (th_deg0 + k * rot_step_deg) % 360
            for k in range(-rot_window_deg // rot_step_deg,
                            rot_window_deg // rot_step_deg + 1)
        ]

        return self.global_relocalise_local_window(
            pts,
            gx_range,
            gy_range,
            rot_candidates
        )

    def relocalise_fine(self, pts, medium_pose,
                        rot_window_deg=4,
                        rot_step_deg=1,
                        trans_window_cells=6,
                        trans_step_cells=1):

        cx, cy, cth = medium_pose

        gx0 = int(cx / (self.resolution_m * 1000))
        gy0 = int((self.dist_field.shape[0] - 1) - cy / (self.resolution_m * 1000))

        gx_range = range(gx0 - trans_window_cells,
                        gx0 + trans_window_cells + 1,
                        trans_step_cells)

        gy_range = range(gy0 - trans_window_cells,
                        gy0 + trans_window_cells + 1,
                        trans_step_cells)

        th_deg0 = math.degrees(cth)
        rot_candidates = [
            (th_deg0 + k * rot_step_deg) % 360
            for k in range(-rot_window_deg // rot_step_deg,
                            rot_window_deg // rot_step_deg + 1)
        ]

        return self.global_relocalise_local_window(
            pts,
            gx_range,
            gy_range,
            rot_candidates
        )

    def global_relocalise_local_window(self, pts, gx_range, gy_range, rot_candidates):

        best_score = -1e18
        best_pose = None

        for th_deg in rot_candidates:
            th = math.radians(th_deg)
            c, s = math.cos(th), math.sin(th)

            # Rotate scan once
            wx_base = c * pts[:,0] - s * pts[:,1]
            wy_base = s * pts[:,0] + c * pts[:,1]

            for gx in gx_range:
                for gy in gy_range:

                    # Convert grid → world
                    wx = gx * self.resolution_m * 1000
                    wy = (self.dist_field.shape[0] - 1 - gy) * self.resolution_m * 1000

                    # Apply translation
                    wx_pts = wx + wx_base
                    wy_pts = wy + wy_base

                    # Convert to grid
                    gx_pts = (wx_pts / (self.resolution_m * 1000)).astype(np.int32)
                    gy_pts = (self.dist_field.shape[0] - 1 - (wy_pts / (self.resolution_m * 1000))).astype(np.int32)

                    valid = (gx_pts >= 0) & (gx_pts < self.dist_field.shape[1]) & \
                            (gy_pts >= 0) & (gy_pts < self.dist_field.shape[0])

                    if not np.any(valid):
                        continue

                    df_vals = self.dist_field[gy_pts[valid], gx_pts[valid]]
                    score = -np.mean(df_vals)

                    if score > best_score:
                        best_score = score
                        best_pose = (wx, wy, th)

        return (*best_pose, best_score)

    def merge_scans(self, scans, angle_step_deg=1.0):
        # scans = list of Nx2 arrays in robot frame (mm)
        # Convert each scan to polar
        all_angles = []
        all_ranges = []

        for pts in scans:
            x = pts[:, 0]
            y = pts[:, 1]
            angles = np.degrees(np.arctan2(y, x)) % 360
            ranges = np.hypot(x, y)
            all_angles.append(angles)
            all_ranges.append(ranges)

        all_angles = np.concatenate(all_angles)
        all_ranges = np.concatenate(all_ranges)

        # Bin by angle
        bins = int(360 / angle_step_deg)
        merged_ranges = np.full(bins, np.inf)

        for ang, r in zip(all_angles, all_ranges):
            idx = int(ang // angle_step_deg)
            merged_ranges[idx] = min(merged_ranges[idx], r)

        # Convert back to Cartesian
        merged_pts = []
        for i in range(bins):
            r = merged_ranges[i]
            if np.isfinite(r):
                ang = np.radians(i * angle_step_deg)
                merged_pts.append([r * np.cos(ang), r * np.sin(ang)])

        return np.array(merged_pts)

    def extract_local_map_points(self, local_map, occ_threshold=0.0):
        ys, xs = np.where(local_map.L > occ_threshold)
        cx = cy = local_map.pixels // 2
        cell_mm = local_map.resolution_m * 1000

        lx = (xs - cx) * cell_mm
        ly = (cy - ys) * cell_mm
        return np.column_stack((lx, ly))

    def calculate_pose_confidence(self, local_pts_mm, pose_guess):
        # We are a little bit lost now - check if we are in a new area and the overlap with the old map is small
        x_mm, y_mm, th = pose_guess
        gx_mm, gy_mm = self.transform_points_world(local_pts_mm, x_mm, y_mm, th)

        cell_mm = self.resolution_m * 1000.0
        gx = (gx_mm / cell_mm).astype(int)
        gy = (gy_mm / cell_mm).astype(int)

        H, W = self.dist_field.shape
        valid = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
        gxv = gx[valid]
        gyv = gy[valid]

        df_vals = self.dist_field[gyv, gxv]   # units: metres
        good_mask = df_vals < config.good_dist_m
        overlap_ratio = np.mean(good_mask)
        mean_dist_m = np.mean(df_vals)
        # Overlap term: 0 at 0.1, 1 at 0.7+
        o = np.clip((overlap_ratio - 0.1) / (0.7 - 0.1), 0.0, 1.0)

        # Distance term: 1 at 50mm, 0 at 400mm+
        d = np.clip((4.0 - mean_dist_m) / (4.0 - 0.05), 0.0, 1.0)
        
        # Combine (geometric mean is harsher than average)
        confidence_val = float(np.sqrt(o * d))
        return confidence_val
        
    def refine_pose_with_micro_map_and_merge(self, heading_deg, avg_move_heading_deg, distance_mm, confidence, has_reset):
        """
        Full stationary refinement pipeline:

        1. Motion prior from odometry + move_confidence + IMU heading
        2. Local relocalisation (if move_confidence high enough)
        3. If local fails → do global relocalisation coarse/med/fine. If global fails, retry with pose, if this fails, 
            refine pose with ICP and then retry global using new pose (if good score)
        3. Build local micro-map from stationary scans using refined pose (if relocalisation worked)
        4. Align micro-map to global map (distance-field based)
        5. Merge micro-map into global log-odds map
        6. Update internal map structures (prob, dist_field, etc.)
        """

        # Convert compass heading to SLAM frame: 0°=north, CCW+
        self.heading_deg = 90.0 - heading_deg
        self.theta = math.radians(self.heading_deg)

        if has_reset:
            # Robot has been initialised or watchdog failure - we cant trust any odometer values
            # self.init_data()
            self.distance_moved_mm = 0
            self.move_confidence = 0
            self.avg_move_heading_deg = heading_deg
        else:            
            self.distance_moved_mm = distance_mm
            self.move_confidence = confidence
            self.avg_move_heading_deg = 90.0 - avg_move_heading_deg

        
        if len(self.scans_mm_stationary) == 0:
            print(f"{datetime.now()}: No scans to process for micro-map refinement!")
            return False

        # print("Number of scans in stationary buffer:", len(self.scans_mm_stationary))
        
        self.slam_scores = [0, 0, 0, 0, 0]
        relocalise_fail = False
        if not self.first_ever_scan:
            merged_pts = self.merge_scans(self.scans_mm_stationary)
            
            # We cannot do any of this on the very first scan since we have no prior pose or map, so just skip to building the initial map. After the first scan, we have a pose guess and can start refining with micro-maps.
            # print("Refining pose with micro-map and merging into global map...")
            # ------------------------------------------------------------
            # 1. Motion prior
            # ------------------------------------------------------------

            dx = self.move_confidence * self.distance_moved_mm * math.cos(math.radians(self.avg_move_heading_deg))
            dy = self.move_confidence * self.distance_moved_mm * math.sin(math.radians(self.avg_move_heading_deg))

            x_pred = self.x + dx
            y_pred = self.y + dy
            pose_from_odometer = (x_pred, y_pred, self.theta)

            # ------------------------------------------------------------
            # 2. ICP refinement + fallback relocalisation
            # ------------------------------------------------------------
            pose_seed = pose_from_odometer
            global_relocalise = False
            print(f"{datetime.now()}: 1. Pose guess from motion prior: {x_pred:.0f},{y_pred:.0f} bearing {config.map_rads_to_world(self.theta):.0f} with move confidence: {self.move_confidence}")
            # if self.move_confidence >= 0.6 and not self.lost and not has_reset:
            #     print(f"{datetime.now()}: 1(b). Performing local relocalisation")
            #     pose_seed, score, overlap, conf = self.local_relocalise(merged_pts, pose_from_odometer, win_xy=1.0, win_th_deg=15)
            #     print(f"{datetime.now()}: 1(c). Local relocalisation score {score} with confidence {conf}, overlap {overlap} set pose: {pose_seed[0]:.0f},{pose_seed[1]:.0f} @ {config.map_rads_to_world(pose_seed[2]):.0f}")
            #     if conf < 0.4 or score <= -0.1:
            #         # local failed → global
            #         global_relocalise = True                    
            # else:
            global_relocalise = True                    
            if global_relocalise:
                print(f"{datetime.now()}: 2(a) Performing global relocalisation with no prior")
                x_rel, y_rel, th_rel, score_coarse = self.global_relocalise(
                    merged_pts
                )
                # If coarse result is bad or theta way off, retry using best guess pose
                deg_diff = abs(math.degrees((th_rel - self.theta + math.pi) % (2 * math.pi) - math.pi))
                self.slam_scores[1] = getattr(score_coarse, "tolist", lambda: score_coarse)()
                if score_coarse <= -0.1 or deg_diff > 25:
                    print(f"{datetime.now()}: 2(b) Retrying global relocalisation (score {score_coarse:.4f} with pose {x_rel:.0f},{y_rel:.0f},{config.map_rads_to_world(th_rel):.0f})")
                    coarse_pose, score_coarse = self.relocalise_coarse_retry(
                        merged_pts, pose_from_odometer)
                    self.slam_scores[2] = getattr(score_coarse, "tolist", lambda: score_coarse)()
                    if score_coarse <= -0.1 or abs(th_rel - self.theta) > 25:
                        print(f"{datetime.now()}: 2(c) Trying ICP refinement as global relocalisation failed (score {score_coarse:.4f} theta {config.map_rads_to_world(th_rel):.0f}")
                        # Try ICP around motion prior
                        x_rel, y_rel, th_rel, icp_error = self.icp_scan_to_map(
                            merged_pts,
                            pose_from_odometer,
                            self.move_confidence)
                        self.slam_scores[0] = float(icp_error)
                        deg_diff = abs(math.degrees((th_rel - self.theta + math.pi) % (2 * math.pi) - math.pi))
                        if icp_error < self.icp_good_threshold and deg_diff < 25:
                            # ICP succeeded → use it as seed
                            print(f"{datetime.now()}: 2(d) ICP refinement of pose successful with error:", icp_error)
                            pose_seed = (x_rel, y_rel, th_rel)
                            print(f"{datetime.now()}: 2(e) Retrying global relocalisation (score {score_coarse:.4f} with pose {x_rel:.0f},{y_rel:.0f},{config.map_rads_to_world(th_rel):.0f})")
                            coarse_pose, score_coarse = self.relocalise_coarse_retry(
                                merged_pts, pose_seed)
                            x_rel, y_rel, th_rel = coarse_pose
                            self.slam_scores[2] = getattr(score_coarse, "tolist", lambda: score_coarse)()
                            if score_coarse <= -0.1 or abs(th_rel - self.theta) > 25:
                                print(f"{datetime.now()}: 2(f) Global relocalisation failed again (score {score_coarse:.4f} with pose {x_rel:.0f},{y_rel:.0f},{config.map_rads_to_world(th_rel):.0f})")
                                relocalise_fail = True
                            else:
                                pose_seed = coarse_pose
                        else:
                            print(f"{datetime.now()}: ICP refinement failed with error {icp_error:.4f} and theta {config.map_rads_to_world(th_rel):.0f}")
                            relocalise_fail = True
                    else:
                        pose_seed = coarse_pose
                else:
                    pose_seed = pose_from_odometer
            if not relocalise_fail:
                # Coarse relocalisation passed
                x_rel, y_rel, th_rel = pose_seed
                # 3. Medium
                print(f"{datetime.now()}: 3 Medium relocalisation with prior {x_rel:.0f},{y_rel:.0f},{config.map_rads_to_world(th_rel):.0f}")
                x_rel, y_rel, th_rel, score_med = self.relocalise_medium(merged_pts,(x_rel, y_rel, th_rel))
                self.slam_scores[3] = getattr(score_med, "tolist", lambda: score_med)()
                # 3. Fine
                print(f"{datetime.now()}: 4 Fine relocalisation with prior {x_rel:.0f},{y_rel:.0f},{config.map_rads_to_world(th_rel):.0f}")
                x_rel, y_rel, th_rel, score_fine = self.relocalise_fine(merged_pts, (x_rel, y_rel, th_rel))                
                pose_seed = (x_rel, y_rel, th_rel)
                self.slam_scores[4] = getattr(score_fine, "tolist", lambda: score_fine)()
                print(f"{datetime.now()}: 5 Relocalised position: {x_rel:.0f},{y_rel:.0f},{config.map_rads_to_world(th_rel):.0f} final score: {score_fine:.4f}")

            # At this point, pose_seed is our best estimate before micro-map

        else:
            pose_seed = (self.x, self.y, self.theta)
            print(f"{datetime.now()}: 1 & 2: First ever scan - using initial pose guess without refinement")
            
        # ------------------------------------------------------------
        # 3. Build local micro-map from stationary scans
        # ------------------------------------------------------------
        print(f"{datetime.now()}: 6. Building local map from this sweep's points, with pose: {pose_seed[0]:.0f},{pose_seed[1]:.0f} @ {config.map_rads_to_world(pose_seed[2]):.0f}")
        local_map = self.build_local_map_from_points(
            pose_seed,
            size_m=config.map_size,
        )

        # ------------------------------------------------------------
        # 4. Align local micro-map to global map
        # ------------------------------------------------------------
        confidence = 0
        if not self.first_ever_scan:
            local_pts_mm = self.extract_local_map_points(local_map)
            confidence = self.calculate_pose_confidence(local_pts_mm, pose_seed)
            if confidence >= 0.7:
                print(f"{datetime.now()}: 7(a). Relocalisation pose confidence high ({confidence:.04f}) - use calc pose")
                pose_align = pose_seed
            elif confidence >= 0.3:
                print(f"{datetime.now()}: 7(b). Relocalisation pose confidence med {confidence:.04f} - blending with odometer")
                #Blend the pose guesses from the (failed) relocalisation and from the odometer
                x_o, y_o, th_o = pose_from_odometer
                x_a, y_a, th_a = pose_seed

                # Blend translation
                x = (1 - confidence) * x_o + confidence * x_a
                y = (1 - confidence) * y_o + confidence * y_a

                # Blend rotation (shortest path)
                dth = (th_a - th_o + np.pi) % (2*np.pi) - np.pi
                th = th_o + confidence * dth
                pose_align = (x, y, th)
            else:
                print(f"{datetime.now()}: 7(c). Relocalisation pose confidence low {confidence:.04f} - NOT updating map")
                self.x, self.y, self.theta = pose_from_odometer
                self.scans_mm_stationary.clear()  # Clear stored scans after processing
                self.lost = True
                return False
            print(f"{datetime.now()}: 7(d). Align local map from this sweep to global map, with pose seed: : {pose_align[0]:.0f},{pose_align[1]:.0f} @ {config.map_rads_to_world(pose_align[2]):.0f}")
            x_map, y_map, theta_map, align_score, can_use = self.align_map_pts_to_global(local_pts_mm, pose_align)
            if can_use:
                self.x = x_map
                self.y = y_map
                self.theta = theta_map
            else:
                #Couldnt align scans to the current map - reject and retry
                print(f"{datetime.now()}: 7(e) Scan alignment to map failed not updating map (score {align_score:.4f}) {self.x:.0f},{self.y:.0f} @ {config.map_rads_to_world(self.theta):.0f})")
                # Save best guess of where we are
                self.x, self.y, self.theta = pose_align
                self.scans_mm_stationary.clear()  # Clear stored scans after processing
                self.lost = True
                return False


        # ------------------------------------------------------------
        # 5. Merge local micro-map into global log-odds map
        # ------------------------------------------------------------
        print(f"{datetime.now()}: 5. Merging local map to global map")
        self.merge_local_map_into_global(local_map, (self.x, self.y, self.theta))

        # ------------------------------------------------------------
        # 6. Rebuild probability, occupancy mask, distance field, etc.
        # ------------------------------------------------------------
        self.update_internal_maps()
        
        self.scans_mm_stationary.clear()  # Clear stored scans after processing
        self.first_ever_scan = False
        self.lost = False
       
        return True

    def update(self, scan_mm):
        pts = self.scan_to_points(scan_mm)
        if pts.shape[0] == 0:
            return

        # Store scan for building local micro-maps during stationary phases
        self.scans_mm_stationary.append(pts)
