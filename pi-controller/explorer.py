import math
import numpy as np
from scipy.ndimage import distance_transform_edt
from datetime import datetime

from icp_slam_scan_to_map import ICP_SLAM
from move_veto import MoveVeto
import config

class Explorer:
    def __init__(self, slam, map_pixels, resolution_m):
        self.slam : ICP_SLAM = slam
        self.resolution_m = resolution_m  # meters per cell
        self.map_pixels = map_pixels

    def grid_to_world_mm(self, gx, gy):
        wx = gx * self.resolution_m * 1000
        # wy = (self.map_pixels - 1 - gy) * self.resolution_m * 1000
        wy = gy * self.resolution_m * 1000
        return wx, wy

 
    def compute_move_mm(self, robot_x_mm, robot_y_mm, path):
        """
        path: (gx, gy) world coordinates
        returns: (dist_mm, target_world_x_mm, target_world_y_mm)
        """

        # next_gx, next_gy = path

        # # Convert to world mm
        # wx, wy = self.grid_to_world_mm(next_gx, next_gy)
        wx, wy = path

        # Compute world-frame delta
        dx = wx - robot_x_mm
        dy = wy - robot_y_mm

        print(f"{datetime.now()}: Computing move to grid {wx:.0f},{wy:.0f} → world ({wx:.0f} mm, {wy:.0f} mm), robot {robot_x_mm:.0f} mm, {robot_y_mm:.0f} mm, delta ({dx:.1f} mm, {dy:.1f} mm)")
        dist_mm = math.sqrt(dx*dx + dy*dy)
        bearing = math.atan2(dy, dx)
        bearing_world_deg = 90 - math.degrees(bearing)
        # Normalize to 0–360
        norm_bearing = (bearing_world_deg + 360) % 360

        return norm_bearing, dist_mm

    def raycast_into_map(self, grid, px, py, angle, max_range):
        r = 0.0
        step = 2 * self.resolution_m * 1000.0
        H, W = grid.shape
        result = "max"
        while r < max_range:
            x = px + r * math.cos(angle)
            y = py + r * math.sin(angle)
            gx, gy = config.world_to_grid(x, y, self.map_pixels)

            if not (0 <= gx < W and 0 <= gy < H):
                return "out", r

            cell = config.classify_cell(grid[gy, gx])
            # print(f"At {x}, {y}, grid {gx, gy} val {grid[gy, gx]} cell is {cell}")

            if cell == "occupied":
                return "hit", r

            if cell == "unknown":
                return "unknown", r
                
            # free → continue
            r += step

        return result, r

    def sample_candidate_poses(self, robot_x, robot_y, radii=0.6, num_angles=12):
        poses = []
        angle_factor = 2 * math.pi/ num_angles
        r_mm = radii * 1000
        for i in range(num_angles):
            a = angle_factor * i
            px = robot_x + r_mm * math.cos(a)
            py = robot_y + r_mm * math.sin(a)
            poses.append((px, py))
        return poses

    def inflate_occupancy(self, grid):
        # cell_mm = resolution_m * 1000
        # robot_radius_cells = int(robot_radius_mm / cell_mm)

        # print("Grid Distance field min/max:", np.min(self.slam.dist_field), np.max(self.slam.dist_field))
        # print("Grid Occupied cells:", np.sum(grid))
        # print("Free cells:", np.sum(~grid))
        # print("Grid shape:", grid.shape)

        # Occupied mask
        occupied = grid > (255 * config.occ_threshold)
        # print("Occupied cells:", np.sum(occupied))
        # print("Free cells:", np.sum(~occupied))
        # print("Grid shape:", grid.shape)
        
        # Distance transform on free space
        dist = distance_transform_edt(~occupied)
        # print("Inflated distance field min/max:", np.min(self.slam.dist_field), np.max(self.slam.dist_field))
        inflated = dist <= config.robot_radius_cells
        
        # Inflate: mark all cells within robot radius as occupied
        # inflated = self.slam.dist_field <= config.robot_radius_cells

        return inflated.astype(np.uint8)  # 1 = inflated obstacle, 0 = free

    def has_line_of_sight(self, inflated_grid, robot_pos, cx, cy):
        rx, ry = robot_pos

        dx = cx - rx
        dy = cy - ry

        steps = max(abs(dx), abs(dy))
        if steps == 0:
            return True

        x_inc = dx / steps
        y_inc = dy / steps

        x = rx
        y = ry

        for _ in range(steps):
            x += x_inc
            y += y_inc

            ix = int(round(x))
            iy = int(round(y))
            # Bounds check
            if iy < 0 or iy >= inflated_grid.shape[0] or ix < 0 or ix >= inflated_grid.shape[1]:
                return False

            if inflated_grid[iy, ix]:  # robot cannot fit here
                return False

            # if iy < 0 or iy >= grid.shape[0] or ix < 0 or ix >= grid.shape[1]:
            #     return False
            # # Use distance field to check for occupied cells along the path to the target
            # if self.slam.dist_field[iy, ix] < config.robot_radius_cells:
            #     return False

            # cell = config.classify_cell(grid[iy, ix])

            # if cell == "occupied":
            #     return False

        return True

    def bearing_in_arc(self, bearing, arc_start, arc_end):
        if arc_start <= arc_end:
            return arc_start <= bearing <= arc_end
        else:
            # wrap-around case (e.g. 350° → 10°)
            return bearing >= arc_start or bearing <= arc_end

    def candidate_blocked_by_obstacle(self, robot_pos, cx, cy):
        rx, ry = robot_pos
        dx = cx - rx
        dy = cy - ry

        candidate_bearing = math.degrees(math.atan2(dy, dx)) % 360
        candidate_dist = math.hypot(dx, dy)

        for obs in config.obstacles:
            width = obs.get("width", 0)
            dist_cm = obs.get("avgDistance", 0)

            if width <= 0 or dist_cm <= 0:
                continue

            arc_center = (90 - obs["bearing"]) % 360
            half_width = width / 2.0
            arc_start = (arc_center - half_width) % 360
            arc_end   = (arc_center + half_width) % 360

            # Check bearing
            if self.bearing_in_arc(candidate_bearing, arc_start, arc_end):
                # Check distance
                if candidate_dist >= (dist_cm / 100.0):
                    # print(f"{cx},{cy} blocked by {arc_center},{dist_cm}")
                    return True  # candidate is inside obstacle arc

        return False

    def score_pose(self, px, py, robot_x, robot_y, info_gain,
               w_ig=1.0, w_dist=0.3):
        dist = math.dist((px, py), (robot_x, robot_y))
        return w_ig * info_gain - w_dist * dist

    def filter_candidate_poses(self, grid, inflated_grid, robot_x, robot_y, poses):
        H, W = grid.shape
        # print(f"Map dimensions: {W}, {H}")
        rx_cell = config.world_to_grid(robot_x, robot_y, self.map_pixels)   
        # print(f"Robot at {rx_cell}")

        valid = []
        # Poses are in world coordinates (metres), map occupancy is in int coords
        for px, py in poses:
            # print(f"Candidate: {px},{py}")
            gx, gy = config.world_to_grid(px, py, self.map_pixels)
            if not (0 <= gx < W and 0 <= gy < H):
                # print(f"Pose {gx},{gy} out of bounds")
                continue
            if not config.is_free(grid[gy, gx]):
                # print(f"Pose {gx},{gy} not free: {grid[gy, gx]}")
                continue
            if not self.has_line_of_sight(inflated_grid, rx_cell, gx, gy):
                # print(f"Pose {gx},{gy} not reachable")
                continue
            if self.candidate_blocked_by_obstacle((robot_x/1000.0, robot_y/1000.0), px/1000.0, py/1000.0):
                # print(f"Pose {gx},{gy} behind ultrasound barrier")
                continue
            valid.append((px, py))
        return valid
        
    def compute_information_gain(self, grid, px, py, max_range):
        gain = 0
        num_rays=72
        angle_factor = 2 * math.pi / num_rays 
        for i in range(num_rays):
            angle = angle_factor * i
            result, dist = self.raycast_into_map(grid, px, py, angle, max_range)
            # print(f"At {px,py} Angle: {math.degrees(angle)} result: {result} at {dist}mm with max at {max_range}mm")
            if result in ["unknown", "out", "max"]:
                # This position has a view of an unknown part of the map
                gain += 1
        return gain
    
    def choose_next_best_view(self, grid, robot_x, robot_y, map_updated, max_range=2000):
        if map_updated:
            distance = [0.3, 0.6, 0.9, 1.2]
        else:
            #If we havent updated the map, then keep any movement small until we find where we are
            distance = [0.2, 0.3, 0.4]
        best_pose = None
        pose_found = False
        poses = []
        gains = {}
        inflated_grid = self.inflate_occupancy(grid)
        for d in distance:
            # 1) sample
            d_poses = self.sample_candidate_poses(robot_x, robot_y, radii=d)

            # 2) filter
            f_poses = self.filter_candidate_poses(grid, inflated_grid, robot_x, robot_y, d_poses)
            if not f_poses:
                # print(f"No candidate targets/poses found at radius {d}!")
                continue
            # else:
            #     print(f"Candidate targets: {len(poses)}")
            poses += f_poses

            # 3) info gain per pose
            for px, py in f_poses:
                gains[(px, py)] = self.compute_information_gain(grid, px, py, max_range)

            # 4) score and pick best
            best_pose = None
            if map_updated:
                best_score = -1e9
            else:
                #Map wasnt updated - need to move to a target that has the smallest gain
                # This will give a greater chance that global relocalisation of the current pose will work
                best_score = 10000
            for (px, py) in poses:
                # s = self.score_pose(px, py, robot_x, robot_y, gains[(px, py)])
                s = gains[(px, py)]
                if map_updated and s > best_score:
                    best_score = s
                    best_pose = (px, py)
                    if best_score >= 5:
                        # This one is worthwhile, otherwise increase distance
                        pose_found = True
                        break
                        
                if not map_updated and s < best_score:
                    best_score = s
                    best_pose = (px, py)
                    if best_score <= 2:
                        # Stick with this one as we good chance we have been here before 
                        # and so can determine where we are
                        pose_found = True
                        break
            if pose_found:
                # print(f"Found target {px:.0f},{py:.0f} with score of {best_score}")
                break

        # print(f"Filtered Poses: {poses} have gains {gains}")
        return best_pose, [(p, gains[p]) for p in poses]
    

class ExplorationManager:
    def __init__(self, slam):
        self.map_size_m=config.map_size
        self.slam : ICP_SLAM = slam
        self.resolution_m = config.map_resolution_m
        self.map_pixels = int(self.map_size_m / self.resolution_m) 
        # print(f"Explorer map pixels {self.map_pixels}")

        self.veto = MoveVeto(slam, self.resolution_m)
        self.explorer = Explorer(slam, self.map_pixels, self.resolution_m)

        self.obstacles = []   # ultrasonic obstacles from robot
        self.clusters = []    # current frontier clusters
        self.target = None    # current target frontier (gx, gy)
        self.candidate_targets = []
        self.next_waypoint = None # next waypoint on path to target (gx, gy)

    # ---------------------------------------------------------
    # Robot sends ultrasonic sweep results
    # ---------------------------------------------------------
    def receive_obstacles(self, obstacles):
        self.obstacles = obstacles

    # ---------------------------------------------------------
    # Main API: robot requests next move
    # ---------------------------------------------------------
    
    def get_next_move(self, map_updated):
        
        """
        Returns a single next move:
            - distance_mm
            - absolute_bearing_deg (north-up world frame)
            - turn_rad (relative rotation needed)
            - (target_world_x_mm, target_world_y_mm)
        """
        occ_img = self.slam.get_map()
        robot_x_mm, robot_y_mm, robot_theta = self.slam.get_pose()

        self.target, self.candidate_targets = self.explorer.choose_next_best_view(occ_img, robot_x_mm, robot_y_mm, map_updated)

        next_move = None
        # ------------------------------------------------------------
        # 6. Plan A* path to the chosen cluster centroid
        # ------------------------------------------------------------
        if self.target is not None:
            print(f"{datetime.now()}: Chosen target coords mm: {self.target[0]:.0f},{self.target[1]:.0f}")
            # path = self.explorer.astar(occ_img, (robot_gx, robot_gy), self.target)
            # if not path or len(path) < 2:
            #     print("No path to target or already at target")
            #     return (config.NO_SAFE_DIRECTION, 0)  # no path or already at target
            # # ------------------------------------------------------------
            # # 7. Skip tiny A* steps (avoid 20–28 mm grid artefacts)
            # # ------------------------------------------------------------
            # for startPath in range(1, len(path)):  # Find a viable path
            #     next_waypoint = self.explorer.pick_next_waypoint(robot_x_mm, robot_y_mm, path, startPath, min_step_mm=300)
            #     if next_waypoint:
                    # print(f"Next_waypoint to {self.target}: {next_waypoint} ")
            move = self.explorer.compute_move_mm(robot_x_mm, robot_y_mm, self.target)
            print(f"{datetime.now()}: Proposed move: {move[1]:.1f}mm, {move[0]:.1f} degs")
            next_move = self.veto.adjust_move(move, self.obstacles)
            # if next_move:
            #     break

       # ------------------------------------------------------------
        # 9. Return single next move
        # ------------------------------------------------------------
        if next_move:
            world_bearing = self.slam.convert_bearing_from_slam_frame(next_move[0])
            self.next_waypoint = (world_bearing, next_move[1])  # Store for logging
            next_move = (world_bearing, next_move[1]/10.0)  # convert mm to cm
            print(f"{datetime.now()}: Next move: {next_move[1]:.0f}cm @ {next_move[0]:.0f} deg to target {self.target[0]:.0f}mm,{self.target[1]:.0f}mm ")
            return next_move
        else:
            # No safe move available - Let the robot decide what to do (e.g. rotate in place)
            # return (config.NO_SAFE_DIRECTION, 0)  # No direction, no distance  
            print(f"{datetime.now()}: No safe move found, returning NO_SAFE_DIRECTION") 
            # self.world_bearing = config.NO_SAFE_DIRECTION
            return (config.NO_SAFE_DIRECTION, 0)  # No direction, no distance   

