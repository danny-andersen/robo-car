import math
import heapq
import numpy as np
from collections import deque

from icp_slam_scan_to_map import ICP_SLAM
from move_veto import MoveVeto
import config

class Explorer:
    def __init__(self, map_pixels, resolution_m):
        self.resolution_m = resolution_m  # meters per cell
        self.map_pixels = map_pixels

    # ---------------------------------------------------------
    # Utility: 8-connected neighbors
    # ---------------------------------------------------------
    def neighbors(self, x, y, w, h):
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),
                       (-1,-1),(-1,1),(1,-1),(1,1)]:
            nx, ny = x+dx, y+dy
            if 0 <= nx < w and 0 <= ny < h:
                yield nx, ny

    # ---------------------------------------------------------
    # Step 1: Detect frontier cells
    # ---------------------------------------------------------
    def find_frontiers(self, occ_img, free_thresh=180, occ_thresh=80):
        """
        occ_img: uint8 map from get_map()
        Returns list of (x, y) grid coords that are frontier cells.
        Frontier = free cell adjacent to at least one unknown cell.
        """
        if not isinstance(occ_img, np.ndarray):
            raise TypeError(f"occ_img must be a numpy array, got {type(occ_img)}")
        if occ_img.ndim != 2:
            raise ValueError(f"occ_img must be 2D, got shape {occ_img.shape}")
        h, w = occ_img.shape

        # Classify
        free = occ_img >= free_thresh
        occ  = occ_img <= occ_thresh
        # Unknown = neither clearly free nor clearly occupied
        unknown = ~(free | occ)

        frontiers = []

        for y in range(1, h - 1):
            for x in range(1, w - 1):
                if not free[y, x]:
                    continue

                # 4‑connected neighbours
                if (unknown[y-1, x] or unknown[y+1, x] or
                    unknown[y, x-1] or unknown[y, x+1]):
                    frontiers.append((x, y))

        return frontiers

    def is_visible(self, grid, rx, ry, fx, fy):
        """Return True if frontier (fx, fy) is visible from robot (rx, ry)."""
        for cx, cy in config.bresenham(rx, ry, fx, fy):
            if config.is_occupied(grid[cy, cx]):
                return False
        return True


    def compute_visibility(self, occ_img, robot_gx, robot_gy, max_range_cells=300):
        h, w = occ_img.shape
        visible = np.zeros_like(occ_img, dtype=bool)

        for angle in np.linspace(0, 2*np.pi, 720):  # 0.5° resolution
            dx = math.cos(angle)
            dy = math.sin(angle)

            x = robot_gx + 0.5
            y = robot_gy + 0.5

            for _ in range(max_range_cells):
                ix = int(x)
                iy = int(y)

                if ix < 0 or iy < 0 or ix >= w or iy >= h:
                    break

                visible[iy, ix] = True

                if config.is_occupied(occ_img[iy, ix]):  # occupied
                    break

                x += dx
                y += dy

        return visible

    def find_occlusion_frontiers(self, occ_img, visible):
        h, w = occ_img.shape
        frontiers = []

        for y in range(1, h-1):
            for x in range(1, w-1):
                if not visible[y, x]:
                    continue

                # Check 8 directions
                for dx, dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
                    nx = x + dx
                    ny = y + dy

                    if not (0 <= nx < w and 0 <= ny < h):
                        continue

                    if visible[ny, nx]:
                        continue

                    if config.is_occupied(occ_img[ny, nx]):  # occupied
                        continue

                    # This is a shadow boundary
                    frontiers.append((x, y))
                    break

        return frontiers

    # ---------------------------------------------------------
    # Step 2: Cluster frontier cells
    # ---------------------------------------------------------
    # def cluster_frontiers(self, frontier_pixels, min_cluster_size=20):
    #     if not frontier_pixels:
    #         return []

    #     frontier_set = set(frontier_pixels)
    #     visited = set()
    #     clusters = []

    #     for p in frontier_pixels:
    #         if p in visited:
    #             continue

    #         stack = [p]
    #         cluster = []

    #         while stack:
    #             cx, cy = stack.pop()
    #             if (cx, cy) in visited:
    #                 continue

    #             visited.add((cx, cy))
    #             cluster.append((cx, cy))

    #             # 4-connected neighbours
    #             for nx, ny in [(cx+1, cy), (cx-1, cy), (cx, cy+1), (cx, cy-1)]:
    #                 if (nx, ny) in frontier_set and (nx, ny) not in visited:
    #                     stack.append((nx, ny))

    #         if len(cluster) >= min_cluster_size:
    #             clusters.append(cluster)

    #     return clusters

    def cluster_frontiers(self, frontiers, dist_thresh=0.5):
        clusters = []
        used = set()

        for i, p in enumerate(frontiers):
            if i in used:
                continue

            cluster = [p]
            used.add(i)

            for j, q in enumerate(frontiers):
                if j in used:
                    continue

                if math.dist(p, q) < dist_thresh:
                    cluster.append(q)
                    used.add(j)

            clusters.append(cluster)

        return clusters

    def pick_nearest_cluster(self, clusters, robot_x_mm, robot_y_mm):
        best = None
        best_d2 = 1e18

        for cluster in clusters:
            cx, cy = self.cluster_centroid(cluster)
            wx, wy = self.grid_to_world_mm(cx, cy)

            dx = wx - robot_x_mm
            dy = wy - robot_y_mm
            d2 = dx*dx + dy*dy

            if d2 < best_d2:
                best_d2 = d2
                best = (cx, cy)

        return best  # (gx, gy)

    # ---------------------------------------------------------
    # Utility: compute cluster centroid
    # ---------------------------------------------------------
    def cluster_centroid(self, cluster):
        xs = [p[0] for p in cluster]
        ys = [p[1] for p in cluster]
        return (int(sum(xs)/len(xs)), int(sum(ys)/len(ys)))

    def detect_frontier_clusters(self, occ_img, robot_x_mm, robot_y_mm, res):
        frontiers = self.find_frontiers(occ_img)
        print("Detected frontiers:", len(frontiers))
        clusters = self.cluster_frontiers(frontiers, min_cluster_size=20)
        if (len(clusters) == 0):
            # Try a lower threshold for cluster size to get more targets, even if they are smaller
            clusters = self.cluster_frontiers(frontiers, min_cluster_size=10)
        print("Detected clusters:", len(clusters))

        # Convert robot world→grid
        gx = int((robot_x_mm / 1000.0) / res)
        gy = int((robot_y_mm / 1000.0) / res)
        gy = occ_img.shape[0] - 1 - gy

        cluster_targets = []
        for cluster in clusters:
            # Option A: centroid
            cx, cy = self.cluster_centroid(cluster)

            # Option B: nearest frontier cell
            # cx, cy = nearest_frontier(cluster, gx, gy)

            cluster_targets.append((cx, cy))

        return clusters, cluster_targets

    # ---------------------------------------------------------
    # Step 3: Pick best frontier cluster (closest to robot)
    # ---------------------------------------------------------
    def pick_best_frontier(self, clusters, robot_x_mm, robot_y_mm):
        best = None
        best_dist = float('inf')

        for cluster in clusters:
            # Compute centroid
            cx = sum(x for x, _ in cluster) / len(cluster)
            cy = sum(y for _, y in cluster) / len(cluster)

            # Convert robot pose to grid coords
            rx = robot_x_mm / 1000.0 / self.resolution_m
            ry = robot_y_mm / 1000.0 / self.resolution_m

            dist = (cx - rx)**2 + (cy - ry)**2
            if dist < best_dist:
                best_dist = dist
                best = (int(cx), int(cy))

        return best

    # ---------------------------------------------------------
    # Step 4: A* path planning
    # ---------------------------------------------------------
    def astar(self, grid, start, goal):
        h, w = grid.shape
        sx, sy = start
        gx, gy = goal

        open_set = []
        heapq.heappush(open_set, (0, (sx, sy)))

        came_from = {}
        gscore = { (sx, sy): 0 }

        while open_set:
            _, (x, y) = heapq.heappop(open_set)

            if (x, y) == (gx, gy):
                # Reconstruct path
                path = [(x, y)]
                while (x, y) in came_from:
                    x, y = came_from[(x, y)]
                    path.append((x, y))
                return path[::-1]

            for nx, ny in self.neighbors(x, y, w, h):
                if grid[ny, nx] == 255:  # obstacle
                    continue

                tentative = gscore[(x, y)] + 1
                if (nx, ny) not in gscore or tentative < gscore[(nx, ny)]:
                    gscore[(nx, ny)] = tentative
                    priority = tentative + abs(nx - gx) + abs(ny - gy)
                    heapq.heappush(open_set, (priority, (nx, ny)))
                    came_from[(nx, ny)] = (x, y)

        return None  # no path

    def pick_next_waypoint(self, robot_x_mm, robot_y_mm, paths, startPath, min_step_mm=300):
        # """
        # path: list of (gx, gy) from A*
        # min_step_mm: minimum world-frame distance to justify a move
        # """

        if len(paths) < 2:
            return None  # already at target

        # Try each waypoint until one is far enough away
        for gx, gy in paths[startPath:]:
            wx, wy = self.grid_to_world_mm(gx, gy)
            dx = wx - robot_x_mm
            dy = wy - robot_y_mm
            dist_mm = math.sqrt(dx*dx + dy*dy)

            if dist_mm >= min_step_mm:
                return gx, gy

        # If all steps are tiny, use the final target
        return paths[-1]


    # ---------------------------------------------------------
    # Step 5: Convert path to robot moves
    # ---------------------------------------------------------
    # def path_to_moves(self, path):
    #     moves = []
    #     if not path:
    #         return moves

    #     def to_world(ix, iy):
    #         x_m = ix * self.res
    #         y_m = iy * self.res
    #         return x_m * 1000, y_m * 1000  # mm

    #     prev_x, prev_y = to_world(*path[0])

    #     for ix, iy in path[1:]:
    #         x, y = to_world(ix, iy)
    #         dx = x - prev_x
    #         dy = y - prev_y

    #         # Map-frame bearing (mathematical)
    #         map_bearing = math.degrees(math.atan2(dy, dx))

    #         # Convert to compass absolute bearing
    #         compass_bearing = (90 - map_bearing) % 360

    #         distance = math.sqrt(dx*dx + dy*dy)

    #         moves.append((compass_bearing, distance))

    #         prev_x, prev_y = x, y

    #     return moves

    def grid_to_world_mm(self, gx, gy):
        wx = gx * self.resolution_m * 1000
        # wy = (self.map_pixels - 1 - gy) * self.resolution_m * 1000
        wy = gy * self.resolution_m * 1000
        return wx, wy

 
    def compute_move_mm(self, robot_x_mm, robot_y_mm, path):
        """
        path: (gx, gy) grid cells from A*
        returns: (dist_mm, target_world_x_mm, target_world_y_mm)
        """

        # next_gx, next_gy = path

        # # Convert to world mm
        # wx, wy = self.grid_to_world_mm(next_gx, next_gy)
        wx, wy = path

        # Compute world-frame delta
        dx = wx - robot_x_mm
        dy = wy - robot_y_mm

        print(f"Computing move to grid {path} → world ({wx:.1f} mm, {wy:.1f} mm), robot {robot_x_mm:.1f} mm, {robot_y_mm:.1f} mm, delta ({dx:.1f} mm, {dy:.1f} mm)")
        dist_mm = math.sqrt(dx*dx + dy*dy)
        bearing = math.atan2(dy, dx)
        bearing_world_deg = math.degrees(bearing)

        return bearing_world_deg, dist_mm

    def score_cluster(self, cluster, robot_x, robot_y, robot_theta,
                    w_ig=1.0, w_dist=1.0, w_orient=0.5):

        # Centroid
        xs = [p[0] for p in cluster]
        ys = [p[1] for p in cluster]
        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)

        # Information gain
        IG = len(cluster)

        # Distance
        D = math.dist((robot_x, robot_y), (cx, cy))

        # Orientation alignment
        dx = cx - robot_x
        dy = cy - robot_y
        bearing = math.atan2(dx, dy)
        angle_diff = abs((bearing - robot_theta + math.pi) % (2*math.pi) - math.pi)
        O = math.cos(angle_diff)

        score = w_ig * IG - w_dist * D + w_orient * O
        return score, (cx, cy)
    
    def choose_best_cluster(self, clusters, robot_x, robot_y, robot_theta):
        best_score = -1e9
        best_cluster = None
        best_centroid = None

        for cluster in clusters:
            score, centroid = self.score_cluster(cluster, robot_x, robot_y, robot_theta)
            if score > best_score:
                best_score = score
                best_cluster = cluster
                best_centroid = centroid

        return best_cluster, best_centroid

    def raycast_into_map(self, grid, px, py, angle, max_range, step=0.05):
        r = 0.0
        H, W = grid.shape

        while r < max_range:
            x = px + r * math.cos(angle)
            y = py + r * math.sin(angle)
            gx, gy = config.world_to_grid(x, y, self.map_pixels)

            if not (0 <= gx < W and 0 <= gy < H):
                return "out", r

            cell = config.classify_cell(grid[gy, gx])

            if cell == "occupied":
                return "hit", r

            if cell == "unknown":
                return "unknown", r

            # free → continue
            r += step

        return "max", max_range

    def choose_target_point(self, cluster, centroid):
        cx, cy = centroid
        return min(cluster, key=lambda p: math.dist(p, (cx, cy)))
    
    def sample_candidate_poses(self, robot_x, robot_y, radii=(0.3, 0.6, 1.0), num_angles=24):
        poses = []
        for r in radii:
            for i in range(num_angles):
                a = 2 * math.pi * i / num_angles
                px = robot_x + 1000 * r * math.cos(a)
                py = robot_y + 1000 * r * math.sin(a)
                poses.append((px, py))
        return poses

    def has_line_of_sight(self, grid, robot_pos, cx, cy):
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
            if iy < 0 or iy >= grid.shape[0] or ix < 0 or ix >= grid.shape[1]:
                return False

            cell = config.classify_cell(grid[iy, ix])

            if cell == "occupied":
                return False

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
                    print(f"{cx},{cy} blocked by {arc_center},{dist_cm}")
                    return True  # candidate is inside obstacle arc

        return False

    def score_pose(self, px, py, robot_x, robot_y, info_gain,
               w_ig=1.0, w_dist=0.5):
        dist = math.dist((px, py), (robot_x, robot_y))
        return w_ig * info_gain - w_dist * dist

    def filter_candidate_poses(self, grid, robot_x, robot_y, poses):
        H, W = grid.shape
        print(f"Map dimensions: {W}, {H}")
        rx_cell = config.world_to_grid(robot_x, robot_y, self.map_pixels)   
        print(f"Robot at {rx_cell}")

        valid = []
        # Poses are in world coordinates (metres), map occupancy is in int coords
        for px, py in poses:
            print(f"Candidate: {px},{py}")
            gx, gy = config.world_to_grid(px, py, self.map_pixels)
            if not (0 <= gx < W and 0 <= gy < H):
                print(f"Pose {gx},{gy} out of bounds")
                continue
            if not config.is_free(grid[gy, gx]):
                print(f"Pose {gx},{gy} not free: {grid[gy, gx]}")
                continue
            if not self.has_line_of_sight(grid, rx_cell, gx, gy):
                print(f"Pose {gx},{gy} not reachable")
                continue
            if self.candidate_blocked_by_obstacle((robot_x/1000.0, robot_y/1000.0), px/1000.0, py/1000.0):
                print(f"Pose {gx},{gy} behind ultrasound barrier")
                continue
            valid.append((px, py))
        return valid
        
    def compute_information_gain(self, grid, px, py, max_range, num_rays=72):
        gain = 0
        for i in range(num_rays):
            angle = 2 * math.pi * i / num_rays
            result, _ = self.raycast_into_map(grid, px, py, angle, max_range)
            if result == "unknown" or result == "out":
                # This position has a view of an unknown part of the map
                gain += 1
        return gain
    
    def choose_next_best_view(self, grid, robot_x, robot_y, max_range=4.0):
        # 1) sample
        poses = self.sample_candidate_poses(robot_x, robot_y)

        # 2) filter
        poses = self.filter_candidate_poses(grid, robot_x, robot_y, poses)
        if not poses:
            print("No candidate targets/poses found!")
            return None, []  # no NBV
        else:
            print(f"Candidate targets: {len(poses)}")

        # 3) info gain per pose
        gains = {}
        for px, py in poses:
            gains[(px, py)] = self.compute_information_gain(grid, px, py, max_range)

        # 4) score and pick best
        best_pose = None
        best_score = -1e9
        for (px, py) in poses:
            s = self.score_pose(px, py, robot_x, robot_y, gains[(px, py)])
            if s > best_score:
                best_score = s
                best_pose = (px, py)

        return best_pose, [(p, gains[p]) for p in poses]
    

class ExplorationManager:
    def __init__(self, slam, map_size_m=config.map_size, resolution_m=config.map_resolution_m):
        self.slam : ICP_SLAM = slam
        self.resolution_m = resolution_m
        self.map_pixels = int(map_size_m / resolution_m) 
        print(f"Explorer map pixels {self.map_pixels}")

        self.veto = MoveVeto(slam, resolution_m)
        self.explorer = Explorer(self.map_pixels, resolution_m)

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
    
    def get_next_move(self):
        
        """
        Returns a single next move:
            - distance_mm
            - absolute_bearing_deg (north-up world frame)
            - turn_rad (relative rotation needed)
            - (target_world_x_mm, target_world_y_mm)
        """
        occ_img = self.slam.get_map()
        robot_x_mm, robot_y_mm, robot_theta = self.slam.get_pose()

        self.target, self.candidate_targets = self.explorer.choose_next_best_view(occ_img, robot_x_mm, robot_y_mm, robot_theta)

        print(f"Chosen target grid: {self.target}")
        next_move = None
        # ------------------------------------------------------------
        # 6. Plan A* path to the chosen cluster centroid
        # ------------------------------------------------------------
        if self.target is not None:
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
            print("Proposed move:", move)
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
            print(f"Next move (bearing, distance_cm): {self.next_waypoint} to target {self.target}")
            return next_move
        else:
            # No safe move available - Let the robot decide what to do (e.g. rotate in place)
            # return (config.NO_SAFE_DIRECTION, 0)  # No direction, no distance  
            print("No safe move found, returning NO_SAFE_DIRECTION") 
            self.world_bearing = config.NO_SAFE_DIRECTION
            return (config.NO_SAFE_DIRECTION, 0)  # No direction, no distance   


    # def get_next_move(self):
    #     grid = self.slam.get_map()
    #     robot_x_mm, robot_y_mm, th = self.slam.get_pose()

    #     (clusters, targets) = self.explorer.detect_frontier_clusters(grid, robot_x_mm, robot_y_mm, self.resolution_m)
    #     print(f"Detected clusters {len(clusters)} targets: {len(targets)} {targets}")
    #     self.frontier_log.append((clusters, targets))
    #     # frontiers = self.find_frontiers(grid)
    #     # print("Detected frontiers:", len(frontiers))
    #     # if not frontiers:
    #     #     return []

    #     # clusters = self.cluster_frontiers(frontiers)
    #     # target = self.pick_best_frontier(clusters, robot_x_mm, robot_y_mm)
    #     if not targets:
    #         return (config.NO_DIRECTION, 0)

    #     # Convert robot pose to grid coords
    #     sx = int(robot_x_mm / 1000.0 / self.resolution_m)
    #     sy = int(robot_y_mm / 1000.0 / self.resolution_m)

        
    #     next_move = None
    #     for target in targets:
    #         print(f"Computing path to target {target}...")
    #         paths = self.explorer.astar(grid, (sx, sy), target)
    #         for startPath in range(1, len(paths)):  # Find a viable path
    #             next_waypoint = self.explorer.pick_next_waypoint(robot_x_mm, robot_y_mm, paths, startPath)
    #             if next_waypoint:
    #                 print(f"Next_waypoint to {target}: {next_waypoint} ")
    #                 move = self.explorer.compute_move_mm(robot_x_mm, robot_y_mm, next_waypoint)
    #                 print("Proposed move:", move)
    #                 next_move = self.veto.adjust_move(move, self.obstacles)
    #                 if next_move:
    #                     break
    #         if next_move:
    #             break
             
 
    #     if next_move:
    #         world_bearing = self.slam.convert_bearing_from_slam_frame(next_move[0])
    #         next_move = (world_bearing, next_move[1]/10.0)  # convert mm to cm
    #         print(f"Next move (bearing, distance_mm): {next_move} to target {target}")
    #         return next_move
    #     else:
    #         # No safe move available - Let the robot decide what to do (e.g. rotate in place)
    #         # return (config.NO_SAFE_DIRECTION, 0)  # No direction, no distance  
    #         print("No safe move found, returning NO_SAFE_DIRECTION") 
    #         return (config.NO_SAFE_DIRECTION, 0)  # No direction, no distance   
