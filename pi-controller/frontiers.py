import math
import heapq
import numpy as np

from icp_slam_scan_to_map import ICP_SLAM
from move_veto import MoveVeto
import config

class Explorer:
    def __init__(self, resolution_m):
        self.resolution_m = resolution_m  # meters per cell

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

                if occ_img[iy, ix] < 80:  # occupied
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

                    if occ_img[ny, nx] <= 80:  # occupied
                        continue

                    # This is a shadow boundary
                    frontiers.append((x, y))
                    break

        return frontiers

    # ---------------------------------------------------------
    # Step 2: Cluster frontier cells
    # ---------------------------------------------------------
    def cluster_frontiers(self, frontier_pixels, min_cluster_size=20):
        if not frontier_pixels:
            return []

        frontier_set = set(frontier_pixels)
        visited = set()
        clusters = []

        for p in frontier_pixels:
            if p in visited:
                continue

            stack = [p]
            cluster = []

            while stack:
                cx, cy = stack.pop()
                if (cx, cy) in visited:
                    continue

                visited.add((cx, cy))
                cluster.append((cx, cy))

                # 4-connected neighbours
                for nx, ny in [(cx+1, cy), (cx-1, cy), (cx, cy+1), (cx, cy-1)]:
                    if (nx, ny) in frontier_set and (nx, ny) not in visited:
                        stack.append((nx, ny))

            if len(cluster) >= min_cluster_size:
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

        next_gx, next_gy = path

        # Convert to world mm
        wx, wy = self.grid_to_world_mm(next_gx, next_gy)

        # Compute world-frame delta
        dx = wx - robot_x_mm
        dy = wy - robot_y_mm

        print(f"Computing move to grid {path} → world ({wx:.1f} mm, {wy:.1f} mm), robot {robot_x_mm:.1f} mm, {robot_y_mm:.1f} mm, delta ({dx:.1f} mm, {dy:.1f} mm)")
        dist_mm = math.sqrt(dx*dx + dy*dy)
        bearing = math.atan2(dy, dx)
        bearing_world_deg = math.degrees(bearing)

        return bearing_world_deg, dist_mm

class ExplorationManager:
    def __init__(self, slam, resolution_m=0.02):
        self.slam : ICP_SLAM = slam
        self.resolution_m = resolution_m
        self.veto = MoveVeto(slam, resolution_m)
        self.explorer = Explorer(resolution_m)

        self.obstacles = []   # ultrasonic obstacles from robot
        self.frontier_log = []   # log of frontier detection results
        

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
        # ------------------------------------------------------------
        # 1. Convert robot world pose → grid
        # ------------------------------------------------------------
        robot_gx = int(robot_x_mm / (self.resolution_m * 1000))
        robot_gy = int(robot_y_mm / (self.resolution_m * 1000))

        # ------------------------------------------------------------
        # 2. Compute visibility mask (ray casting)
        # ------------------------------------------------------------
        visible = self.explorer.compute_visibility(occ_img, robot_gx, robot_gy)

        # ------------------------------------------------------------
        # 3. Detect occlusion frontiers
        # ------------------------------------------------------------
        frontier_pixels = self.explorer.find_occlusion_frontiers(occ_img, visible)
        print(f"Detected {len(frontier_pixels)} frontier pixels")
        if not frontier_pixels:
            print("No frontiers detected, cannot explore further")
            return (config.NO_SAFE_DIRECTION, 0)  # no exploration targets

        # ------------------------------------------------------------
        # 4. Cluster occlusion frontiers
        # ------------------------------------------------------------
        clusters = self.explorer.cluster_frontiers(frontier_pixels)
        print(f"Clustered into {len(clusters)} frontier clusters")
        if not clusters:
            print("No frontier clusters found, cannot explore further")
            return (config.NO_SAFE_DIRECTION, 0)

        # ------------------------------------------------------------
        # 5. Pick nearest cluster centroid (local exploration first)
        # ------------------------------------------------------------
        target_gx, target_gy = self.explorer.pick_nearest_cluster(
            clusters,
            robot_x_mm,
            robot_y_mm
        )

        # ------------------------------------------------------------
        # 6. Plan A* path to the chosen cluster centroid
        # ------------------------------------------------------------
        path = self.explorer.astar(occ_img, (robot_gx, robot_gy), (target_gx, target_gy))
        if not path or len(path) < 2:
            print("No path to target or already at target")
            return (config.NO_SAFE_DIRECTION, 0)  # no path or already at target
        # ------------------------------------------------------------
        # 7. Skip tiny A* steps (avoid 20–28 mm grid artefacts)
        # ------------------------------------------------------------
        for startPath in range(1, len(path)):  # Find a viable path
            next_waypoint = self.explorer.pick_next_waypoint(robot_x_mm, robot_y_mm, path, startPath, min_step_mm=300)
            if next_waypoint:
                print(f"Next_waypoint to {target_gx, target_gy}: {next_waypoint} ")
                move = self.explorer.compute_move_mm(robot_x_mm, robot_y_mm, next_waypoint)
                print("Proposed move:", move)
                next_move = self.veto.adjust_move(move, self.obstacles)
                if next_move:
                    break

       # ------------------------------------------------------------
        # 9. Return single next move
        # ------------------------------------------------------------
        if next_move:
            world_bearing = self.slam.convert_bearing_from_slam_frame(next_move[0])
            next_move = (world_bearing, next_move[1]/10.0)  # convert mm to cm
            print(f"Next move (bearing, distance_mm): {next_move}")
            return next_move
        else:
            # No safe move available - Let the robot decide what to do (e.g. rotate in place)
            # return (config.NO_SAFE_DIRECTION, 0)  # No direction, no distance  
            print("No safe move found, returning NO_SAFE_DIRECTION") 
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
