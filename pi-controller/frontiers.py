import math
import heapq

from move_veto import MoveVeto
import config

class Explorer:
    def __init__(self, resolution_m):
        self.res = resolution_m  # meters per cell

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
    def find_frontiers(self, grid):
        h, w = grid.shape
        frontiers = []

        for y in range(h):
            for x in range(w):

                # 1. Free cell test (tight)
                if grid[y, x] != config.FREE:
                    continue

                # 2. Must have at least one unknown neighbour
                unknown_neighbour = False
                for nx, ny in self.neighbors(x, y, w, h):
                    if grid[ny, nx] == config.UNKNOWN:
                        unknown_neighbour = True
                        break

                if not unknown_neighbour:
                    continue

                # 3. Must NOT be too close to obstacles
                obstacle_nearby = False
                for nx, ny in self.neighbors(x, y, w, h):
                    if grid[ny, nx] == config.OCCUPIED:
                        obstacle_nearby = True
                        break

                if obstacle_nearby:
                    continue

                frontiers.append((x, y))

        return frontiers

    # ---------------------------------------------------------
    # Step 2: Cluster frontier cells
    # ---------------------------------------------------------
    def cluster_frontiers(self, frontiers):
        clusters = []
        visited = set()
        frontier_set = set(frontiers)

        for cell in frontiers:
            if cell in visited:
                continue

            queue = [cell]
            cluster = []

            while queue:
                cx, cy = queue.pop()
                if (cx, cy) in visited:
                    continue

                visited.add((cx, cy))
                cluster.append((cx, cy))

                for nx, ny in self.neighbors(cx, cy, 99999, 99999):
                    if (nx, ny) in frontier_set and (nx, ny) not in visited:
                        queue.append((nx, ny))

            clusters.append(cluster)

        return clusters

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
            rx = robot_x_mm / 1000.0 / self.res
            ry = robot_y_mm / 1000.0 / self.res

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

    # ---------------------------------------------------------
    # Step 5: Convert path to robot moves
    # ---------------------------------------------------------
    def path_to_moves(self, path):
        moves = []
        if not path:
            return moves

        def to_world(ix, iy):
            x_m = ix * self.res
            y_m = iy * self.res
            return x_m * 1000, y_m * 1000  # mm

        prev_x, prev_y = to_world(*path[0])

        for ix, iy in path[1:]:
            x, y = to_world(ix, iy)
            dx = x - prev_x
            dy = y - prev_y

            # Map-frame bearing (mathematical)
            map_bearing = math.degrees(math.atan2(dy, dx))

            # Convert to compass absolute bearing
            compass_bearing = (90 - map_bearing) % 360

            distance = math.sqrt(dx*dx + dy*dy)

            moves.append((compass_bearing, distance))

            prev_x, prev_y = x, y

        return moves

    # ---------------------------------------------------------
    # Main API: get next exploration moves
    # ---------------------------------------------------------
    def get_exploration_moves(self, grid, robot_x_mm, robot_y_mm):
        frontiers = self.find_frontiers(grid)
        print("Detected frontiers:", len(frontiers))
        if not frontiers:
            return []

        clusters = self.cluster_frontiers(frontiers)
        target = self.pick_best_frontier(clusters, robot_x_mm, robot_y_mm)
        if not target:
            return []

        # Convert robot pose to grid coords
        sx = int(robot_x_mm / 1000.0 / self.res)
        sy = int(robot_y_mm / 1000.0 / self.res)

        path = self.astar(grid, (sx, sy), target)
        if not path:
            return []

        return self.path_to_moves(path)


class ExplorationManager:
    def __init__(self, slam, resolution_m=0.02):
        self.slam = slam
        self.explorer = Explorer(resolution_m)
        self.veto = MoveVeto(slam, resolution_m)

        self.cached_moves = []
        self.obstacles = []   # ultrasonic obstacles from robot

    # ---------------------------------------------------------
    # SLAM update (called continuously while robot is stationary)
    # ---------------------------------------------------------
    def update_map(self, scan_mm, imu_theta):
        self.slam.update(scan_mm, imu_theta)

    # ---------------------------------------------------------
    # Robot sends ultrasonic sweep results
    # ---------------------------------------------------------
    def receive_obstacles(self, obstacles):
        self.obstacles = obstacles

    # ---------------------------------------------------------
    # Main API: robot requests next move
    # ---------------------------------------------------------
    def get_next_move(self):
        grid = self.slam.get_map()
        x, y, th = self.slam.get_pose()

        moves = self.explorer.get_exploration_moves(grid, x, y)

        # 3. Check moves to see if they are vetoed by either ultrasonic or LIDAR obstacles, or if they are too long
        while moves:
            move = moves.pop(0)
            print("Proposed move:", move)
            adjusted = self.veto.adjust_move(move, self.obstacles)
            if adjusted:
                return adjusted
 
        # No safe move available - Let the robot decide what to do (e.g. rotate in place)
        # return (config.NO_SAFE_DIRECTION, 0)  # No direction, no distance   
        return (config.NO_DIRECTION, 0)  # No direction, no distance   
