import numpy as np
import math
import datetime
import glob
import os
import re
import json


from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QImage, QColor, QLinearGradient, QPainter, QPen
from PyQt5.QtCore import QLineF, QPointF, QRectF, Qt


class WorldMapWidget(QtWidgets.QWidget):
    """
    Fixed global occupancy grid:
    - Map is drawn in a global frame and does not move.
    - Robot pose moves within the map.
    """

    def __init__(self, resolution_m, parent=None):
        super().__init__(parent)
        
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.setFocus()
        self.setMouseTracking(True)
        self.mouse_pos = None
        self.parent = parent

        self.setFocusPolicy(QtCore.Qt.ClickFocus)

        self.init_data()
        self.load_data()
        
        self.map_pixels = self.map_history[0].shape[0] if len(self.map_history) > 0 else 500
        print(f"Map pixels: {self.map_pixels}")
        self._map = np.zeros((self.map_pixels, self.map_pixels), dtype=np.uint8)
        self.resolution = resolution_m      # meters per cell

        # Timer to animate robot path and map updates
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.nextPose)
        self.timer.start(2000)  # Update every 2 seconds


        self.lut = self.build_qt_colormap()

        # self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        # self.slider.setMinimum(0)
        # self.slider.setMaximum(len(maps) - 1)
        # self.slider.setValue(0)
        # self.slider.valueChanged.connect(self.on_slider_change)
        # self.slider.setFocusPolicy(QtCore.Qt.NoFocus)

        self._scan_world = None   # Nx2 array of (wx_mm, wy_mm)

        self._clusters = None
        self._target = None

        self.setMinimumSize(400, 400)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                           QtWidgets.QSizePolicy.Expanding)


    def init_data(self):
        self.index = 0
        self.poses = []
        self.time_stamps = []
        self.map_history = []
        self.cluster_history = []
        self.slam_score_history = []
        self.target_history = []
        self.status_history = []
        self.obstacle_history = []
        self.candidate_target_history = []
        self.hover_candidate = None
        self._pose = (0.0, 0.0, 0.0)        # x_mm, y_mm, theta_rad
        self._map_image = None
        self.show_grid = True
        self.show_bearing_line = True
        self.show_ultrasonic = True
        self.show_candidates = True
        self.show_target = True
        self.zoom = 1.0
        self.pan_x = 0
        self.pan_y = 0
        self.diff_mode = False
        self.show_scans = True
        
    def load_data(self):
        self.map_history = self.load_map_history()
        if len(self.map_history) == 0:
            print("No map files found in the current directory.")
        # if os.path.exists("slam_logs/poses.csv"):
        #     self.poses = np.loadtxt("slam_logs/poses.csv", delimiter=",")
        # else:
        #     print("No pose file found.")

        # self.scans = np.load("slam_logs/scans.npy", allow_pickle=True)
        # print("Number of scans:", len(self.scans))
        # print("Scan shape:", self.scans[0].shape)

        #Check that clusters.npy exists and load it
        self.load_cluster_history()
        if len(self.cluster_history) == 0:
            print("No cluster files found in the current directory.")
        else:
            print("Number of cluster sets:", len(self.cluster_history))
            # print("Example cluster shape:", self.cluster_history[0].shape)
            # print("Example frontier targets shape:", self.frontiers[0][1].shape)
        # if os.path.exists("slam_logs/targets.csv"):
        #     self.targets = np.loadtxt("slam_logs/targets.csv", delimiter=",")
        # else:
        #     print("No target file found.")
        self.compute_global_gain_range()
        
    def load_map_history(self):
        files = glob.glob(f"slam_logs/map_*.npy")

        # Sort numerically by index
        files.sort(key=lambda f: int(re.findall(r"map_(\d+)\.npy", f)[0]))
        print(f"Found {len(files)} map files.")
        map_history = [np.load(f) for f in files]
        return map_history

    def load_cluster_history(self):
        files = glob.glob(f"slam_logs/clusters_*.json")

        # Sort numerically by index
        files.sort(key=lambda f: int(re.findall(r"clusters_(\d+)\.json", f)[0]))
        print(f"Found {len(files)} cluster files.")
        for file in files:
            with open(file, "r") as f:
                data = json.load(f)

            #Load json data if not none
            self.time_stamps.append(data["timestamp"])
            self.cluster_history.append(data["frontier_clusters"])
            self.slam_score_history.append(data["SLAM_scores"])
            self.candidate_target_history.append(data["candidate_targets"])
            if data["chosen_target"] is not None:
                self.target_history.append(tuple(data["chosen_target"]))
            if data["robot_pose"] is not None:
                self.poses.append(tuple(data["robot_pose"]))
            if data["status"] is not None:
                self.status_history.append(tuple(data["status"]))
            self.obstacle_history.append(data["obstacles"])

        # print(f"Frontier clusters: {self.cluster_history}")
        # print(f"Candidate targets: {self.candidate_target_history}")
    # def load_cluster_history(self):
    #     files = glob.glob(f"slam_logs/clusters_*.npy")

    #     # Sort numerically by index
    #     files.sort(key=lambda f: int(re.findall(r"clusters_(\d+)\.npy", f)[0]))
    #     print(f"Found {len(files)} cluster files.")
    #     cluster_history = [np.load(f, allow_pickle=True) for f in files]
    #     return cluster_history




    def nextPose(self):
        if len(self.poses) == 0:
            return
        self.index = min(len(self.poses) - 1, self.index + 1)
        self.update_view()
        

    def update_view(self):
        if self.index >= len(self.poses) or self.index >= len(self.map_history):
            return
        
        # print(f"Current index {self.index}")
        # if len(self.poses) == 1:    
        #     x, y, th = self.poses
        # else:
        x, y, th = self.poses[self.index]
            
        self._map = self.map_history[self.index]
        # print(f"No of Clusters: {len(self.cluster_history[self.index]) if len(self.cluster_history) > self.index else 'N/A'}")
       
        # scan_mm = self.scans[self.index]
        # print(f"{self.index}: {np.argmin(scan_mm)} -> {np.argmax(scan_mm)}")
        # Convert scan to world coords only if needed
        # scan_world = self.scan_to_world(scan_mm, x, y, th) if self.show_scans else None 
        self._scan_world  = None 


        self._pose = (x, y, th)
        # self.slider.setValue(map_index)
        if (self.diff_mode and self.index > 0):
            self._map_image = self.compute_map_diff(self.index - 1, self.index, self.lut)
        else:
            self._update_qimage(self._map)
        self.update()


    def scan_to_world(self, scan_mm, x_mm, y_mm, theta):
        angles = -np.deg2rad(np.arange(360))
        d = np.array(scan_mm, dtype=float)
        mask = (d > 50) & (d < 12000)

        if not np.any(mask):
            return np.zeros((0, 2))

        xr = d[mask] * np.cos(angles[mask])
        yr = d[mask] * np.sin(angles[mask])

        c = math.cos(theta)
        s = math.sin(theta)

        wx = x_mm + (c * xr - s * yr)
        wy = y_mm + (s * xr + c * yr)

        return np.vstack((wx, wy)).T

   
    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_G:
            self.show_grid = not self.show_grid
            self.update()
            event.accept()
        elif event.key() == QtCore.Qt.Key_B:
            self.show_bearing_line = not self.show_bearing_line
            self.update()
            event.accept()
        elif event.key() == QtCore.Qt.Key_Space:
            if self.timer.isActive():
                self.timer.stop()
            else:
                self.timer.start(2000)  # 0.5 FPS
        elif event.key() in (QtCore.Qt.Key_Return, QtCore.Qt.Key_Enter):
            self.init_data()
            self.load_data()  # Reload data to reset to initial state
            print("Reset to start")
            # Reset start of animation and reload data
            if self.timer.isActive():
                self.timer.stop()
            self.timer.start(2000)  # 0.5 FPS
            self.update_view()
        elif event.key() == QtCore.Qt.Key_Left:
                if self.index == 0:
                    self.index = len(self.map_history) - 1
                else:
                    self.index = max(0, self.index - 1)
                self.timer.stop()
                self.update_view()
        elif event.key() == QtCore.Qt.Key_Right:
            if self.index == len(self.map_history) - 1:
                self.index = 0
            else:
                min_index = len(self.map_history) - 1 if self.map_history else 0       
                self.index = min(min_index, self.index + 1)
            self.timer.stop()
            self.update_view()
        elif event.key() == QtCore.Qt.Key_D:
            self.diff_mode = not self.diff_mode
            self.update_view()
        elif event.key() == QtCore.Qt.Key_O:
            self.show_ultrasonic = not self.show_ultrasonic
            self.update()        
        elif event.key() == QtCore.Qt.Key_C:
            self.show_candidates = not self.show_candidates
            self.update()        
        elif event.key() == QtCore.Qt.Key_T:
            self.show_target = not self.show_target
            self.update()        
        else:
            super().keyPressEvent(event)  # Pass to parent for default handling
        
    def mousePressEvent(self, event):
        if event.button() == Qt.MiddleButton:
            self.last_pan = event.pos()
            event.accept()
        else:
            self.setFocus()
            super().mousePressEvent(event)

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta > 0:
            self.zoom *= 1.1
        else:
            self.zoom /= 1.1
        self.update()

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.MiddleButton:
            dx = event.x() - self.last_pan.x()
            dy = event.y() - self.last_pan.y()
            self.pan_x += dx
            self.pan_y += dy
            self.last_pan = event.pos()
            self.update()
        else:
            self.mouse_pos = event.pos()
            self.update()

    def on_slider_change(self, index):
        self.current_index = index
        self.update()

    def compute_global_gain_range(self):
        all_gains = []

        for pose_candidates in self.candidate_target_history:
            for _, gain in pose_candidates:
                all_gains.append(gain)

        if not all_gains:
            self.global_min_gain = 0
            self.global_max_gain = 1
            return

        self.global_min_gain = min(all_gains)
        self.global_max_gain = max(all_gains)
        
        if self.global_max_gain == self.global_min_gain:
            #No variation in gain - force it
            self.global_min_gain = self.global_max_gain / 2

    def gain_to_color(self, gain):
        t = (gain - self.global_min_gain) / (self.global_max_gain - self.global_min_gain + 1e-6)
        r = int(255 * t)
        g = int(255 * (1 - t))
        return QColor(r, g, 0)  # red→green

    def draw_candidate_poses(self, painter, target_rect):
        if not getattr(self, "show_candidates", False):
            return

        if len(self.candidate_target_history) < self.index:            
            return

        candidate_poses = self.candidate_target_history[self.index]
        if candidate_poses is None or len(candidate_poses) == 0:
            return
        gains = [g for (_, g) in candidate_poses]
        min_gain, max_gain = min(gains), max(gains)

        for (pose, gain) in candidate_poses:
            color = self.gain_to_color(gain)
            painter.setPen(QPen(color, 2))
            painter.setBrush(color)

            px = pose[0] / 1000.0
            py = pose[1] / 1000.0
            # world → map pixel → screen
            map_pos = self.convert_point_to_map_coords((px, py), target_rect)
            if map_pos:
                sx, sy = map_pos
                painter.drawEllipse(QPointF(sx, sy), 5, 5)


    def draw_hover_candidate(self, painter, target_rect):
        if not self.mouse_pos or  \
            not getattr(self, "show_candidates", False) or \
            len(self.candidate_target_history) < self.index:
            return False

        hover_candidate = None

        # How close the mouse must be to count as "hovering"
        hover_radius = 8
        candidate_poses = self.candidate_target_history[self.index]
        for (px, py), gain in candidate_poses:
            # Convert world → grid
            map_pos = self.convert_point_to_map_coords((px/1000.0, py/1000.0), target_rect)
            if map_pos:
                sx, sy = map_pos
            else:
                #Not on current map
                return False

            # Distance from mouse
            dx = self.mouse_pos.x() - sx
            dy = self.mouse_pos.y() - sy
            if dx*dx + dy*dy <= hover_radius * hover_radius:
                hover_candidate = (gain, sx, sy)
                break

        if not hover_candidate:
            return False

        gain, sx, sy = hover_candidate

        text = f"gain={gain}"

        painter.setPen(QColor(255, 0, 0))

        # Background box
        # fm = painter.fontMetrics()
        # w = fm.horizontalAdvance(text) + 8
        # h = fm.height() + 4
        # painter.drawRect(QPointF(sx + 10, sy - h - 10), w, h)

        # Text
        painter.drawText(QPointF(sx + 14, sy - 14), text)
        return True


    def draw_cluster_centroids(self, painter, target_rect):
        for cluster in self.cluster_history[self.index]:
            print(f"Showing cluster of len {len(cluster)}")
            xs = [p[0] for p in cluster]
            ys = [p[1] for p in cluster]
            cx = self.resolution * sum(xs) / len(xs)
            cy = self.resolution * sum(ys) / len(ys)

            # world → map pixel → screen
            map_pos = self.convert_point_to_map_coords((cx, cy), target_rect) 
            if map_pos:
                sx, sy = map_pos
            else:
                # Not on map
                return

            painter.setPen(QPen(QColor(255, 0, 255), 2))
            painter.drawEllipse(QPointF(sx, sy), 6, 6)


    def draw_chosen_target(self, painter, target_rect):
        if not getattr(self, "show_target", False):
            return
        
        if len(self.target_history) > self.index:
            tx, ty = self.target_history[self.index]
            tx = tx  / 1000
            ty = ty  / 1000
            # print(f"Target is {tx, ty}")

            # world → map pixel
            map_pos = self.convert_point_to_map_coords((tx,ty), target_rect)
            if map_pos:
                sx, sy = map_pos
                # print(f"Target coords {sx,sy}")
                pen = QPen(QColor(255, 0, 0))
                pen.setWidth(3)
                painter.setPen(pen)

                size = 10
                painter.drawLine(QLineF(sx - size, sy, sx + size, sy))
                painter.drawLine(QLineF(sx, sy - size, sx, sy + size))
            else:
                print(f"Target outside map?? {tx},{ty}")
            
        # else:
        #     print("No target set")

    def draw_ultrasonic_obstacles(self, painter, target_rect):
        if not getattr(self, "show_ultrasonic", False):
            return

        x_mm, y_mm, rtheta = self._pose

        rx = x_mm / 1000.0
        ry = y_mm / 1000.0
        
        pen = QPen(QColor(255, 255, 0, 180))
        pen.setWidth(2)
        painter.setPen(pen)

        for obs in self.obstacle_history[self.index]:
            bearing = obs.get("bearing", 0)
            width = obs.get("width", 0)
            dist_cm = obs.get("avgDistance", 0)

            # Skip invalid obstacles
            if width <= 0 or dist_cm <= 0:
                continue

            dist_m = dist_cm / 100.0

            # Convert to radians, allowing for 0 is straight up
            bearing_rad = math.radians(90 - bearing)
            half_width = math.radians(width) / 2.0

            # Arc start/end angles in world frame
            a1 = self.normalize_angle_rad(bearing_rad - half_width)
            a2 = self.normalize_angle_rad(bearing_rad + half_width)

            # print(f"Arc mid: {bearing}, distance: {dist_m} arc: {math.degrees(a1)} -> {math.degrees(a2)}")
            # Generate arc angles
            angles = self.generate_arc_angles(a1, a2, steps=20)

            # Draw arc as a polyline
            pts = []
            for ang in angles:
                wx = rx + dist_m * math.cos(ang)
                wy = ry + dist_m * math.sin(ang)

                # print(f"world {rx}, {ry} {math.degrees(ang)} {dist_m} -> {wx}, {wy}")
                screen = self.convert_point_to_map_coords((wx,wy), target_rect)
                
                if (screen):
                    sx, sy = screen
                    # print(f"map {sx, sy}")
                    pts.append(QPointF(sx, sy))
                else:
                    print(f"Nulling Arc outside of map: Arc mid: {bearing}, distance: {dist_m} arc: {math.degrees(a1)} -> {math.degrees(a2)}")
                    # print(f"point world {rx}, {ry} {math.degrees(ang)} {dist_m} -> {wx}, {wy}")
                    obs["avgDistance"] = 0
                    pts = []
                    break


            # Draw the arc
            for i in range(len(pts) - 1):
                painter.drawLine(pts[i], pts[i+1])

    def _draw_scan(self, painter, target_rect):
        if self._scan_world is None or len(self._scan_world) == 0:
            return

        h = self.map_pixels
        w = self.map_pixels

        for wx_mm, wy_mm in self._scan_world:
            # world mm → meters
            x_m = wx_mm / 1000.0
            y_m = wy_mm / 1000.0

            # meters → grid index
            gx = x_m / self.resolution
            gy = y_m / self.resolution

            # flip Y for display
            gy = h - 1 - gy

            if not (0 <= gx < w and 0 <= gy < h):
                continue

            # grid → screen
            sx = target_rect.left() + (gx / w) * target_rect.width()
            sy = target_rect.top() + (gy / h) * target_rect.height()

            painter.setPen(QtGui.QPen(QtGui.QColor(255, 0, 0), 2))
            painter.drawPoint(QtCore.QPointF(sx, sy))

    # ---------------------------------------------------------
    # Numpy → QImage
    # ---------------------------------------------------------
    def _update_qimage(self, map):
        h, w = map.shape
        data = map.copy().tobytes()
        # print(f"Updating QImage with shape {map.shape} and data length {len(data)}")
        self._map_image = QtGui.QImage(
            data, w, h, w, QtGui.QImage.Format_Grayscale8
        )

    def build_qt_colormap(self):
        # Create a 256×1 image to hold the gradient
        img = QImage(256, 1, QImage.Format_RGB32)

        # Define a Jet-like gradient
        grad = QLinearGradient(0, 0, 255, 0)
        grad.setColorAt(0.0, QColor(0, 0, 128))     # dark blue
        grad.setColorAt(0.25, QColor(0, 0, 255))    # blue
        grad.setColorAt(0.5, QColor(0, 255, 0))     # green
        grad.setColorAt(0.75, QColor(255, 255, 0))  # yellow
        grad.setColorAt(1.0, QColor(255, 0, 0))     # red

        painter = QPainter(img)
        painter.fillRect(img.rect(), grad)
        painter.end()

        # Extract LUT as list of QColor
        lut = [QColor(img.pixel(i, 0)) for i in range(256)]
        return lut

    def compute_map_diff(self, map1_index, map2_index, lut):
        """
        map1, map2: uint8 numpy arrays (grayscale)
        lut: list of 256 QColor entries
        returns: QImage with heatmap diff
        """

        # Absolute difference (NumPy only)
        diff = np.abs(self.map_history[map1_index].astype(np.int16) - self.map_history[map2_index].astype(np.int16)).astype(np.uint8)

        h, w = diff.shape
        img = QImage(w, h, QImage.Format_RGB32)

        # Apply LUT pixel-by-pixel
        for y in range(h):
            row = diff[y]
            for x in range(w):
                img.setPixelColor(x, y, lut[row[x]])

        return img


    def draw_grid(self, painter, target_rect):
        spacing_m = 0.5
        spacing_px = spacing_m / self.resolution

        # Convert spacing in map pixels → screen pixels
        spacing_screen_x = spacing_px * (target_rect.width() / self.map_pixels)
        spacing_screen_y = spacing_px * (target_rect.height() / self.map_pixels)

        pen = QPen(QColor(180, 180, 180, 120))
        pen.setWidth(1)
        painter.setPen(pen)

        # Vertical lines (North–South)
        x = target_rect.left()
        while x <= target_rect.right():
            painter.drawLine(QLineF(
                QPointF(x, target_rect.top()),
                QPointF(x, target_rect.bottom())
            ))
            x += spacing_screen_x

        # Horizontal lines (East–West)
        y = target_rect.top()
        while y <= target_rect.bottom():
            painter.drawLine(QLineF(
                QPointF(target_rect.left(), y),
                QPointF(target_rect.right(), y)
            ))
            y += spacing_screen_y

    def draw_scale_bar(self, painter, target_rect):
        # Length of scale bar in meters
        bar_m = 1.0

        # Convert meters → map pixels → screen pixels
        bar_px_map = bar_m / self.resolution
        bar_px_screen = bar_px_map * (target_rect.width() / self.map_pixels)

        # Position: bottom-left inside the map
        margin = 10
        x1 = target_rect.left() + margin
        y1 = target_rect.bottom() - margin
        x2 = x1 + bar_px_screen
        y2 = y1

        pen = QPen(QColor(255, 255, 0))
        pen.setWidth(3)
        painter.setPen(pen)
        painter.drawLine(QLineF(x1, y1, x2, y2))

        # Label
        painter.setPen(QColor(255, 255, 255))
        painter.drawText(QPointF(x1, y1 - 5), f"{bar_m:.0f} m")

    def draw_axis_labels(self, painter, target_rect):
        painter.setPen(QColor(255, 255, 0))
        font = painter.font()
        font.setPointSize(12)
        painter.setFont(font)
        margin = 20
        # North (top-center)
        painter.drawText(
            QPointF(target_rect.center().x(), target_rect.top() + margin),
            "N"
        )
        # South (bottom-center)
        painter.drawText(
            QPointF(target_rect.center().x(), target_rect.bottom() - margin),
            "S"
        )
        # West (left-center)
        painter.drawText(
            QPointF(target_rect.left() + margin, target_rect.center().y()),
            "W"
        )
        # East (right-center)
        painter.drawText(
            QPointF(target_rect.right() - margin, target_rect.center().y()),
            "E"
        )

    def draw_gain_legend(self, painter, target_rect):
        if not getattr(self, "show_candidates", False):
            return

        if len(self.candidate_target_history) == 0:            
            return


        # Legend size
        legend_width = 60
        legend_height = 160
        margin = 10

        # Position inside the map area (bottom-right corner of target_rect)
        x0 = target_rect.right() - legend_width - margin
        y0 = target_rect.bottom() - legend_height - margin
        # print(f"Max: {max_gain}, min: {min_gain}, x0 {x0}, y0{y0}")

        # Background box
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(0, 0, 0, 160))
        painter.drawRect(int(x0), int(y0), legend_width, legend_height)

        # Gradient bar area
        bar_x = x0 + 20
        bar_y = y0 + 25
        bar_w = 20
        bar_h = legend_height - 50

        # Draw continuous vertical gradient using gain_to_color()
        for i in range(bar_h):
            # t goes from 0 (top) to 1 (bottom)
            t = i / (bar_h - 1)

            # Map t back to a gain value
            gain = self.global_max_gain - t * (self.global_max_gain - self.global_min_gain)

            # Use your canonical color function
            color = self.gain_to_color(gain)

            painter.setPen(color)
            painter.drawLine(QLineF(bar_x, bar_y + i, bar_x + bar_w, bar_y + i))

        # Labels
        painter.setPen(QColor(255, 255, 255))
        painter.drawText(QPointF(bar_x + bar_w + 5, bar_y + 5), f"{self.global_max_gain:.0f}")
        painter.drawText(QPointF(bar_x + bar_w + 5, bar_y + bar_h), f"{self.global_min_gain:.0f}")

        # Title
        painter.drawText(QPointF(x0 + 10, y0 + 15), "Gain")


    def draw_mouse_coordinates(self, painter, target_rect):
        if self.mouse_pos is None:
            return

        mx, my = self.mouse_pos.x(), self.mouse_pos.y()

        # Check if mouse is inside the map
        if not target_rect.contains(mx, my):
            return

        # Convert screen → map pixel
        gx = (mx - target_rect.left()) / target_rect.width() * self.map_pixels
        gy = (my - target_rect.top()) / target_rect.height() * self.map_pixels

        # Convert map pixel → world meters
        x_m = gx * self.resolution
        y_m = (self.map_pixels - 1 - gy) * self.resolution

        # Robot pose in meters
        rx_m = self._pose[0] / 1000.0
        ry_m = self._pose[1] / 1000.0

        # Distance
        dx = x_m - rx_m
        dy = y_m - ry_m
        dist = (dx*dx + dy*dy) ** 0.5

        # Bearing (North = 0°, East = 90°)
        # dy_flipped = ry_m - y_m  # flip Y for compass orientation
        bearing_rad = math.atan2(dx, dy)  # atan2 returns angle from Y-axis
        bearing_deg = math.degrees(bearing_rad)
        if bearing_deg < 0:
            bearing_deg += 360

        # Draw text
        painter.setPen(QColor(255, 255, 0))
        painter.drawText(
            QPointF(target_rect.left() + 10, target_rect.top() + 60),
            f"x={x_m:.2f}m, y={y_m:.2f}m ({int(gx)},{int(gy)}) dist={dist:.2f}m, bearing={bearing_deg:.1f}°"
        )

    def draw_status(self, painter, target_rect):
        tempC, humidity, battery = self.status_history[self.index]
        #Max voltage is 8.1V, min is 6.3V
        battperc = 100*(battery/100.0-6.3)/(8.1-6.3)
        # timestamp = datetime.datetime(self.time_stamps[self.index])
        timestamp = datetime.datetime.fromtimestamp(self.time_stamps[self.index])
        painter.setPen(QColor(255, 255, 0))
        painter.drawText(
            QPointF(target_rect.left() + 10, target_rect.top() + 20),
            f"Index {self.index} at {timestamp.strftime('%d/%m/%y %H:%M:%S')}"
        )
        painter.drawText(
            QPointF(target_rect.left() + 10, target_rect.top() + 40),
            f"{tempC/10}°C RH: {humidity/10}% Battery: {battperc:.1f}%")

    def draw_slam_scores(self, painter, target_rect):
        slam_scores = self.slam_score_history[self.index]
        painter.setPen(QColor(255, 255, 0))
        painter.drawText(
            QPointF(target_rect.left() + 10, target_rect.top() + 80),
            # f"ICP score: {slam_scores[0]:0.4f}, Local reloc {slam_scores[1]:0.4f}, Global reloc {slam_scores[2]:0.4f}"
            f"ICP score: {slam_scores[0]:0.4f}, Global Coarse {slam_scores[2]:0.4f}, Med {slam_scores[3]:0.4f}, Fine {slam_scores[4]:0.4f}"
        )
        

    def generate_arc_angles(self, a1, a2, steps=20):
        if a1 <= a2:
            # normal arc
            return [a1 + (a2 - a1) * (i / steps) for i in range(steps + 1)]
        else:
            # wrap-around arc (e.g. 350° to 10°)
            return [
                self.normalize_angle_rad(a1 + ((a2 + 2*math.pi - a1) * (i / steps)))
                for i in range(steps + 1)
            ]

    def normalize_angle_rad(self, a):
        return a % (2 * math.pi)        
        
    def convert_point_to_map_coords(self, point, target_rect):
        (x_m, y_m) = point
        
        # Meters → grid index
        gx = x_m / self.resolution
        gy = y_m / self.resolution

        # Flip Y because image coordinates have origin at top-left
        gy = self.map_pixels - 1 - gy

        # If is outside the map, don't draw it
        if not (0 <= gx < self.map_pixels and 0 <= gy < self.map_pixels):
            print(f"{x_m},{y_m} -> {gx},{gy} is outside the map!")
            return

        # Grid index → screen coordinates inside target_rect
        sx = target_rect.left() + (gx / self.map_pixels) * target_rect.width()
        sy = target_rect.top() + (gy / self.map_pixels) * target_rect.height()
        # print(f"{x_m},{y_m} -> {gx},{gy}  -> {sx},{sy}")
        
        return (sx, sy)
        
    def draw_bearing_line(self, painter, target_rect, draw_coords):
        if self.mouse_pos is None:
            return

        mx, my = self.mouse_pos.x(), self.mouse_pos.y()

        # Only draw if mouse is inside the map
        if not target_rect.contains(mx, my):
            return

        # Convert mouse → map pixel
        gx = (mx - target_rect.left()) / target_rect.width() * self.map_pixels
        gy = (my - target_rect.top()) / target_rect.height() * self.map_pixels

        # Convert map pixel → world meters
        x_m = gx * self.resolution
        y_m = (self.map_pixels - 1 - gy) * self.resolution

        # Robot world pose to screen coords
        rx_m = self._pose[0] / 1000.0
        ry_m = self._pose[1] / 1000.0

        map_pos = self.convert_point_to_map_coords((rx_m, ry_m), target_rect)
        if map_pos:
            rsx, rsy = map_pos
        else:
            # Outside of map
            return

        # Bearing line end = mouse screen coords
        msx = mx
        msy = my

        # Draw line
        pen = QPen(QColor(255, 255, 0, 180))
        pen.setWidth(2)
        painter.setPen(pen)
        painter.drawLine(QLineF(rsx, rsy, msx, msy))

        # Draw arrowhead at mouse end
        self.draw_arrowhead(painter, rsx, rsy, msx, msy)

        # Compute bearing and distance for label
       
        if draw_coords: 
            dx = x_m - rx_m
            dy = y_m - ry_m
            dist = (dx*dx + dy*dy) ** 0.5
        
            bearing_rad = math.atan2(dx, dy)
            bearing_deg = math.degrees(bearing_rad)
            if bearing_deg < 0:
                bearing_deg += 360

            # Draw bearing label near cursor
            painter.setPen(QColor(255, 255, 0))
            painter.drawText(
                QPointF(msx + 10, msy - 10),
                f"({bearing_deg:.1f}°,{dist:.2f}m)"
            )        

    def draw_arrowhead(self, painter, x1, y1, x2, y2):
        # Arrowhead size
        size = 12.0

        # Direction vector
        angle = math.atan2(y2 - y1, x2 - x1)

        # Two arrowhead points
        left = QPointF(
            x2 - size * math.cos(angle - math.pi / 6),
            y2 - size * math.sin(angle - math.pi / 6)
        )
        right = QPointF(
            x2 - size * math.cos(angle + math.pi / 6),
            y2 - size * math.sin(angle + math.pi / 6)
        )

        # Draw arrowhead
        painter.drawLine(QLineF(x2, y2, left.x(), left.y()))
        painter.drawLine(QLineF(x2, y2, right.x(), right.y()))

    # ---------------------------------------------------------
    # Painting
    # ---------------------------------------------------------
    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        if self._map_image is None:
            painter.fillRect(self.rect(), QtCore.Qt.black)
            return

        widget_rect = self.rect()
        map_w = self._map_image.width()
        map_h = self._map_image.height()

        # Preserve aspect ratio
        scale_x = widget_rect.width() / map_w
        scale_y = widget_rect.height() / map_h
        scale = min(scale_x, scale_y)

        # Compute target rect where the map will be drawn
        draw_w = map_w * scale * self.zoom
        draw_h = map_h * scale * self.zoom
        
        left = widget_rect.left() + (widget_rect.width() - draw_w) / 2 + self.pan_x
        top  = widget_rect.top()  + (widget_rect.height() - draw_h) / 2 + self.pan_y

        target_rect = QRectF(left, top, draw_w, draw_h)        
        # left = widget_rect.left() + (widget_rect.width() - draw_w) / 2
        # top = widget_rect.top() + (widget_rect.height() - draw_h) / 2
        # target_rect = QtCore.QRectF(left, top, draw_w, draw_h)

        # Draw global map
        painter.drawImage(target_rect, self._map_image)

        # Draw the scale grid
        if self.show_grid:
            self.draw_grid(painter, target_rect)
        
        # self.draw_cluster_centroids(painter, target_rect)
        
        self.draw_candidate_poses(painter, target_rect)
        self.draw_chosen_target(painter, target_rect)
        self.draw_ultrasonic_obstacles(painter, target_rect)
        
        # Draw robot pose relative to this fixed map
        self._draw_robot(painter, target_rect)

        self.draw_scale_bar(painter, target_rect)
        self.draw_axis_labels(painter, target_rect)        
        self.draw_gain_legend(painter, target_rect)

        candidate_coords_shown = self.draw_hover_candidate(painter, target_rect)
        if self.show_bearing_line:
            self.draw_bearing_line(painter, target_rect, not candidate_coords_shown)

        self.draw_status(painter, target_rect)
        self.draw_mouse_coordinates(painter, target_rect)
        self.draw_slam_scores(painter, target_rect)
        

        # Draw scan points
        # self._draw_scan(painter, target_rect)
        

    # ---------------------------------------------------------
    # Draw robot pose in global map coordinates
    # ---------------------------------------------------------
    def _draw_robot(self, painter, target_rect):
        x_mm, y_mm, theta = self._pose

        x_m = x_mm / 1000.0
        y_m = y_mm / 1000.0

        map_pos = self.convert_point_to_map_coords((x_m, y_m), target_rect)
        if map_pos:
            sx, sy = map_pos
        else:
            #Robot outside of map
            return
        # Draw robot body
        painter.setPen(QtGui.QPen(QtCore.Qt.red, 2))
        painter.setBrush(QtGui.QBrush(QtCore.Qt.red))
        painter.drawEllipse(QtCore.QPointF(sx, sy), 5, 5)

        # Draw heading
        arrow_len = 20
        hx = sx + arrow_len * math.cos(theta)
        hy = sy - arrow_len * math.sin(theta)
        painter.drawLine(QtCore.QPointF(sx, sy), QtCore.QPointF(hx, hy))
