import numpy as np
import math
import datetime

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

        self.map_pixels = self.parent.map_history[0].shape[0] if len(self.parent.map_history) > 0 else 500
        self.resolution = resolution_m      # meters per cell

        self.init_data()
        
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
        self.target_history = []
        self.status_history = []
        self.obstacle_history = []
        self.candidate_target_history = []
        self._map = np.zeros((self.map_pixels, self.map_pixels), dtype=np.uint8)
        self._pose = (0.0, 0.0, 0.0)        # x_mm, y_mm, theta_rad
        self._map_image = None
        self.show_grid = True
        self.show_bearing_line = True
        self.zoom = 1.0
        self.pan_x = 0
        self.pan_y = 0
        self.diff_mode = False
        self.show_scans = True
        
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
            self.index = 0
            self.parent.load_data()  # Reload data to reset to initial state
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

    def gain_to_color(self, gain, min_gain, max_gain):
        t = (gain - min_gain) / (max_gain - min_gain + 1e-6)
        r = int(255 * t)
        g = int(255 * (1 - t))
        return QColor(r, g, 0)  # red→green

    def draw_candidate_poses(self, painter, target_rect):

        if len(self.candidate_target_history) < self.index:            
            return

        candidate_poses = self.candidate_target_history[self.index]
        if candidate_poses is None or len(candidate_poses) == 0:
            return
        gains = [g for (_, _, g) in candidate_poses]
        min_gain, max_gain = min(gains), max(gains)

        for (px, py, gain) in self.candidate_poses:
            color = self.gain_to_color(gain, min_gain, max_gain)
            painter.setPen(QPen(color, 2))
            painter.setBrush(color)

            # world → map pixel → screen
            gx = px / self.resolution
            gy = (self.map_pixels - 1) - (py / self.resolution)
            sx, sy = self.convert_point_to_map_coords((gx, gy), target_rect)
            
            painter.drawEllipse(QPointF(sx, sy), 5, 5)


    def draw_cluster_centroids(self, painter, target_rect):
        for cluster in self.cluster_history[self.index]:
            print(f"Showing cluster of len {len(cluster)}")
            xs = [p[0] for p in cluster]
            ys = [p[1] for p in cluster]
            cx = self.resolution * sum(xs) / len(xs)
            cy = self.resolution * sum(ys) / len(ys)

            # world → map pixel → screen
            sx, sy = self.convert_point_to_map_coords((cx, cy), target_rect)

            painter.setPen(QPen(QColor(255, 0, 255), 2))
            painter.drawEllipse(QPointF(sx, sy), 6, 6)


    def draw_chosen_target(self, painter, target_rect):
        
        if len(self.target_history) > self.index:
            tx, ty = self.target_history[self.index]
            tx = tx * self.resolution
            ty = ty * self.resolution
            # print(f"Target is {tx, ty}")

            # world → map pixel
            sx, sy = self.convert_point_to_map_coords((tx,ty), target_rect)

            # print(f"Target coords {sx,sy}")
            pen = QPen(QColor(255, 0, 0))
            pen.setWidth(3)
            painter.setPen(pen)

            size = 10
            painter.drawLine(QLineF(sx - size, sy, sx + size, sy))
            painter.drawLine(QLineF(sx, sy - size, sx, sy + size))
            
        # else:
        #     print("No target set")

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
            f"{timestamp.strftime('%d/%m/%y %H:%M:%S')}"
        )
        painter.drawText(
            QPointF(target_rect.left() + 10, target_rect.top() + 40),
            f"{tempC/10}°C RH: {humidity/10}% Battery: {battperc:.1f}%")
        
        
    def convert_point_to_map_coords(self, point, target_rect):
        (x_m, y_m) = point
        
        # Meters → grid index
        gx = x_m / self.resolution
        gy = y_m / self.resolution

        # Flip Y because image coordinates have origin at top-left
        gy = self.map_pixels - 1 - gy

        # If robot is outside the map, don't draw it
        if not (0 <= gx < self.map_pixels and 0 <= gy < self.map_pixels):
            return

        # Grid index → screen coordinates inside target_rect
        sx = target_rect.left() + (gx / self.map_pixels) * target_rect.width()
        sy = target_rect.top() + (gy / self.map_pixels) * target_rect.height()
        
        return (sx, sy)
        
    def draw_bearing_line(self, painter, target_rect):
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

        rsx, rsy = self.convert_point_to_map_coords((rx_m, ry_m), target_rect)

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

        # Draw robot pose relative to this fixed map
        self._draw_robot(painter, target_rect)

        self.draw_scale_bar(painter, target_rect)
        self.draw_axis_labels(painter, target_rect)        

        if self.show_bearing_line:
            self.draw_bearing_line(painter, target_rect)
        self.draw_mouse_coordinates(painter, target_rect)
        self.draw_status(painter, target_rect)
        

        # Draw scan points
        # self._draw_scan(painter, target_rect)
        

    # ---------------------------------------------------------
    # Draw robot pose in global map coordinates
    # ---------------------------------------------------------
    def _draw_robot(self, painter, target_rect):
        x_mm, y_mm, theta = self._pose

        x_m = x_mm / 1000.0
        y_m = y_mm / 1000.0

        sx, sy = self.convert_point_to_map_coords((x_m, y_m), target_rect)
        # Draw robot body
        painter.setPen(QtGui.QPen(QtCore.Qt.red, 2))
        painter.setBrush(QtGui.QBrush(QtCore.Qt.red))
        painter.drawEllipse(QtCore.QPointF(sx, sy), 5, 5)

        # Draw heading
        arrow_len = 20
        hx = sx + arrow_len * math.cos(theta)
        hy = sy - arrow_len * math.sin(theta)
        painter.drawLine(QtCore.QPointF(sx, sy), QtCore.QPointF(hx, hy))
