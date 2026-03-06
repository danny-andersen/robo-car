import numpy as np
import math

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QImage, QColor, QLinearGradient, QPainter


class WorldMapWidget(QtWidgets.QWidget):
    """
    Fixed global occupancy grid:
    - Map is drawn in a global frame and does not move.
    - Robot pose moves within the map.
    """

    def __init__(self, map_pixels, maps, resolution_m, parent=None):
        super().__init__(parent)
        
        self.setFocusPolicy(QtCore.Qt.NoFocus)

        self.map_pixels = map_pixels        # number of grid cells per side
        self.resolution = resolution_m      # meters per cell

        self._map = np.zeros((map_pixels, map_pixels), dtype=np.uint8)
        self._pose = (0.0, 0.0, 0.0)        # x_mm, y_mm, theta_rad
        self._map_image = None
        self.map_history = maps

        self.lut = self.build_qt_colormap()

        # self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        # self.slider.setMinimum(0)
        # self.slider.setMaximum(len(maps) - 1)
        # self.slider.setValue(0)
        # self.slider.valueChanged.connect(self.on_slider_change)
        # self.slider.setFocusPolicy(QtCore.Qt.NoFocus)

        self._scan_world = None   # Nx2 array of (wx_mm, wy_mm)

        self._clusters = None
        self._targets = None

        self.setMinimumSize(400, 400)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                           QtWidgets.QSizePolicy.Expanding)

    # ---------------------------------------------------------
    # Public API: update map and robot pose
    # ---------------------------------------------------------
    def update_map_and_pose(self, occ_grid, pose_mm):
        if occ_grid.shape != self._map.shape:
            raise ValueError("Occupancy grid has wrong shape")

        self._map = occ_grid.astype(np.uint8)
        self._pose = pose_mm
        
        self._update_qimage()
        self.update()

    def update_map_pose_scan_frontiers(self, map_index, diff_mode, pose_mm,  scan_world, clusters, targets):
 
        self._pose = pose_mm
        # self.slider.setValue(map_index)
        self._scan_world = scan_world
        self._clusters = clusters
        self._targets = targets
        self._map = self.map_history[map_index]
        if (diff_mode and map_index > 0):
            self._map_image = self.compute_map_diff(map_index - 1, map_index, self.lut)
        else:
            self._update_qimage(self._map)
        self.update()
   
    def on_slider_change(self, index):
        self.current_index = index
        self.update()


    def _draw_target_cluster(self, painter, target_rect):
        if not self._clusters:
            return

        h = self.map_pixels
        w = self.map_pixels

        painter.setPen(QtGui.QPen(QtGui.QColor(0, 255, 0), 3))

        for cluster in self._clusters:
            for (gx, gy) in cluster:
                sx = target_rect.left() + (gx / w) * target_rect.width()
                sy = target_rect.top() + (gy / h) * target_rect.height()
                painter.drawPoint(QtCore.QPointF(sx, sy))

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
        draw_w = map_w * scale
        draw_h = map_h * scale
        left = widget_rect.left() + (widget_rect.width() - draw_w) / 2
        top = widget_rect.top() + (widget_rect.height() - draw_h) / 2
        target_rect = QtCore.QRectF(left, top, draw_w, draw_h)

        # Draw global map
        painter.drawImage(target_rect, self._map_image)

        # Draw robot pose relative to this fixed map
        self._draw_robot(painter, target_rect)

        # Draw scan points
        # self._draw_scan(painter, target_rect)
        
        self._draw_target_cluster(painter, target_rect)
        # self._draw_frontier_targets(painter, target_rect)


    # ---------------------------------------------------------
    # Draw robot pose in global map coordinates
    # ---------------------------------------------------------
    def _draw_robot(self, painter, target_rect):
        x_mm, y_mm, theta = self._pose

        # World mm → meters
        x_m = x_mm / 1000.0
        y_m = y_mm / 1000.0

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

        # Draw robot body
        painter.setPen(QtGui.QPen(QtCore.Qt.red, 2))
        painter.setBrush(QtGui.QBrush(QtCore.Qt.red))
        painter.drawEllipse(QtCore.QPointF(sx, sy), 5, 5)

        # Draw heading
        arrow_len = 20
        hx = sx + arrow_len * math.cos(theta)
        hy = sy - arrow_len * math.sin(theta)
        painter.drawLine(QtCore.QPointF(sx, sy), QtCore.QPointF(hx, hy))
