from PyQt5 import QtCore, QtGui, QtWidgets
import numpy as np
import math


class WorldMapWidget(QtWidgets.QWidget):
    """
    Fixed global occupancy grid:
    - Map is drawn in a global frame and does not move.
    - Robot pose moves within the map.
    """

    def __init__(self, map_pixels, resolution_m, parent=None):
        super().__init__(parent)

        self.map_pixels = map_pixels        # number of grid cells per side
        self.resolution = resolution_m      # meters per cell

        self._map = np.zeros((map_pixels, map_pixels), dtype=np.uint8)
        self._pose = (0.0, 0.0, 0.0)        # x_mm, y_mm, theta_rad
        self._map_image = None

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

    # ---------------------------------------------------------
    # Numpy → QImage
    # ---------------------------------------------------------
    def _update_qimage(self):
        h, w = self._map.shape
        data = self._map.copy().tobytes()
        self._map_image = QtGui.QImage(
            data, w, h, w, QtGui.QImage.Format_Grayscale8
        )

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
