from PyQt5 import QtCore, QtGui, QtWidgets
import numpy as np
import math


class SlamMapWidget(QtWidgets.QWidget):
    """
    Robot‑centred SLAM map widget.
    - Robot stays fixed at the centre of the widget.
    - Map scrolls relative to robot movement.
    - Occupancy grid is drawn in world coordinates.
    """

    def __init__(self, map_pixels, resolution_m, parent=None):
        super().__init__(parent)

        self.map_pixels = map_pixels
        self.resolution = resolution_m  # meters per cell

        self._map = np.zeros((map_pixels, map_pixels), dtype=np.uint8)
        self._pose = (0.0, 0.0, 0.0)  # x_mm, y_mm, theta_rad
        self._map_image = None

        self.setMinimumSize(400, 400)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                           QtWidgets.QSizePolicy.Expanding)

    # ---------------------------------------------------------
    # Update map + pose
    # ---------------------------------------------------------
    def update_map_and_pose(self, occ_grid, pose_mm):
        self._map = occ_grid.astype(np.uint8)
        self._pose = pose_mm
        self._update_qimage()
        self.update()

    # ---------------------------------------------------------
    # Convert numpy → QImage
    # ---------------------------------------------------------
    def _update_qimage(self):
        h, w = self._map.shape
        data = self._map.copy().tobytes()
        self._map_image = QtGui.QImage(
            data, w, h, w, QtGui.QImage.Format_Grayscale8
        )

    # ---------------------------------------------------------
    # Paint event
    # ---------------------------------------------------------
    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        if self._map_image is None:
            painter.fillRect(self.rect(), QtCore.Qt.black)
            return

        widget_rect = self.rect()

        # Robot stays at the centre of the widget
        cx = widget_rect.width() / 2
        cy = widget_rect.height() / 2

        # Convert robot world pose → map grid coords
        x_mm, y_mm, theta = self._pose

        # Convert mm → meters
        x_m = x_mm / 1000.0
        y_m = y_mm / 1000.0

        # Convert meters → grid cells
        rx = x_m / self.resolution
        ry = y_m / self.resolution

        # Flip Y for image coordinates
        ry = self.map_pixels - ry

        # Convert robot grid coords → screen coords
        map_w = self._map_image.width()
        map_h = self._map_image.height()

        # Scale factor: map pixels → screen pixels
        scale_x = widget_rect.width() / map_w
        scale_y = widget_rect.height() / map_h
        scale = min(scale_x, scale_y)

        # Robot position in screen coords (before centering)
        robot_screen_x = rx * scale
        robot_screen_y = ry * scale

        # Compute translation to centre robot
        dx = cx - robot_screen_x
        dy = cy - robot_screen_y

        # Apply translation
        painter.translate(dx, dy)

        # Draw map scaled
        painter.scale(scale, scale)
        painter.drawImage(0, 0, self._map_image)

        # Reset transform for robot drawing
        painter.resetTransform()

        # Draw robot at centre
        painter.setPen(QtGui.QPen(QtCore.Qt.red, 2))
        painter.setBrush(QtGui.QBrush(QtCore.Qt.red))
        painter.drawEllipse(QtCore.QPointF(cx, cy), 6, 6)

        # Draw heading arrow
        arrow_len = 25
        hx = cx + arrow_len * math.cos(theta)
        hy = cy - arrow_len * math.sin(theta)
        painter.drawLine(QtCore.QPointF(cx, cy), QtCore.QPointF(hx, hy))
