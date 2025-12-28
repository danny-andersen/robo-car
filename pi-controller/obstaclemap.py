from PyQt5 import QtCore, QtGui, QtWidgets
import numpy as np
import math


class SlamMapWidget(QtWidgets.QWidget):
    """PyQt5 widget that displays an occupancy grid + robot pose."""

    def __init__(self, map_pixels, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 400)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                           QtWidgets.QSizePolicy.Expanding)

        self.map_pixels = map_pixels
        self._map = np.zeros((map_pixels, map_pixels), dtype=np.uint8)
        self._map_image = None

        # Robot pose (mm, mm, radians)
        self._pose = (0.0, 0.0, 0.0)

    # ---------------------------------------------------------
    # Update map + pose
    # ---------------------------------------------------------
    def update_map_and_pose(self, occ_grid, pose_mm):
        if occ_grid.shape != self._map.shape:
            raise ValueError("Occupancy grid has wrong shape")

        self._map = occ_grid.copy()
        self._pose = pose_mm

        self._update_qimage()
        self.update()

    # ---------------------------------------------------------
    # Convert numpy grid → QImage
    # ---------------------------------------------------------
    def _update_qimage(self):
        h, w = self._map.shape
        data = self._map.tobytes()
        self._map_image = QtGui.QImage(
            data, w, h, w, QtGui.QImage.Format_Grayscale8
        )

    # ---------------------------------------------------------
    # Paint event
    # ---------------------------------------------------------
    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, True)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)

        if self._map_image is None:
            painter.fillRect(self.rect(), QtCore.Qt.black)
            return

        widget_rect = self.rect()
        map_w = self._map_image.width()
        map_h = self._map_image.height()

        # Maintain aspect ratio
        target_rect = QtCore.QRectF(widget_rect)
        map_aspect = map_w / map_h
        widget_aspect = widget_rect.width() / widget_rect.height()

        if widget_aspect > map_aspect:
            new_w = widget_rect.height() * map_aspect
            new_h = widget_rect.height()
        else:
            new_w = widget_rect.width()
            new_h = widget_rect.width() / map_aspect

        target_rect.setWidth(new_w)
        target_rect.setHeight(new_h)
        target_rect.moveCenter(widget_rect.center())

        # Draw occupancy grid
        painter.drawImage(target_rect, self._map_image)

        # Draw robot pose
        self._draw_robot_pose(painter, target_rect)

    # ---------------------------------------------------------
    # Draw robot pose (circle + heading arrow)
    # ---------------------------------------------------------
    def _draw_robot_pose(self, painter, target_rect):
        x_mm, y_mm, theta = self._pose

        # Convert mm → map indices
        ix = x_mm / 1000.0 * (self.map_pixels / self.map_pixels)
        iy = y_mm / 1000.0 * (self.map_pixels / self.map_pixels)

        # Convert meters → grid cells
        ix = x_mm / 1000.0 * (self.map_pixels / self.map_pixels)
        iy = y_mm / 1000.0 * (self.map_pixels / self.map_pixels)

        # Convert mm → grid index
        ix = x_mm / 1000.0 * (self.map_pixels / self.map_pixels)
        iy = y_mm / 1000.0 * (self.map_pixels / self.map_pixels)

        # Actually: SLAM map uses mm directly → convert to grid cells
        ix = x_mm / (1000.0 / self.map_pixels)
        iy = y_mm / (1000.0 / self.map_pixels)

        # Flip Y for image coordinates
        iy = self.map_pixels - iy

        # Convert grid coords → screen coords
        sx = target_rect.left() + (ix / self.map_pixels) * target_rect.width()
        sy = target_rect.top() + (iy / self.map_pixels) * target_rect.height()

        # Draw robot body
        painter.setPen(QtGui.QPen(QtCore.Qt.red, 2))
        painter.setBrush(QtGui.QBrush(QtCore.Qt.red))
        painter.drawEllipse(QtCore.QPointF(sx, sy), 6, 6)

        # Draw heading arrow
        arrow_len = 25
        hx = sx + arrow_len * math.cos(theta)
        hy = sy - arrow_len * math.sin(theta)  # minus because screen y increases downward

        painter.drawLine(QtCore.QPointF(sx, sy), QtCore.QPointF(hx, hy))
