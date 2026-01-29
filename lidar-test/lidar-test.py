#!/usr/bin/env python3
import sys
import numpy as np
import matplotlib

matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
from PyQt5 import QtWidgets, QtGui, QtCore, QtWidgets
from PyQt5.QtWidgets import QApplication

# from breezyslam.algorithms import RMHC_SLAM
# from breezyslam.sensors import Laser
from icp_slam_scan_to_map import ICP_SLAM
from world_map_widget import WorldMapWidget
from ld19_reader import LD19Reader

import config

# -----------------------------
# Visualization
# -----------------------------

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, slam, reader):
        super().__init__()

        self.slam = slam
        self.reader = reader

        self.map_widget = WorldMapWidget(map_pixels=slam.size, resolution_m=slam.res)

        self.setCentralWidget(self.map_widget)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_slam)
        self.timer.start(50)  # Update every 50 ms
        self.show()

    def update_slam(self):
        scan = self.reader.latest_scan
        if scan is None:
            return

        self.reader.latest_scan = None
        self.slam.update(scan)

        grid = self.slam.get_map()
        x, y, th = self.slam.get_pose()
        self.map_widget.update_map_and_pose(grid, (x, y, th))


if __name__ == '__main__':
    slam = ICP_SLAM(map_size_m=16.0, resolution=0.02)
    reader = LD19Reader(config.SERIAL_PORT)
    reader.start()

    app = QApplication(sys.argv)
    window = MainWindow(slam, reader)
    sys.exit(app.exec())
    
