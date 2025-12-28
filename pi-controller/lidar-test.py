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
# from obstaclemap import SlamMapWidget
from slam_map_widget import SlamMapWidget
from world_map_widget import WorldMapWidget
from ld19_reader import LD19Reader

# -----------------------------
# Configuration
# -----------------------------
# SERIAL_PORT = "/dev/ttyUSB0"   # USB serial port on Linux
SERIAL_PORT = "/dev/ttyS0"   # GPIO serial port on Raspberry Pi


# SLAM/map parameters
MAP_SIZE_PIXELS = 800          # Occupancy grid width/height
MAP_SIZE_METERS = 16.0         # Map width/height in meters

# -----------------------------
# Visualization
# -----------------------------

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, slam, reader):
        super().__init__()

        self.slam = slam
        self.reader = reader

        # self.map_widget = SlamMapWidget(slam.size)
        # self.map_widget = SlamMapWidget( map_pixels=slam.size, resolution_m=slam.res )
        self.map_widget = WorldMapWidget(map_pixels=slam.size, resolution_m=slam.res)

        self.setCentralWidget(self.map_widget)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_slam)
        self.timer.start(50)
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
    reader = LD19Reader(SERIAL_PORT)
    reader.start()

    app = QApplication(sys.argv)
    window = MainWindow(slam, reader)
    sys.exit(app.exec())
    
# -----------------------------
# Main
# -----------------------------
# def main():

#     plotter = LivePlotter(MAP_SIZE_PIXELS, MAP_SIZE_METERS)

#     print("Starting LD19 SLAM. Press Ctrl+C to exit.")
#     try:
#         last_time = time.time()
#         while True:
#             scan = reader.latest_scan
#             if scan is not None:
#                 reader.latest_scan = None
#                 # Pass distances to SLAM (mm)
#                 slam.update(scan)
#                 # Get pose (x_mm, y_mm, theta_degrees)
#                 x_mm, y_mm, theta_deg = slam.get_pose()
#                 grid = slam.get_map()
#                 plotter.update(grid, (x_mm, y_mm, theta_deg))
#             else:
#                 time.sleep(0.01)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         reader.stop()

# if __name__ == "__main__":
#     main()
