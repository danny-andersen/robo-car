import glob
import os
import re

import numpy as np
import math
from PyQt5 import QtWidgets, QtGui, QtCore

from world_map_widget import WorldMapWidget   # your existing widget


class OfflineViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.setFocus()
        # Load data
        self.index = 0
        self.load_data()

        # Create map widget
        self.map_widget = WorldMapWidget(
            map_pixels=self.map.shape[0],
            maps=self.map_history,
            resolution_m=0.02
        )
        
        
        self.diff_mode = False
        self.setCentralWidget(self.map_widget)
        self.show_scans = True

        # Timer to animate robot path and map updates
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.nextPose)
        self.timer.start(2000)  # Update every 2 seconds
        
        self.activateWindow()
        self.raise_()
        self.setFocus()

    def load_data(self):
        self.map_history = self.load_map_history()
        if len(self.map_history) == 0:
            print("No map files found in the current directory.")
        else:
            self.map = self.map_history[self.index]
            self.map_shape = self.map.shape
            print("Number of set pixels:", np.sum(self.map > 0))
            print(np.unique(self.map, return_counts=True))  # Check unique values and their counts
        self.poses = np.loadtxt("slam_logs/poses.csv", delimiter=",")

        # self.scans = np.load("slam_logs/scans.npy", allow_pickle=True)
        # print("Number of scans:", len(self.scans))
        # print("Scan shape:", self.scans[0].shape)

        #Check that frontiers.npy exists and load it
        if os.path.exists("slam_logs/frontiers.npy"):
            self.frontiers = np.load("slam_logs/frontiers.npy", allow_pickle=True)
            print("Number of frontiers saved:", len(self.frontiers))
        else:
            print("frontiers.npy not found.")
            self.frontiers = []

    def load_map_history(self):
        files = glob.glob(f"slam_logs/map_*.npy")

        # Sort numerically by index
        files.sort(key=lambda f: int(re.findall(r"map_(\d+)\.npy", f)[0]))
        print(f"Found {len(files)} map files.")
        map_history = [np.load(f) for f in files]
        return map_history


    def nextPose(self):
        self.index = min(len(self.poses) - 1, self.index + 1)
        self.update_view()
        
    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Space:
            if self.timer.isActive():
                self.timer.stop()
            else:
                self.timer.start(2000)  # 0.5 FPS
        elif event.key() in (QtCore.Qt.Key_Return, QtCore.Qt.Key_Enter):
            self.index = 0
            self.load_data()  # Reload data to reset to initial state
            print("Reset to start")
        elif event.key() == QtCore.Qt.Key_Left:
                self.index = max(0, self.index - 1)
                self.timer.stop()
                self.update_view()
        elif event.key() == QtCore.Qt.Key_Right:
            min_index = len(self.map_history) - 1 if self.map_history else 0       
            self.index = min(min_index, self.index + 1)
            self.timer.stop()
            self.update_view()
        elif event.key() == QtCore.Qt.Key_D:
            self.diff_mode = not self.diff_mode
            self.update_view()


    def mousePressEvent(self, event):
        self.setFocus()
        super().mousePressEvent(event)

    def update_view(self):
        if self.index >= len(self.poses) or self.index >= len(self.map_history):
            return
        
        if len(self.poses.shape) == 1:    
            _, x, y, th = self.poses
        else:
            _, x, y, th = self.poses[self.index]
            
        # scan_mm = self.scans[self.index]
        # print(f"{self.index}: {np.argmin(scan_mm)} -> {np.argmax(scan_mm)}")
        # Convert scan to world coords only if needed
        # scan_world = self.scan_to_world(scan_mm, x, y, th) if self.show_scans else None 
        scan_world = None 

        if (len(self.frontiers) > self.index):
            clusters, targets = self.frontiers[self.index]
        else:
            clusters, targets = [], []

        self.map_widget.update_map_pose_scan_frontiers(
            self.index,
            self.diff_mode,
            (x, y, th),
            scan_world,
            clusters,
            targets
        )
        self.raise_()
        self.setFocus()


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



app = QtWidgets.QApplication([])
viewer = OfflineViewer()
viewer.show()
app.exec_()
