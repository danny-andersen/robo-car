import glob
import os
import re
import json

import numpy as np
import math
from PyQt5 import QtWidgets, QtGui, QtCore

from world_map_widget import WorldMapWidget   # your existing widget


class OfflineViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        # self.setFocusPolicy(QtCore.Qt.StrongFocus)
        # self.setFocus()
        # Load data
        self.poses = []
        self.time_stamps = []
        self.map_history = []
        self.cluster_history = []
        self.target_history = []
        self.status_history = []
        self.obstacle_history = []
        self.map_widget = None
        self.load_data()

        # Create map widget
        self.map_widget = WorldMapWidget(
            resolution_m=0.02,
            parent=self
        )
        
        self.update_map_widget()
        self.map_widget.update_view()
        self.setCentralWidget(self.map_widget)
        
        self.activateWindow()
        self.raise_()
        # self.setFocus()

    def load_data(self):
        self.clear_data()
        if self.map_widget:
            self.map_widget.init_data()
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
        self.update_map_widget()
        
    def clear_data(self):
        self.poses = []
        self.time_stamps = []
        self.map_history = []
        self.cluster_history = []
        self.candidate_target_history = []
        self.target_history = []
        self.status_history = []
        self.obstacle_history = []
        if (self.map_widget):
            self.map_widget.init_data()
        
    def update_map_widget(self):
        # Update map widget
        if (self.map_widget):
            self.map_widget.map_history = self.map_history
            self.map_widget.poses = self.poses
            self.map_widget.time_stamps = self.time_stamps
            self.map_widget.cluster_history = self.cluster_history
            self.map_widget.target_history = self.target_history
            self.map_widget.status_history = self.status_history
            self.map_widget.obstacle_history = self.obstacle_history
            self.map_widget.candidate_target_history = self.candidate_target_history
        


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
            self.candidate_target_history.append(data["candidate_targets"])
            if data["chosen_target"] is not None:
                self.target_history.append(tuple(data["chosen_target"]))
            if data["robot_pose"] is not None:
                self.poses.append(tuple(data["robot_pose"]))
            if data["status"] is not None:
                self.status_history.append(tuple(data["status"]))
            self.obstacle_history.append(data["obstacles"])

        print(f"Frontier clusters: {self.cluster_history}")
        print(f"Candidate targets: {self.candidate_target_history}")
    # def load_cluster_history(self):
    #     files = glob.glob(f"slam_logs/clusters_*.npy")

    #     # Sort numerically by index
    #     files.sort(key=lambda f: int(re.findall(r"clusters_(\d+)\.npy", f)[0]))
    #     print(f"Found {len(files)} cluster files.")
    #     cluster_history = [np.load(f, allow_pickle=True) for f in files]
    #     return cluster_history




app = QtWidgets.QApplication([])
viewer = OfflineViewer()
viewer.show()
app.exec_()
