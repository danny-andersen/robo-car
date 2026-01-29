import numpy as np
from PyQt5 import QtWidgets, QtGui, QtCore
from world_map_widget import WorldMapWidget   # your existing widget

class OfflineViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        # Load data
        self.map = np.load("map.npy")
        self.poses = np.loadtxt("poses.csv", delimiter=",")

        # Create map widget
        self.map_widget = WorldMapWidget(
            map_pixels=self.map.shape[0],
            resolution_m=0.02
        )
        self.setCentralWidget(self.map_widget)

        # Timer to animate robot path
        self.index = 0
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_view)
        self.timer.start(50)

    def update_view(self):
        if self.index >= len(self.poses):
            return

        _, x, y, th = self.poses[self.index]
        self.map_widget.update_map_and_pose(self.map, (x, y, th))
        self.index += 1


app = QtWidgets.QApplication([])
viewer = OfflineViewer()
viewer.show()
app.exec_()
