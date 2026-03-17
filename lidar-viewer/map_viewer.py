
from PyQt5 import QtWidgets

from world_map_widget import WorldMapWidget   # your existing widget


class OfflineViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        # self.setFocusPolicy(QtCore.Qt.StrongFocus)
        # self.setFocus()
        # Load data

        # Create map widget
        self.map_widget = WorldMapWidget(
            resolution_m=0.02,
            parent=self
        )
        
        self.map_widget.update_view()
        self.setCentralWidget(self.map_widget)
        
        self.activateWindow()
        self.raise_()
        # self.setFocus()




app = QtWidgets.QApplication([])
viewer = OfflineViewer()
viewer.show()
app.exec_()
