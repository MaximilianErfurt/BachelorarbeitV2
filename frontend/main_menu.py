from PyQt5.QAxContainer import QAxBase
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget, QMainWindow, QPushButton, QLabel, QGridLayout, QApplication
import sys
from backend.mono_camera import MonoCamera
from backend.stereo_camera import StereoCamera
from extrinsic_calibration_gui import ExtrinsicCalibrationGUI
from intrinsic_calibration_gui import IntrinsicCalibrationGUI
class MainMenu(QWidget):
    def __init__(self, stereo_cam: StereoCamera):
        super().__init__()
        self.stereo_cam = stereo_cam
        # subwindows
        self.extrinsic_calibration_window_left = None
        self.extrinsic_calibration_window_right = None
        self.intrinsic_calibration_window_left = None
        self.intrinsic_calibration_window_right = None

        # Add Labels
        self.left_camera_label = QLabel("Mono Kamera Links", self)
        self.left_camera_label.setAlignment(Qt.AlignCenter)
        self.left_camera_label.setFixedSize(200,20)
        self.right_camera_label = QLabel("Mono Kamera Rechts", self)
        self.right_camera_label.setAlignment(Qt.AlignCenter)
        self.right_camera_label.setFixedSize(200,20)

        # Add Buttons
        self.button_left_camera_intrinsic = QPushButton("Intrinsiche Kalibrierung")
        self.button_right_camera_intrinsic = QPushButton("Intrinsische Kalibrierung")
        self.button_right_camera_extrinsic = QPushButton("Extrinsische Kalibrierung")
        self.button_left_camera_extrinsic = QPushButton("Extrinische Kalibrierung")
        self.button_find_core = QPushButton("Start")

        # connect Signals
        self.button_find_core.clicked.connect(self.call_start)

        self.button_left_camera_intrinsic.clicked.connect(self.call_intrinsic_calibration_left)
        self.button_left_camera_extrinsic.clicked.connect(self.call_extrinsic_calibration_left)

        self.button_right_camera_intrinsic.clicked.connect(self.call_intrinsic_calibration_right)
        self.button_right_camera_extrinsic.clicked.connect(self.call_extrinsic_calibration_right)

        # create Grid
        layout = QGridLayout()

        # add Widgets
        layout.addWidget(self.button_find_core, 0, 1)

        layout.addWidget(self.left_camera_label, 1, 0)
        layout.addWidget(self.button_left_camera_intrinsic, 2, 0)
        layout.addWidget(self.button_left_camera_extrinsic, 3, 0)

        layout.addWidget(self.right_camera_label, 1, 2)
        layout.addWidget(self.button_right_camera_intrinsic, 2, 2)
        layout.addWidget(self.button_right_camera_extrinsic, 3, 2)

        self.setLayout(layout)
        self.setWindowTitle("Apfelkernhaus Ausstecher")
        self.setGeometry(100, 100, 400, 100)


    def call_intrinsic_calibration_left(self):
        self.intrinsic_calibration_window_left = IntrinsicCalibrationGUI(self, self.stereo_cam.camera_left)
        self.intrinsic_calibration_window_left.exec()

    def call_intrinsic_calibration_right(self):
        self.intrinsic_calibration_window_right = IntrinsicCalibrationGUI(self, self.stereo_cam.camera_right)
        self.intrinsic_calibration_window_right.exec()

    def call_extrinsic_calibration_left(self):
        self.extrinsic_calibration_window_left = ExtrinsicCalibrationGUI(self, self.stereo_cam.camera_left)
        self.extrinsic_calibration_window_left.exec()


    def call_extrinsic_calibration_right(self):
        self.extrinsic_calibration_window_right = ExtrinsicCalibrationGUI(self, self.stereo_cam.camera_right)
        self.extrinsic_calibration_window_right.exec()


    def call_start(self):
        pass

    def hide_window(self):
        """
        Hide Window
        :return:
        """
        self.hide()

    def show_window(self):
        """
        Show Window
        :return:
        """
        self.show()

    def close_window(self):
        """
        Close Window
        :return:
        """
        self.close()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainMenu()
    window.show()
    sys.exit(app.exec())