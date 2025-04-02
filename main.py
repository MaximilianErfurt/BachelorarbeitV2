from backend.mono_camera import  MonoCamera
from backend.stereo_camera import StereoCamera
from frontend.main_menu import MainMenu
import depthai as dai
import sys
from PyQt5.QtWidgets import QApplication
from backend.robot import Robot

if __name__ == "__main__":
    left_cam = MonoCamera(dai.CameraBoardSocket.CAM_B, "left")
    right_cam = MonoCamera(dai.CameraBoardSocket.CAM_C, "right")
    stereo_cam = StereoCamera(left_cam, right_cam)
    robot = Robot('192.168.0.118')
    app = QApplication(sys.argv)
    main_menu = MainMenu(stereo_cam, robot)
    main_menu.show()
    app.aboutToQuit.connect(left_cam.save_camera_config)
    app.aboutToQuit.connect(right_cam.save_camera_config)
    sys.exit(app.exec())