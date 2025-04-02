import socket
import time
import struct
import math
import numpy as np
from functions_UR import*

class Robot:
    def __init__(self, rob_ip):
        self.rob_ip = rob_ip
        self.p_resting = [-0.56596, 0.17438, 0.18912, 2.751, -1.503, -0.009]
        self.p_camera = [-0.56597, 0.17440, 0.04146, 2.751, -1.503, -0.009]
        self.p_pre_drop = [-0.65758, -0.30108, 0.29954, 2.232, -1.224, 0.397]
        self.p_drop = [-0.65826, -0.35267, 0.23073, 2.232, -1.223, 0.397]

    def move_resting_position(self):
        script = generate_urscript_movel(*self.p_resting, 0.1, 0.1)
        send_urscript(script, self.rob_ip)
        # Wait until the robot reaches the position
        while True:
            current_position = get_current_position(self.rob_ip)
            if has_reached_position(current_position, self.p_resting):
                break
            time.sleep(0.1)

    def move_camera_position(self):
        script = generate_urscript_movel(*self.p_camera, 0.1, 0.1)
        send_urscript(script, self.rob_ip)
        # Wait until the robot reaches the position
        while True:
            current_position = get_current_position(self.rob_ip)
            if has_reached_position(current_position, self.p_camera):
                break
            time.sleep(0.1)
    def move_pre_drop(self):
        script = generate_urscript_movel(*self.p_pre_drop, 0.1, 0.1)
        send_urscript(script, self.rob_ip)
        # Wait until the robot reaches the position
        while True:
            current_position = get_current_position(self.rob_ip)
            if has_reached_position(current_position, self.p_pre_drop):
                break
            time.sleep(0.1)

    def move_drop(self):
        script = generate_urscript_movel(*self.p_drop, 0.1, 0.1)
        send_urscript(script, self.rob_ip)
        # Wait until the robot reaches the position
        while True:
            current_position = get_current_position(self.rob_ip)
            if has_reached_position(current_position, self.p_drop):
                break
            time.sleep(0.1)

    def move_l(self, pose):
        script = generate_urscript_movel(*pose, 0.1, 0.1)
        send_urscript(script, self.rob_ip)
        while True:
            current_position = get_current_position(self.rob_ip)
            if has_reached_position(current_position, pose):
                break
            time.sleep(0.1)

    def get_current_position(self):
        port = 30003  # Real-time data port
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self.rob_ip, port))
        data = s.recv(1108)
        s.close()

        # Extract the position data from the received packet correctly
        unpacked_data = struct.unpack('!6d', data[444:492])  # 6 double values starting at byte 444
        x, y, z, rx, ry, rz = unpacked_data

        return x, y, z, rx, ry, rz
    def get_current_tcp_force(self):
        pass




if __name__ == '__main__':
    ip = '172.28.178.77'
    robot = Robot(ip)
    robot.move_camera_position()
