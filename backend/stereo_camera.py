import threading
import json
import cv2
import depthai as dai
import numpy as np
from rtde_control import RTDEControlInterface
import socket
import struct

from mono_camera import MonoCamera
from image import Image
class StereoCamera:
    def __init__(self,camera_left:MonoCamera, camera_right:MonoCamera, name = "stereo_camera"):
        self.name = name
        self.camera_left = camera_left
        self.camera_right = camera_right
        self.image_left = None
        self.image_right = None

        # stereo calibration
        self.calibration_images_left = []
        self.calibration_images_right = []
        self.base_line = 75
        self.cl_R_cr = None
        self.cl_T_cr = None
        self.load_camera_config()

        self.p_tool = None  # found cut out point

    def stereo_calibration(self):
        """
        calibrate the stereo camera
        get rotational matrix, translation vektor and length of the baseline from camera_left to camera_right
        :return:
        """
        imgp_left = []
        imgp_right = []
        objps = []
        # add all the imgp and objp to the arrays
        for i in range(len(self.calibration_images_left)):
            imgp_left.append(self.calibration_images_left[i].imgp)
            imgp_right.append(self.calibration_images_right[i].imgp)
            objps.append(self.calibration_images_left[i].objp)
        # calibrate the stereo camera
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
        ret, CM1, dist1, CM2, dist2, self.cl_R_cr, self.cl_T_cr, E, F = cv2.stereoCalibrate(objps, imgp_left, imgp_right, self.camera_left.camera_matrix, self.camera_left.distortion_coefficients,
                                                                      self.camera_right.camera_matrix, self.camera_right.distortion_coefficients, self.calibration_images_left[0].image.shape,
                                                                      criteria = criteria, flags = cv2.CALIB_FIX_INTRINSIC)
        # update camera parameters
        self.camera_left.camera_matrix = CM1
        self.camera_left.distortion_coefficients = dist1
        self.camera_right.camera_matrix = CM2
        self.camera_right.distortion_coefficients = dist2

        self.base_line = np.linalg.norm(self.cl_T_cr)
        self.save_camera_config()

    def take_stereo_calibration_images(self, counter):
        left_image, right_image = self.take_synced_images()
        left_image = Image("left", left_image)
        right_image = Image("right", right_image)
        ret_left = left_image.find_chessboard(square_size= 20, checkerboard_size= (7,9))
        ret_right = right_image.find_chessboard(square_size= 20, checkerboard_size= (7,9))
        if not ret_left and not ret_right:
            print("no Checkerboard")
        if len(self.calibration_images_right) > counter and len(self.calibration_images_left) > counter:
            self.calibration_images_left[counter] = left_image
            self.calibration_images_right[counter] = right_image
            print("replaced")
        else:
            self.calibration_images_left.append(left_image)
            self.calibration_images_right.append(right_image)
            print("appended")

    def take_synced_images(self):
        """
                Take images from both cameras
                :return: left image, right image
                """
        pipeline = dai.Pipeline()
        # left camera
        cam_left = pipeline.createMonoCamera()
        cam_left.setBoardSocket(self.camera_left.socket)
        cam_left.setResolution(self.camera_left.resolution)
        cam_left.initialControl.setManualExposure(self.camera_left.exposure, self.camera_left.iso)

        # right camera
        cam_right = pipeline.createMonoCamera()
        cam_right.setBoardSocket(self.camera_right.socket)
        cam_right.setResolution(self.camera_right.resolution)
        cam_right.initialControl.setManualExposure(self.camera_right.exposure, self.camera_right.iso)

        # stream cam left
        left_out = pipeline.createXLinkOut()
        left_out.setStreamName("left_out")
        cam_left.out.link(left_out.input)

        # stream cam right
        right_out = pipeline.createXLinkOut()
        right_out.setStreamName("right_out")
        cam_right.out.link(right_out.input)

        with dai.Device(pipeline) as device:
            left_image = device.getOutputQueue(name="left_out", maxSize=1, blocking=True).get().getCvFrame()
            right_image = device.getOutputQueue(name="right_out", maxSize=1, blocking=True).get().getCvFrame()
            return left_image, right_image

    def take_mono_images(self):
        """
        take the images for blossom finding
        :return:
        """
        image_left, image_right = self.take_synced_images()
        image_left, image_right = self.rectify_images(image_left, image_right)
        self.image_left = Image("left_image", image_left)
        self.image_right = Image("right_image", image_right)

    def rectify_images(self, image_left, image_right):
        image_size = image_left.shape
        image_size = (int(image_size[1]), int(image_size[0]))
        cameraMatrix_left = self.camera_left.camera_matrix
        cameraMatrix_right = self.camera_right.camera_matrix
        distCoeffs_left = self.camera_left.distortion_coefficients.flatten().reshape(5,1)
        distCoeffs_right = self.camera_right.distortion_coefficients.flatten().reshape(5,1)
        # calculate rectification
        # R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(cameraMatrix_left, distCoeffs_left,
        #                                             cameraMatrix_right, distCoeffs_right,
        #                                             image_size, self.cl_R_cr, self.cl_T_cr)
        R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(
            cameraMatrix_left, distCoeffs_left,
            cameraMatrix_right, distCoeffs_right,
            image_size, self.cl_R_cr, self.cl_T_cr,
            flags=cv2.CALIB_ZERO_DISPARITY
        )

        # calculate rectification mapping
        map_left_x, map_left_y = cv2.initUndistortRectifyMap(cameraMatrix_left, distCoeffs_left, R1, P1, image_size, cv2.CV_32FC1)
        map_right_x, map_right_y = cv2.initUndistortRectifyMap(cameraMatrix_right, distCoeffs_right, R2, P2, image_size, cv2.CV_32FC1)
        # rectify images
        rect_left = cv2.remap(image_left, map_left_x, map_left_y, cv2.INTER_LINEAR)
        rect_right = cv2.remap(image_right, map_right_x, map_right_y, cv2.INTER_LINEAR)
        return rect_left, rect_right

    def calculate_depth(self):
        x2y2 = self.image_right.apples[0].get_image_coordinate()
        x2 = x2y2[1]
        x1y1 = self.image_left.apples[0].get_image_coordinate()
        x1 = x1y1[1]
        # transform right image coordinates into the coordinate system of camera_left
        lambda_x_1 = self.camera_left.camera_matrix[0][0]
        lambda_x_2 = self.camera_right.camera_matrix[0][0]
        # scale right image to same focal length
        x2_I = x2 # * (lambda_x_1/lambda_x_2)
        d = x1 - x2_I
        print(d)
        Z = lambda_x_1*(self.base_line/d)
        print(Z)
        return Z

    def find_cut_out_point(self):
        # def threads
        left_thread = threading.Thread(target=self.image_left.find_apples)
        right_thread = threading.Thread(target=self.image_right.find_apples)
        # start threads
        left_thread.start()
        right_thread.start()
        # wait for threads to finish
        left_thread.join()
        right_thread.join()
        z = self.calculate_depth()
        x, y = self.calculate_x_y(z)
        p_cam= [[x],
                [y],
                [z],
                [1]]
        self.transform_to_flange(p_cam)

    def calculate_x_y(self, z):
        x = 0
        y = 0
        P1 = self.image_left.apples[0].get_image_coordinate()
        P1 = [P1[1], P1[0], 1]
        mat = self.camera_left.camera_matrix
        mat_inv = np.linalg.inv(mat)
        [a, b, c] = mat_inv @ P1
        x = a * z
        y = b * z
        print(x,y)
        return x, y

    def transform_to_flange(self, p_cam):
        p_flange = self.camera_left.eye_hand_matrix @ p_cam
        print ("p_flange", p_flange)
        p_tool = p_flange #- [[0],
                           #  [0],
                            # [-314.5],
                             #[0]]
        return p_tool

    def send_position_to_robot(self):

        UR_IP = "192.168.1.2"
        PORT = 30004
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(2)
                s.connect((UR_IP, PORT))

                data = struct.pack("fff", self.p_tool[0], self.p_tool[1], self.p_tool[2])
                s.send(data)
                s.close()
            print("position sent")
        except socket.timeout:
            print("Timeout: robot not responding")
        except socket.error as e:
            print(f"Keine Verbindung: {e}")

    def save_camera_config(self):
        """
        save camera config
        :return:
        """
        data = {
            "cl_R_cr": self.cl_R_cr.tolist(),
            "cl_T_cr": self.cl_T_cr.tolist(),
            "base_line": self.base_line,

        }
        with open("config/" + self.name + "calibration_data.json", "w") as file:
            json.dump(data, file)

    def load_camera_config(self):
        """
        load camera config
        :return:
        """
        try:
            with open("config/" + self.name + "calibration_data.json", "r") as file:
                data = json.load(file)

                self.cl_R_cr = np.array(data["cl_R_cr"])
                self.cl_T_cr = np.array(data["cl_T_cr"])
                self.base_line = data["base_line"]
        except FileNotFoundError:
            print("camera config not found")

