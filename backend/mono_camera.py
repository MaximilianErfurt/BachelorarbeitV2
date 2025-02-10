
import cv2
import depthai as dai
import numpy as np
import json
from image import Image
from helper import pose_to_transformation_matrix
from helper import r_vec_to_r_mat
from helper import r_mat_and_t_vec_to_t_mat
from helper import calc_x_T_x
from helper import calc_hand_eye_transformation

class MonoCamera:
    """
    Class for a monocular camera
    input: socket, name
    """
    def __init__(self, socket, name):
        self.name = name
        self.socket = socket
        self.resolution = dai.MonoCameraProperties.SensorResolution.THE_720_P
        self.intrinsic_calibrated = False
        self.extrinsic_calibrated = False
        # init camera specs
        self.exposure = 2500
        self.iso = 100
        self.camera_matrix = np.eye(3)
        self.distortion_coefficients = np.zeros(5)
        self.eye_hand_matrix = None

        # load camera specs
        self.load_camera_config()

        # extrinsic variables
        self.extrinsic_images = []
        self.extrinsic_rob_transformations = []

        # intrinsic variables
        self.intrinsic_images = []

    def take_single_image(self):
        """
        Take a single image from the camera
        :return: CV frame
        """
        # depthai pipeline
        pipeline = dai.Pipeline()

        # camera node
        cam = pipeline.createMonoCamera()
        cam.setBoardSocket(self.socket)
        cam.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        cam.initialControl.setManualExposure(self.exposure, self.iso)

        # output
        xout = pipeline.createXLinkOut()
        xout.setStreamName("image")
        cam.out.link(xout.input)

        # take a single image
        with dai.Device(pipeline) as device:
            image = device.getOutputQueue(name="image", maxSize=1, blocking=True).get().getCvFrame()
            return image

    def take_extrinsic_image(self, next_counter):
        """
        take an image for extrinsic calibration, find checkerboard and save
        :param next_counter:
        :return:
        """
        image = Image(self.name, self.take_single_image())
        ret = image.find_chessboard()
        if not ret:
            print("Schachbrettmuster nicht gefunden")
        if len(self.extrinsic_images) > next_counter:
            self.extrinsic_images[next_counter] = image
            print("replaced")
        else:
            self.extrinsic_images.append(image)
            print("appended")

    def add_extrinsic_rob_poses(self, rob_pose):
        transformation_matrix = pose_to_transformation_matrix(rob_pose)
        self.extrinsic_rob_transformations.append(transformation_matrix)

    def extrinsic_calibration(self):
        """
        finish the extrinsic calibration and calculate the hand eye transformation matrix
        :return:
        """
        # calc rvec and tvec
        objpoints = []
        imgpoints = []
        for image in self.extrinsic_images:
            objpoints.append(image.objp)
            imgpoints.append(image.imgp)
        size = self.extrinsic_images[0].image.shape[::-1]   # size of a single image in pixel

        ret, _, _, r_vecs, t_vecs = cv2.calibrateCamera(objpoints, imgpoints, size, self.camera_matrix, self.distortion_coefficients) # get rotational vectors and translational vectors
        print(r_vecs, t_vecs)

        # turn rotational vector in rotational matrix
        t_mats = []
        for i  in range(len(r_vecs)):
            print(r_vecs[i], r_vecs[i].shape)
            r_mat = r_vec_to_r_mat(r_vecs[i])     # transform rotational vector in rotational matrix
            t_mats.append(r_mat_and_t_vec_to_t_mat(r_mat, t_vecs[i]))       # combine rotational matrix and translational vector to transformation matrix

        # calc flansch i -> flansch i+1 matrices
        f_T_fs = calc_x_T_x(self.extrinsic_rob_transformations)

        # calc camera i -> camera i+1 matrices
        c_T_cs = calc_x_T_x(t_mats)

        # calc hand eye transformation
        self.eye_hand_matrix = calc_hand_eye_transformation(f_T_fs, c_T_cs)

    def take_intrinsic_image(self, next_counter):
        """
        take an image for intrinsic calibration, find checkerboard and save
        :param next_counter:
        :return:
        """
        image = Image(self.name, self.take_single_image())
        ret = image.find_chessboard()
        if not ret:
            print("schachbrettmuster nicht gefunden")
        if len(self.intrinsic_images) > next_counter:
            self.intrinsic_images[next_counter] = image
            print("replaced")
        else:
            self.intrinsic_images.append(image)
            print("appended")

    def intrinsic_calibration(self):
        objpoints = []      # 3D points of all taken images in real world
        imgpoints = []      # 2D points of all taken images in image plane

        # add all points of the taken images in a array for later calibration
        for image in self.intrinsic_images:
            imgpoints.append(image.imgp)    # add img points of image
            objpoints.append(image.objp)    # add obj points of image

        ret, self.camera_matrix, self.distortion_coefficients, _, _ = cv2.calibrateCamera(objpoints, imgpoints, self.intrinsic_images[0].image.shape[::-1], None, None)
        print(self.camera_matrix)

    def save_camera_config(self):
        """
        save camera config
        :return:
        """
        data = {
            "camera_matrix": self.camera_matrix.tolist(),
            "distortion_coefficients": self.distortion_coefficients.tolist(),
            "iso": self.iso,
            "exposure": self.exposure,
            "intrinsic_status": self.intrinsic_calibrated,
            "extrinsic_status": self.extrinsic_calibrated
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

                self.camera_matrix = np.array(data["camera_matrix"])
                self.camera_matrix = np.array(data["distortion_coefficients"])
                self.iso = data["iso"]
                self.exposure = data["exposure"]
                self.intrinsic_calibrated = data["intrinsic_status"]
                self.extrinsic_calibrated = data["extrinsic_status"]
        except FileNotFoundError:
            print("camera config not found")
