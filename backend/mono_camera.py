import os
import cv2
import depthai as dai
import numpy as np
import json
from numpy import linalg
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

    def test_extrinsic_image(self, next_counter):
        """
        load an image for extrinsic calibration, find checkerboard and save
        :return:
        """
        image = Image(self.name, self.load_image(next_counter))
        ret = image.find_chessboard(square_size=2, checkerboard_size=(7, 9))
        if not ret:
            print("Schachbrettmuster nicht gefunden")
        if len(self.extrinsic_images) > next_counter:
            self.extrinsic_images[next_counter] = image
            print("replaced")
        else:
            self.extrinsic_images.append(image)
            print("appended")


    def load_image(self, next_counter):
        #path = "C:/Users/Stevi/Desktop/Bachelorarbeit/Quellen/Camera_Calibration/Camera_Calibration/images"
        #file_name = f"p{next_counter + 1:02d}.png"
        path = "C:/Users/Stevi/PycharmProjects/BachelorarbeitV2/images/"
        file_name = str(next_counter) + ".png"
        image_path = os.path.join(path, file_name)
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        return image

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
        self.save_camera_config()
        self.extrinsic_calibrated = True

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
        self.intrinsic_calibrated = True
        self.save_camera_config()

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
            "extrinsic_status": self.extrinsic_calibrated,
            "eye_hand_matrix": self.eye_hand_matrix.tolist()
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
                self.distortion_coefficients = np.array(data["distortion_coefficients"])
                self.iso = data["iso"]
                self.exposure = data["exposure"]
                self.intrinsic_calibrated = data["intrinsic_status"]
                self.extrinsic_calibrated = data["extrinsic_status"]
                self.eye_hand_matrix = np.array(data["eye_hand_matrix"])
        except FileNotFoundError:
            print("camera config not found")

    def opencv_hand_eye_calibration(self):
        R_cb2cam, t_cb2cam = self.get_transformation_cb2cam()
        R_flange2base, t_flange2base = self.get_transformation_flange2base()
        R_cam2flange, t_cam2flange= cv2.calibrateHandEye(R_flange2base, t_flange2base, R_cb2cam, t_cb2cam, method=cv2.CALIB_HAND_EYE_TSAI)
        print(R_cam2flange)
        print(t_cam2flange)
        self.eye_hand_matrix = np.eye(4)
        self.eye_hand_matrix[:3, :3] = R_cam2flange
        self.eye_hand_matrix[:3, 3] = t_cam2flange.flatten()
        print(self.eye_hand_matrix)
        self.extrinsic_calibrated = True
        self.save_camera_config()

    def get_transformation_cb2cam(self):
        """
        Calculates the extrinsic parameters (rotation and translation vectors) for a set of calibration images.

        This function performs camera calibration using a set of object points (real-world 3D coordinates)
        and corresponding image points (2D pixel coordinates) to determine the rotation and translation vectors
        for each image in the dataset.

        Returns:
            tuple:
                - r_vecs (list of numpy arrays): Rotation vectors representing the orientation of the chessboard
                relative to the camera for each calibration image.
                - t_vecs (list of numpy arrays): Translation vectors representing the position of the chessboard
                relative to the camera for each calibration image.

        """
        # calc rvec and tvec
        objpoints = []
        imgpoints = []
        for image in self.extrinsic_images:
            objpoints.append(image.objp)
            imgpoints.append(image.imgp)
        size = self.extrinsic_images[0].image.shape[::-1]  # size of a single image in pixel

        ret, _, _, r_vecs, t_vecs = cv2.calibrateCamera(objpoints, imgpoints, size, self.camera_matrix,
                                                        self.distortion_coefficients)  # get rotational vectors and translational vectors
        if r_vecs is None or t_vecs is None:
            raise ValueError("cv2.calibrateCamera failed! Check objpoints and imgpoints.")
        r_vecs = list(r_vecs)
        for i in range(len(r_vecs)):
            R, _ = cv2.Rodrigues(r_vecs[i])
            # R_inv = linalg.inv(R)
            r, _ = cv2.Rodrigues(R)
            r_vecs[i] = r
        # t_vecs = [-t for t in t_vecs]
        return r_vecs, t_vecs

    def get_transformation_flange2base(self):
        """
            Extracts rotation and translation vectors from a list of 4x4 transformation matrices.

            This function iterates over a set of transformation matrices, extracts the rotation matrices
            and translation vectors, and converts the rotation matrices into rotation vectors using
            Rodrigues' formula.

            Returns:
                tuple:
                    - r_vecs (list of numpy arrays): Rotation vectors (Rodrigues format) derived from
                      the transformation matrices.
                    - t_vecs (list of numpy arrays): Translation vectors (3x1) extracted from the
                      transformation matrices.
            """
        r_vecs, t_vecs = [], []
        for T in self.extrinsic_rob_transformations:
            R = T[:3, :3]   # slice the rotation matrix
            # R_inv = linalg.inv(R)   # inverse R to get rotation from flange to base
            r,_ = cv2.Rodrigues(R)    # turn rotation matrix int rotational vector
            r_vecs.append(r)    # append to list
            t_vecs.append(T[:3, 3])  # slice t_vec and multiply with -1 to get flange to base

        return r_vecs, t_vecs


