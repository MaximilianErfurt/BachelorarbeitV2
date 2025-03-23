import os
import cv2
import depthai as dai
import numpy as np
import json
from numpy import linalg

import helper
from image import Image
from helper import pose_to_transformation_matrix
from helper import r_vec_to_r_mat
from helper import r_mat_and_t_vec_to_t_mat
from helper import calc_x_T_x
from helper import load_image
from helper import save_image

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
        image = self.take_single_image()
        # save_image(next_counter, image)
        image = Image(self.name, image)
        image = cv2.undistort(image, self.camera_matrix, self.distortion_coefficients, None, self.camera_matrix)
        ret = image.find_chessboard(square_size=20, checkerboard_size=(7,9))
        if not ret:
            print("Schachbrettmuster nicht gefunden")
        if len(self.extrinsic_images) > next_counter:
            self.extrinsic_images[next_counter] = image
            print("replaced")
        else:
            self.extrinsic_images.append(image)
            print("appended")

    def load_extrinsic_image(self, next_counter):
        """
        load an image for extrinsic calibration, find checkerboard and save
        :return:
        """
        image = Image(self.name, load_image(next_counter))
        ret = image.find_chessboard(square_size=20, checkerboard_size=(7, 9))
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


    def calculate_hand_eye_matrix(self):
        # get transformation from camera KS to checkerboard KS
        c_r_cb_i, c_t_cb_i = self.get_r_vecs_and_t_vecs()           # rvecs and tvecs from cam to checkerboard  calculated with cv2.cameraCalibrate
        c_T_cb_i = []
        for i in range(len(c_r_cb_i)):
            c_T_cb_i.append(helper.r_and_t_to_T(c_r_cb_i[i], c_t_cb_i[i]))     # combine rvec and tvec to homogenous transformation

        # calculate transformation from camera i to camera 0
        c_T_c = []
        for i in range(1, len(c_T_cb_i)):
            try:
                c_T_ci = np.dot(c_T_cb_i[i], np.linalg.inv(c_T_cb_i[0]))  # Ci_T_Sb * SB_T_C0 = Ci_T_C0 [B]
                c_T_c.append(c_T_ci)
            except IndexError as e:
                print(e)

        # get transformation from flange KS to robot base KS
        b_T_f_i = self.extrinsic_rob_transformations        # homogenous transformation matrix from base to flange

        # calculate transformations for flange i to flange 0
        f_T_f = []
        for i in range(1, len(b_T_f_i)):
            try:
                f_T_fi = np.dot(np.linalg.inv(b_T_f_i[i]), b_T_f_i[0])    # Fi_T_B * B_T_F0 = Fi_T_F0 [A]
                f_T_f.append(f_T_fi)
            except IndexError as e:
                print(e)
        f_T_c, Dx, rx, DA_i, DB_i, rA_i, rB_i = helper.solve_ax_xb(f_T_f, c_T_c)

        # check solution
        # translation error
        e_t_i = []
        for i in range(len(DA_i)):
            e_t = np.dot(DA_i[i], rx) + np.transpose(rA_i[i][np.newaxis]) - np.dot(Dx, np.transpose(rB_i[i][np.newaxis])) -rx
            e_t_i.append(np.linalg.norm(e_t))
        translational_error = sum(e_t_i)/len(e_t_i)
        print("translational error:\n", str(translational_error) + "mm")
        # rotational error
        e_R_i = []
        for i in range(len(DA_i)):
            e_R = np.linalg.multi_dot([DA_i[i], Dx, np.linalg.inv(np.dot(Dx, DB_i[i]))])
            e_R_i.append(np.linalg.norm(cv2.Rodrigues(e_R)[0]))
        rotational_error = sum(e_R_i)/len(e_R_i)
        print("rotational error:\n", str(rotational_error) + "rad")
        print("rotational error: " + str(rotational_error * 180 / np.pi) + " degrees")





    def get_r_vecs_and_t_vecs(self):
        """
        Computes the rotation and translation vectors for extrinsic calibration images.

        :return: A tuple containing rotation vectors (r_vecs) and translation vectors (t_vecs).
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
        print(r_vecs, t_vecs)
        self.visualize_cb_coord_sys(r_vecs, t_vecs)
        return r_vecs, t_vecs

    def visualize_cb_coord_sys(self, r_vecs, t_vecs):
        images = self.extrinsic_images
        axis = np.float32([[3,0,0], [0,3,0], [0,0,3]]).reshape(-1,3)
        for i in range(len(r_vecs)):
            imgpts, _ = cv2.projectPoints(axis, r_vecs[i], t_vecs[i], self.camera_matrix, self.distortion_coefficients)
            image = cv2.cvtColor(images[i].cb_image, cv2.COLOR_GRAY2RGB)
            # Zeichne Achsen (rot = X, grün = Y, blau = Z)
            origin = tuple(imgpts[0].ravel().astype(int))
            cv2.line(image, origin, tuple(imgpts[1].ravel().astype(int)), (0, 255, 0), 2)
            cv2.line(image, origin, tuple(imgpts[2].ravel().astype(int)), (255, 0, 0), 2)
            cv2.line(image, (320, 240), origin, (0, 0, 255), 2)

            # Zeige das Bild
            cv2.imshow("Achsen", image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()






    #----------------------------------------------------
    # intrinsic calibration
    #----------------------------------------------------
    def take_intrinsic_image(self, next_counter):
        """
        take an image for intrinsic calibration, find checkerboard and save
        :param next_counter:
        :return:
        """
        image = Image(self.name, self.take_single_image())
        ret = image.find_chessboard(square_size= 20, checkerboard_size=(7,9) )
        if not ret:
            print("schachbrettmuster nicht gefunden")
        if len(self.intrinsic_images) > next_counter:
            self.intrinsic_images[next_counter] = image
            print("replaced")
        else:
            self.intrinsic_images.append(image)
            print("appended")

    def test_intrinsic_image(self, next_counter):
        """
        load an image for intrinsic calibration, find checkerboard and save
        :param next_counter:
        :return:
        """
        image = Image(self.name, load_image(next_counter))
        ret = image.find_chessboard(square_size=20, checkerboard_size=(7, 9))
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


    #----------------------------------------------------
    # opencv hand eye calibration (not working)
    #----------------------------------------------------

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
            R_inv = linalg.inv(R)
            r, _ = cv2.Rodrigues(R_inv)
            r_vecs[i] = r
        t_vecs = [-t for t in t_vecs]
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
            R_inv = linalg.inv(R)   # inverse R to get rotation from flange to base
            r,_ = cv2.Rodrigues(R_inv)    # turn rotation matrix int rotational vector
            r_vecs.append(r)    # append to list
            t_vecs.append(-1*T[:3, 3])  # slice t_vec and multiply with -1 to get flange to base

        return r_vecs, t_vecs


