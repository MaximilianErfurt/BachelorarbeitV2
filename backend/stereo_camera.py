import threading

import cv2
import depthai as dai

from mono_camera import MonoCamera
from image import Image
class StereoCamera:
    def __init__(self, camera_left:MonoCamera, camera_right:MonoCamera):
        self.camera_left = camera_left
        self.camera_right = camera_right
        self.image_left = None
        self.image_right = None

        # stereo calibration
        self.calibration_images_left = []
        self.calibration_images_right = []

    def stereo_calibration(self):
        imgp_left = []
        imgp_right = []
        objps = []

        for i in range(len(self.calibration_images_left)):
            imgp_left.append(self.calibration_images_left[i].imgp)
            imgp_right.append(self.calibration_images_right[i].imgp)
            objps.append(self.calibration_images_left[i].objp)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
        ret, CM1, dist1, CM2, dist2, R, T, E, F = cv2.stereoCalibrate(objps, imgp_left, imgp_right, self.camera_left.camera_matrix, self.camera_left.distortion_coefficients,
                                                                      self.camera_right.camera_matrix, self.camera_right.distortion_coefficients, self.calibration_images_left[0].image.shape,
                                                                      criteria = criteria, flags = cv2.CALIB_FIX_INTRINSIC)
        print(R)

    def take_stereo_calibration_images(self, counter):
        left_image, right_image = self.take_synced_images()
        left_image = Image("left", left_image)
        right_image = Image("right", right_image)
        ret_left = left_image.find_chessboard()
        ret_right = right_image.find_chessboard()
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
        self.image_left = Image("left_image", image_left)
        self.image_right = Image("right_image", image_right)



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


