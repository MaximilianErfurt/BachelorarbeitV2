import cv2
import numpy as np
from scipy.spatial import distance_matrix
class Image:
    def __init__(self, name, image):
        self.name = name
        self.image = image
        self.cb_image = image
        self.imgp = None
        self.objp = None


    def show_image(self):
        cv2.imshow("image", self.image)
        cv2.waitKey(0)

    def find_chessboard(self, square_size = 2.55):
        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = np.zeros((6 * 9, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2) * square_size

        ret, self.imgp = cv2.findChessboardCorners(self.image, (9, 6), None)
        if ret:
            corners = cv2.cornerSubPix(self.cb_image, self.imgp, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(self.cb_image, (9, 6), corners, ret)
            #cv2.imshow("image", self.image)
            #cv2.waitKey(0)
            return self.imgp, self.objp
        else:
            print("no chessboard found")
            return False
