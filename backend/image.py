import cv2
import numpy as np
from scipy.spatial import distance_matrix
class Image:
    def __init__(self, name, image):
        self.name = name
        self.image = image

        # param for blossom finding
        self.cropped_image = image[:, 200:1000]
        self.rgb_image = cv2.cvtColor(self.cropped_image, cv2.COLOR_GRAY2BGR)
        self.circle_image = self.rgb_image.copy()

        # param for calibration
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

    def find_blossom(self):
        """
        find the Blossom in the image
        :return:
        """
        # find the apple via hough circles
        circles = self.find_circles()
        cv2.imshow("window",self.circle_image)
        reduced_to_circle  =self.reduce_to_circle(circles)


    def find_circles(self):
        circles = cv2.HoughCircles(cv2.GaussianBlur(
            self.cropped_image, (3, 3), 0), cv2.HOUGH_GRADIENT, 1, 300, param1=80, param2=20, minRadius=20,
            maxRadius=200  #
        )
        if circles is not None:
            if len(circles) == 1:
                circles = np.uint16(np.around(circles))
                for circle in circles[0, :]:
                    # draw outer circle
                    cv2.circle(self.circle_image, (circle[0], circle[1]), circle[2], (0, 255, 0), 2)
                    # draw center
                    cv2.circle(self.circle_image, (circle[0], circle[1]), circle[2], (0, 0, 255), 3)
                return circles
            else:
                print("more then one circle found")
        else:
            print("no circles Found")


    def reduce_to_circle(self, circles):
        for circle in circles[0, : ]:
            # center of the circle
            center = (circle[0], circle[1])
            # radius of the circle - 30 to avoid inconsistency around the edges
            radius = circle[2]-30
            # mask to remove everything except the circle
            mask = np.zeros_like(self.rgb_image)
            cv2.circle(mask, center, radius, (255, 255, 255), -1)
            reduced_to_circle = cv2.bitwise_and(self.rgb_image, mask)
            cv2.imshow("window", reduced_to_circle)
            return reduced_to_circle




