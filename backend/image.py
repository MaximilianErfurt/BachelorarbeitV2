import math

import cv2
import numpy as np
from scipy.spatial import distance_matrix
from helper import  dbscan_clustering
from ultralytics import YOLO

class Image:
    def __init__(self, name, image):
        self.name = name
        self.image = image
        self.rgb_image = cv2.cvtColor(self.image, cv2.COLOR_GRAY2BGR)

        # param for blossom apple finding
        self.apples = []

        # param for calibration
        self.cb_image = self.rgb_image.copy()
        self.imgp = None
        self.objp = None



    def show_image(self):
        cv2.imshow("image", self.image)
        cv2.waitKey(0)

    def find_chessboard(self, square_size, checkerboard_size = (7, 9)):

        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:checkerboard_size[1], 0:checkerboard_size[0]].T.reshape(-1, 2) * square_size

        ret, self.imgp = cv2.findChessboardCorners(self.image, (checkerboard_size[1], checkerboard_size[0]), None)
        if ret:
            self.imgp = cv2.cornerSubPix(self.image, self.imgp, (11, 11), (-1, -1), criteria)
            #if np.linalg.norm(self.imgp[0][0]) > np.linalg.norm(self.imgp[-1][0]):
            #    self.imgp = self.imgp[::-1]
            cv2.drawChessboardCorners(self.cb_image, (checkerboard_size[1], checkerboard_size[0]), self.imgp, ret)
            #cv2.shape = {tuple: 3} (63, 1, 2)imshow("image", self.image)
            #cv2.waitKey(0)
            return self.imgp, self.objp
        else:
            print("no chessboard found")
            return False

    def invert_imgpt(self, checkerboard_size = (7, 9)):
        self.imgp = self.imgp[::-1]
        #self.objp = self.objp[::-1]
        image = self.rgb_image.copy()
        cv2.drawChessboardCorners(image, (checkerboard_size[1], checkerboard_size[0]), self.imgp, True)
        self.cb_image = image.copy()

    def find_apples(self):
        # load pre trained model
        model = YOLO("yolov8n.pt")
        # display model infos
        #model.info()
        # analyse image
        results = model(self.rgb_image, imgsz=[736,1280])   # change imgsz according to used camera model
        for result in results:
            for obj_nr in range(len(result.boxes)):
                if result.boxes.cls[obj_nr] == 47:
                    xyxy = result.boxes.xyxy[obj_nr]
                    apple_image = self.rgb_image[round(xyxy[1].item()-10):round(xyxy[3].item()+10), round(xyxy[0].item()-10):round(xyxy[2].item())+10]
                    self.apples.append(Apple(xyxy,apple_image))
        for apple in self.apples:
            apple.find_blossom()
            self.mark_apple_blossom_on_image(apple)

    def mark_apple_blossom_on_image(self, apple):
        cords = apple.get_image_coordinate()
        print(cords)

        y, x = cords  # Falls cords als (row, col) gegeben ist
        line_length = 5  # Länge der Linien des Kreuzes

        # Zeichne das Kreuz
        cv2.line(self.rgb_image, (x - line_length, y), (x + line_length, y), (255, 0, 0), 1)
        cv2.line(self.rgb_image, (x, y - line_length), (x, y + line_length), (255, 0, 0), 1)
        #cv2.imshow("real image", self.rgb_image)
        #cv2.waitKey(0)


class Apple:
    def __init__(self, orig_coords, image):
        self.orig_coords = orig_coords

        # blossom finding param
        self.grey_cropped_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        self.rgb_cropped_image = image
        self.rgb_center_marked_image = self.rgb_cropped_image.copy()
        self.apple_marked_image = self.rgb_cropped_image.copy()
        self.apple_reduced_rgb_image = None
        self.edges_image = None
        self.no_exterior_edges = None
        self.centers = None
        self.apple_center = None
        self.blossom = None

    def get_image_coordinate(self):
        """
        calculate the coordinates on the original image
        :return:
        """
        x_cord = self.orig_coords[1]-10 + self.blossom[0]
        y_cord = self.orig_coords[0]-10 + self.blossom[1]
        return round(x_cord.item()), round(y_cord.item())

    def find_blossom(self):
        """
        find the Blossom in the image
        :return:
        """
        outlines = self.find_outlines()

        if len(outlines) > 1:
            print("outlines not precisely found to many circles")
            return
        if outlines is None:
            print("outlines not found")
            return
        outline = outlines[0]
        self.reduce_rgb_to_apple(outline)
        self.find_edges_with_canny_edge()
        self.remove_exterior_edges(outline)
        contours = self.find_contours()
        finished_image = self.remove_contours(300, contours)
        points = np.argwhere(finished_image == 255)
        if points.size == 0:
            print("No Blossom found")
            return
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        self.centers = dbscan_clustering(finished_image)
        self.blossom = self.classify_right_blossom()
        self.mark_center_on_rgb_image(self.blossom)

    def find_outlines(self):
        """
        find the outlines with hough circle transformation
        :return:
        """
        # gausian blured image
        gausian_blured_image = cv2.GaussianBlur(self.grey_cropped_image, (3, 3), 0)
        # find circles
        circles = cv2.HoughCircles(gausian_blured_image, cv2.HOUGH_GRADIENT, 1, 300, param1=80, param2=20, minRadius=20, maxRadius=200)
        # draw circle into an image and show it in cv2 only necessary for debugging can be commented out in normal use
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for circle in circles[0, :]:
                # draw outline
                cv2.circle(self.apple_marked_image, (circle[0], circle[1]), circle[2], (0, 255, 0), 2)
                # draw center
                cv2.circle(self.apple_marked_image, (circle[0], circle[1]), 2, (0, 0, 255), 3)
                #cv2.imshow("apple marked", self.apple_marked_image)
                #cv2.waitKey(0)
            return circles[0,:]
        else:
            print("no apple found")
            return circles

    def reduce_rgb_to_apple(self, apple):
        self.apple_center = (apple[0], apple[1])
        radius = apple[2]
        mask = np.zeros_like(self.rgb_cropped_image)
        cv2.circle(mask, self.apple_center, radius, (255, 255, 255), -1)
        self.apple_reduced_rgb_image= cv2.bitwise_and(self.rgb_cropped_image, mask)
        #cv2.imshow("reduced to apple",self.apple_reduced_rgb_image)
        #cv2.waitKey(0)

    def find_edges_with_canny_edge(self):
        median_blur = cv2.medianBlur(self.apple_reduced_rgb_image, 3)
        canny_edges = cv2.Canny(median_blur, 70, 120)
        #cv2.imshow("canny edges",canny_edges)
        #cv2.waitKey(0)
        self.edges_image = canny_edges

    def remove_exterior_edges(self, apple):
        center = (apple[0], apple[1])
        radius = apple[2]-30
        mask = np.zeros_like(self.grey_cropped_image, dtype=np.uint8)
        cv2.circle(mask, center, radius, 255, -1)
        self.no_exterior_edges = cv2.bitwise_and(self.edges_image, mask)
        #cv2.imshow("no exterior edges", self.no_exterior_edges)
        #cv2.waitKey(0)

    def find_contours(self):
        # find contours
        countours, _ = cv2.findContours(self.no_exterior_edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return countours

    def remove_contours(self, length, contours):
        cleared_image = self.no_exterior_edges.copy()
        for contour in contours:
            perimeter = cv2.arcLength(contour, False)
            if perimeter > length:
                cv2.drawContours(cleared_image, [contour], 0, (0,0,0), -1)
        #cv2.imshow("cleared image", cleared_image)
        #cv2.waitKey(0)
        return cleared_image

    def mark_center_on_rgb_image(self, center):
        # for center in self.centers:
            # self.rgb_center_marked_image[round(center[0]),round(center[1])] = (0,0,255)
        self.rgb_center_marked_image[round(center[0]), round(center[1])] = (0, 0, 255)
        #cv2.imshow("finished", self.rgb_center_marked_image)
        #cv2.waitKey(0)

    def classify_right_blossom(self):
        if len(self.centers) == 1:
            return self.centers[0]
        else:
            dist_to_apple_center = []
            cluster_sizes = []
            for center in self.centers:
                dist_x = center[0] - self.apple_center[0]
                dist_y = center[1] - self.apple_center[1]
                dist = math.sqrt(dist_x ** 2 + dist_y ** 2)
                dist_to_apple_center.append(dist)
                cluster_sizes.append(center[2])
            min_dist = np.min(dist_to_apple_center)
            dist_arr = np.where(dist_to_apple_center == min_dist, 1, 0)
            max_size = np.max(cluster_sizes)
            size_arr = np.where(cluster_sizes == max_size, 1, 0)
            classify = size_arr + dist_arr
            max_index = np.where(classify == np.max(classify))[0][0]
            return self.centers[max_index]




























