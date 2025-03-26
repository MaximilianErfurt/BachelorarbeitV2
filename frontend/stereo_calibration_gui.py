from PyQt5.QtWidgets import QWidget, QPushButton, QLabel, QGridLayout, QDialog
from PyQt5.QtCore import Qt
from Tools.scripts.generate_sre_constants import update_file
from seaborn import jointplot
import time

from helper import image_to_QImage
from PyQt5.QtGui import QPixmap
import threading
class StereoCalibrationGUI(QDialog):
    def __init__(self, parent=None, stereo_cam = None):
        super().__init__(parent)
        # add stereo_cam
        self.stereo_cam = stereo_cam
        self.counter = 0

        # add threads
        self.image_taking_thread = None
        self.update_label_thread = None

        # def labels
        self.text_image_left = QLabel("Foto Kamera links", self)
        self.text_image_left.setAlignment(Qt.AlignCenter)
        self.text_image_right = QLabel("Foto Kamera rechts", self)
        self.text_image_right.setAlignment(Qt.AlignCenter)
        self.image_label_left = QLabel(self)
        self.image_label_right = QLabel(self)

        # def buttons
        self.take_images_button = QPushButton("Fotos aufnehmen")
        self.next_button = QPushButton("Next")
        self.next_button.setEnabled(False)
        # connect signals
        self.take_images_button.clicked.connect(self.call_take_images_button)
        self.next_button.clicked.connect(self.call_next_button)

        # def layout
        self.layout = QGridLayout()

        # add widgets
        self.layout.addWidget(self.text_image_left, 1,0)
        self.layout.addWidget(self.text_image_right, 1, 2)
        self.layout.addWidget(self.image_label_left, 2, 0)
        self.layout.addWidget(self.image_label_right, 2, 2)
        self.layout.addWidget(self.take_images_button, 0,1)
        self.layout.addWidget(self.next_button, 1,1)
        self.setLayout(self.layout)
        self.setWindowTitle("Stereo Kalibrierung")



    def call_take_images_button(self):
        self.take_images_button.setEnabled(False)
        self.next_button.setEnabled(False)
        self.image_taking_thread = threading.Thread(target= self.stereo_cam.take_stereo_calibration_images, args=(self.counter,))
        self.image_taking_thread.start()
        self.update_label_thread = threading.Thread(target=self.update_labels)
        self.update_label_thread.start()


    def call_next_button(self):
        try:
            left = self.stereo_cam.calibration_images_left[self.counter]
            right = self.stereo_cam.calibration_images_right[self.counter]
        except IndexError:
            return
        self.next_button.setEnabled(False)
        self.image_label_left.clear()
        self.image_label_right.clear()
        self.adjustSize()
        if self.counter == 16:
            self.next_button.setText("Finish")
        if self.counter == 17:
            self.stereo_cam.stereo_calibration()
            self.close()
        self.counter += 1

    def update_labels(self):
        self.image_taking_thread.join()
        try:
            # try to get the image from according cam
            image_left = self.stereo_cam.calibration_images_left[self.counter].cb_image
            image_right = self.stereo_cam.calibration_images_right[self.counter].cb_image
            # convert grey image to qimage
            qimage_left = image_to_QImage(image_left)
            qimage_right = image_to_QImage(image_right)
            # put image into the QLabel
            # convert images in pixmaps
            pixmap_left = QPixmap.fromImage(qimage_left)
            pixmap_right = QPixmap.fromImage(qimage_right)

            self.image_label_left.setPixmap(pixmap_left)  # put pixmap on label
            self.image_label_left.setScaledContents(True)  # scale image to fit in label
            self.image_label_right.setPixmap(pixmap_right)
            self.image_label_right.setScaledContents(True)

            # activate the buttons
            self.take_images_button.setEnabled(True)
            self.next_button.setEnabled(True)

        except IndexError as e:
            print("not available yet")
            time.sleep(1)
            self.update_image_label()

        print("bild updated")