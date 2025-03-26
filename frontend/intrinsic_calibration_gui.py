import time

from PyQt5.QAxContainer import QAxBase
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QWidget, QMainWindow, QPushButton, QLabel, QGridLayout, QApplication, QLineEdit, QDialog
import sys
import threading
from backend.image import Image
import cv2
from helper import image_to_QImage


class IntrinsicCalibrationGUI(QDialog):
    def __init__(self, parent = None, mono_cam = None):
        super().__init__(parent)
        # threads
        self.image_getting_thread = None
        self.image_taking_thread = None

        # variables
        self.next_counter = 0
        self.mono_cam = mono_cam

        # create image label
        self.image_label = QLabel(self)

        # create buttons
        self.next_button = QPushButton("Next")
        self.next_button.clicked.connect(self.call_next_button)
        self.take_image_button = QPushButton("Foto aufnehmen")
        self.take_image_button.clicked.connect(self.call_take_image)

        # create layout
        layout = QGridLayout()

        # add widgets
        layout.addWidget(self.take_image_button, 0, 0)
        layout.addWidget(self.next_button, 0,1)
        layout.addWidget(self.image_label, 1, 0, 2, 2)

        self.setLayout(layout)
        self.setWindowTitle("Intrinsische Kalibrierung")

    def call_next_button(self):
        try:
            image = self.mono_cam.intrinsic_images[self.next_counter].cb_image
        except IndexError:
            return
        self.image_label.clear()
        self.adjustSize()

        if self.next_counter == 16:
            self.next_button.setText("Finish")
        if self.next_counter == 17:
            self.mono_cam.intrinsic_calibration()
            self.close()
        self.next_counter += 1

    def update_image_label(self):
        self.image_taking_thread.join()
        try:
            # try to get the image from according cam
            image = self.mono_cam.intrinsic_images[self.next_counter].cb_image
            # convert grey image to qimage
            qimage = image_to_QImage(image)
            # put image into the QLabel
            pixmap = QPixmap.fromImage(qimage)  # convert image in pixmap
            self.image_label.setPixmap(pixmap)  # put pixmap on label
            self.image_label.setScaledContents(True)  # scale image to fit in label


        except IndexError as e:
            print("not available yet")
            time.sleep(1)
            self.update_image_label()

        print("bild updated")

    def call_take_image(self):
        self.image_taking_thread = threading.Thread(target=self.mono_cam.take_intrinsic_image,
                                                    args=(self.next_counter,))
        self.image_taking_thread.start()
        self.image_getting_thread = threading.Thread(target=self.update_image_label)
        self.image_getting_thread.start()

