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
from helper import load_rob_poses

class ExtrinsicCalibrationGUI(QDialog):
    def __init__(self, parent = None, mono_cam = None):
        super().__init__(parent)

        self.next_counter = 0
        self.mono_cam = mono_cam
        self.image_taking_thread = None
        self.image_getting_thread = None

        # create Image Label
        self.image_label = QLabel(self)

        # create Labels
        self.x_label = QLabel("X-Koordinate")
        self.y_label = QLabel("Y-Koordinate")
        self.z_label = QLabel("Z-Koordinate")
        self.a_label = QLabel("A-Koordinate")
        self.b_label = QLabel("B-Koordinate")
        self.c_label = QLabel("C-Koordinate")
        self.next_counter_label = QLabel("next counter:" + str(self.next_counter))

        # create textboxes
        self.x_text = QLineEdit(self)
        self.y_text = QLineEdit(self)
        self.z_text = QLineEdit(self)
        self.a_text = QLineEdit(self)
        self.b_text = QLineEdit(self)
        self.c_text = QLineEdit(self)

        # create button
        self.next_button = QPushButton("Nächste Postition")
        self.next_button.clicked.connect(self.call_next_button)
        self.take_image_button = QPushButton("Foto aufnehmen")
        self.take_image_button.clicked.connect(self.take_image)
        self.invert_button = QPushButton("Invertieren")
        self.invert_button.clicked.connect(self.call_invert_button)
        self.invert_button.setEnabled(False)

        # create layout
        layout = QGridLayout()

        # add widgets
        layout.addWidget(self.x_label, 0, 0)
        layout.addWidget(self.y_label, 0, 1)
        layout.addWidget(self.z_label, 0, 2)
        layout.addWidget(self.x_text, 1, 0)
        layout.addWidget(self.y_text, 1, 1)
        layout.addWidget(self.z_text, 1, 2)
        layout.addWidget(self.a_label, 2, 0)
        layout.addWidget(self.b_label, 2, 1)
        layout.addWidget(self.c_label, 2, 2)
        layout.addWidget(self.a_text, 3, 0)
        layout.addWidget(self.b_text, 3, 1)
        layout.addWidget(self.c_text, 3, 2)
        layout.addWidget(self.next_button, 4, 0)
        layout.addWidget(self.take_image_button, 4, 1)
        layout.addWidget(self.next_counter_label, 4, 2)
        layout.addWidget(self.image_label, 6, 0, 3, 3)
        layout.addWidget(self.invert_button, 5, 0)

        self.setLayout(layout)
        self.setWindowTitle("Extrinsische Kalibrierung")
        #self.setGeometry(100, 100, 400, 100)

    def take_image(self):
        self.take_image_button.setEnabled(False)
        #self.image_taking_thread = threading.Thread(target = self.mono_cam.take_extrinsic_image, args=(self.next_counter,))
        self.image_taking_thread = threading.Thread(target=self.mono_cam.load_extrinsic_image,
                                                   args=(self.next_counter,))
        self.image_taking_thread.start()
        self.image_getting_thread = threading.Thread(target=self.update_image_label)
        self.image_getting_thread.start()

    def update_image_label(self):
        self.image_taking_thread.join()
        self.take_image_button.setEnabled(True)
        self.invert_button.setEnabled(True)
        try:
            # try to get the image from according cam
            image = self.mono_cam.extrinsic_images[self.next_counter].cb_image
            # convert grey image to qimage
            qimage = image_to_QImage(image)
            # put image into the QLabel
            pixmap = QPixmap.fromImage(qimage)      # convert image in pixmap
            self.image_label.setPixmap(pixmap)      # put pixmap on label
            self.image_label.setScaledContents(False)# scale image to fit in label

        except IndexError as e:
            print("not available")

    def update_counter_label(self):
        self.next_counter_label.setText("next counter:" + str(self.next_counter))


    def call_next_button(self):
        self.invert_button.setEnabled(False)
        # save values
        x = self.x_text.text()
        y = self.y_text.text()
        z = self.z_text.text()
        a = self.a_text.text()
        b = self.b_text.text()
        c = self.c_text.text()
        x,y,z,a,b,c = load_rob_poses(self.next_counter)

        # check for image
        try:
            image = self.mono_cam.extrinsic_images[self.next_counter].cb_image
        except IndexError:
            return

        # check entries
        if x == "" or y == "" or z == "" or a == "" or b == "" or c == "":
            return

        rob_poses = (x, y, z, a, b, c)
        self.mono_cam.add_extrinsic_rob_poses(rob_poses)

        # clear textbox
        self.x_text.clear()
        self.y_text.clear()
        self.z_text.clear()
        self.a_text.clear()
        self.b_text.clear()
        self.c_text.clear()
        self.image_label.clear()
        self.adjustSize()

        if self.next_counter == 18:
            self.next_button.setText("Finish")
        if self.next_counter == 19:
            self.mono_cam.calculate_hand_eye_matrix()
            #self.mono_cam.opencv_hand_eye_calibration()
            self.close()
        self.next_counter += 1
        self.update_counter_label()

    def call_invert_button(self):
        self.mono_cam.invert_cb(self.next_counter)
        self.update_image_label()



if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ExtrinsicCalibrationGUI()
    window.show()
    sys.exit(app.exec())