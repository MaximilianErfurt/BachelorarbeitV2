from PyQt5.QtWidgets import QWidget, QPushButton, QLabel, QGridLayout, QDialog
from PyQt5.QtCore import Qt
from Tools.scripts.generate_sre_constants import update_file
from seaborn import jointplot

from helper import image_to_QImage
from PyQt5.QtGui import QPixmap

from backend import stereo_camera
import threading



class StartGUI(QDialog):
    def __init__(self, parent=None, stereo_cam = None):
        super().__init__(parent)
        # vars
        self.stereo_cam = stereo_cam
        
        # threads
        self.image_taking_thread = None
        self.update_label_thread = None
        self.processing_thread = None
        
        
        # def labels
        self.text_image_left = QLabel("Foto Kamera links", self)
        self.text_image_left.setAlignment(Qt.AlignCenter)
        self.text_image_right = QLabel("Foto Kamera rechts", self)
        self.text_image_right.setAlignment(Qt.AlignCenter)
        self.image_label_left = QLabel(self)
        self.image_label_right = QLabel(self)

        # def buttons
        self.processing_button = None
        self.take_images_button = QPushButton("Fotos aufnehmen")
        # connect signals
        self.take_images_button.clicked.connect(self.call_take_images_button)

        # def layout
        self.layout = QGridLayout()
        # add widgets
        self.layout.addWidget(self.take_images_button, 0, 1)
        self.layout.addWidget(self.text_image_left, 1, 0)
        self.layout.addWidget(self.text_image_right, 1, 2)
        self.layout.addWidget(self.image_label_left, 2, 0)
        self.layout.addWidget(self.image_label_right,2, 2)

        self.setLayout(self.layout)
        self.setWindowTitle("Processing")



    def call_take_images_button(self):
        # deactivate image taking button to avoid multiple streams to the camera
        # gets activated in update image label after the thread has finished
        self.take_images_button.setEnabled(False)
        # start threads to take the images and update the label to show them in the gui
        self.image_taking_thread = threading.Thread(target=self.stereo_cam.take_mono_images)
        self.image_taking_thread.start()
        self.update_label_thread = threading.Thread(target=self.update_image_label_not_processed)
        self.update_label_thread.start()
        # add the processing button
        self.processing_button = QPushButton("Finde Ausstechpunkt")
        self.processing_button.setEnabled(False)
        self.processing_button.clicked.connect(self.call_processing_button)
        self.layout.addWidget(self.processing_button, 1,1)
        #self.showMaximized()
        self.update()

    def call_processing_button(self):
        # deactivate the buttons during processing
        self.processing_button.setEnabled(False)
        self.take_images_button.setEnabled(False)
        # start threads for processing and
        self.processing_thread = threading.Thread(target=self.stereo_cam.find_cut_out_point)
        self.processing_thread.start()
        self.update_label_thread = threading.Thread(target=self.update_image_label_processed)
        self.update_label_thread.start()
        
    def update_image_label_not_processed(self):
        # wait for image taking thread to finish
        self.image_taking_thread.join()

        # get the taken images
        image_left = self.stereo_cam.image_left.rgb_image
        image_right = self.stereo_cam.image_right.rgb_image

        # convert left image to qimage
        qimage_left = image_to_QImage(image_left)
        # put left image into the QLabel
        pixmap_left = QPixmap.fromImage(qimage_left)  # convert qimage to pixmap
        self.image_label_left.setPixmap(pixmap_left)  # put pixmap on label
        self.image_label_left.setScaledContents(True)  # scale image to fit in label

        # convert right image to qimage
        qimage_right = image_to_QImage(image_right)
        # put right image into the QLabel
        pixmap_right = QPixmap.fromImage(qimage_right)  # convert qimage to pixmap
        self.image_label_right.setPixmap(pixmap_right)  # put pixmpat on label
        self.image_label_right.setScaledContents(True)

        # activate the image taking button and the start process button again to allow taking the images again
        self.take_images_button.setEnabled(True)
        self.processing_button.setEnabled(True)

    def update_image_label_processed(self):
        # wait for processing to finish
        self.processing_thread.join()

        # get the taken images
        image_left = self.stereo_cam.image_left.rgb_image
        image_right = self.stereo_cam.image_right.rgb_image

        # convert left image to qimage
        qimage_left = image_to_QImage(image_left)
        # put left image into the QLabel
        pixmap_left = QPixmap.fromImage(qimage_left)  # convert qimage to pixmap
        self.image_label_left.setPixmap(pixmap_left)  # put pixmap on label
        self.image_label_left.setScaledContents(True)  # scale image to fit in label

        # convert right image to qimage
        qimage_right = image_to_QImage(image_right)
        # put right image into the QLabel
        pixmap_right = QPixmap.fromImage(qimage_right)  # convert qimage to pixmap
        self.image_label_right.setPixmap(pixmap_right)  # put pixmpat on label
        self.image_label_right.setScaledContents(True)

        # activate the image taking button and the start process button again to allow taking the images again
        self.take_images_button.setEnabled(True)
        self.processing_button.setEnabled(True)





        
        

