from PyQt5.QAxContainer import QAxBase
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QWidget, QMainWindow, QPushButton, QLabel, QGridLayout, QApplication, QLineEdit
import sys

class ErrorWindow(QWidget):
    def __init__(self, error_message):
        # add label
        super().__init__()
        self.error_message_label = QLabel(error_message)
        self.ok_button = QPushButton("OK")
        self.ok_button.clicked.connect()
        layout = QGridLayout()
        layout.addWidget(self.error_message_label, 0, 0)
        layout.addWidget(self.ok_button, 0, 1)
        self.setLayout(layout)
        self.setWindowTitle("Error")


    def close_window_and_call_function(self):
        self.close()
