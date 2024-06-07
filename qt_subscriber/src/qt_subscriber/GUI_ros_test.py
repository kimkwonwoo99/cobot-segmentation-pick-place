#!/usr/bin/env python3

import sys
import os
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap, QImage, QIcon
import rospy
from PyQt5.QtCore import QTimer, QTime
from qt_subscriber.poppy_window import PoppyWindow
from qt_subscriber.pick_window import PickWindow
from qt_subscriber.put_window import PutWindow


script_dir = os.path.dirname(os.path.abspath(__file__))
# Load UI files
form_main = uic.loadUiType(os.path.join(script_dir, 'main.ui'))[0]
form_pick = uic.loadUiType(os.path.join(script_dir, 'pick.ui'))[0]
form_poppy = uic.loadUiType(os.path.join(script_dir, 'poppy.ui'))[0]
form_put = uic.loadUiType(os.path.join(script_dir, 'put.ui'))[0]

class WindowClass(QMainWindow, form_main):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle('My Application')
        self.setWindowIcon(QIcon('/home/kkw/catkin_ws/src/qt_subscriber/src/qt_subscriber/back_images/dressme_icon.png'))
        self.setGeometry(100, 100, 800, 600)
        image_path = '/home/kkw/catkin_ws/src/qt_subscriber/src/qt_subscriber/back_images/background.jpg'
        self.set_background_image(image_path)
        icon_image_path = '/home/kkw/catkin_ws/src/qt_subscriber/src/qt_subscriber/back_images/dressme_icon.png'
        self.set_icon_image(icon_image_path)
        button_image_path = '/home/kkw/catkin_ws/src/qt_subscriber/src/qt_subscriber/back_images/button_icon.png'
        self.set_button_image(self.putButton, button_image_path, "옷 넣기", 120, 40)
        self.set_button_image(self.pickButton, button_image_path, "옷 꺼내기", 120, 40)
        self.set_button_image(self.ppButton, button_image_path, "뽀삐", 120, 40)

        self.pickButton.clicked.connect(self.pickFunction)
        self.ppButton.clicked.connect(self.ppFunction)
        self.putButton.clicked.connect(self.putFunction)

        # 추가된 부분: 시간을 표시하기 위한 QLabel과 QTimer 설정
        self.time_label = self.findChild(QLabel, "timeLabel")
        font = self.time_label.font()
        font.setPointSize(36)  # Set the desired font size
        self.time_label.setFont(font)
        self.time_label.setStyleSheet("color: rgb(109, 101, 90);")
        
        # Initialize the time display
        self.update_time()

        # Set up the timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_time)
        self.timer.start(1000)  # Update every second

    def set_button_image(self, button, image_path, text, width, height):
        # Get the original position and size of the button
        button_geometry = button.geometry()

        # Create a label for the icon
        icon_label = QLabel(self)
        icon_label.setPixmap(QPixmap(image_path).scaled(width, height, Qt.IgnoreAspectRatio))
        icon_label.setGeometry(button_geometry.x(), button_geometry.y(), width, height)
        icon_label.setAttribute(Qt.WA_TransparentForMouseEvents)

        # Create a label for the text
        text_label = QLabel(text, self)
        text_label.setAlignment(Qt.AlignCenter)
        text_label.setGeometry(button_geometry.x(), button_geometry.y(), width, height)
        text_label.setStyleSheet("color: white; font-size: 16px;")
        text_label.setAttribute(Qt.WA_TransparentForMouseEvents)

        # Set button to be transparent
        button.setStyleSheet("background: transparent; border: none;")
        button.setGeometry(button_geometry)

        # Raise the button to ensure it is on top and clickable
        button.raise_()
        button.show()
        

        
        

    def set_background_image(self, image_path):
        # Load the image
        image = QImage(image_path)

        # Resize the image to 640x480
        resized_image = image.scaled(800, 600, Qt.IgnoreAspectRatio)

        # Create a label to display the image
        background_label = QLabel(self)
        pixmap = QPixmap.fromImage(resized_image)
        background_label.setPixmap(pixmap)

        # Make the label fill the entire window
        background_label.setGeometry(0, 0, 800, 600)
        background_label.lower()  # Send the label to the background

    def set_icon_image(self, image_path):
        # Load the image
        image = QImage(image_path)
        resized_image = image.scaled(400, 200, Qt.IgnoreAspectRatio)
        # Find the QLabel by name
        icon_label = QLabel(self)
        
        # Set the pixmap to the QLabel
        pixmap = QPixmap.fromImage(resized_image)
        icon_label.setPixmap(pixmap)
        icon_label.setGeometry(200, 120, 400, 200)

    def update_time(self):
        current_time = QTime.currentTime()
        time_text = current_time.toString('HH:mm')
        self.time_label.setText(time_text)

    def pickFunction(self):
        self.pick = PickWindow(self)
        self.pick.show()
        self.hide()

    def ppFunction(self):
        self.poppy = PoppyWindow(self)
        self.poppy.show()
        self.hide()

    def putFunction(self):
        self.put = PutWindow(self)
        self.put.show()
        self.hide()


if __name__ == '__main__':
    import sys
    rospy.init_node('GUI_node', anonymous=True)
    app = QApplication(sys.argv)
    myWindow = WindowClass()
    myWindow.show()
    sys.exit(app.exec_())
