#!/usr/bin/env python3

import os
import rospy
import cv2
from cv_bridge import CvBridge
from PyQt5.QtWidgets import *
from PyQt5 import uic, QtGui
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap, QImage, QIcon
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage


script_dir = os.path.dirname(os.path.abspath(__file__))
form_poppy = uic.loadUiType(os.path.join(script_dir, 'poppy.ui'))[0]

class ImageSubscriber:
    def __init__(self, window):
        self.window = window
        self.bridge = CvBridge()
        self.subscriber = rospy.Subscriber('/camera/agv/compressed', CompressedImage, self.image_callback)

    def image_callback(self, msg):
        # Decode compressed image message
        img_np = self.bridge.compressed_imgmsg_to_cv2(msg)
        img_np = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
        # Convert numpy array to QImage
        qImg = QtGui.QImage(img_np.data, img_np.shape[1], img_np.shape[0], img_np.shape[1] * 3, QtGui.QImage.Format_RGB888)
        # Convert QImage to QPixmap
        pixmap = QtGui.QPixmap.fromImage(qImg)
        # Update QLabel in window with the new image
        self.window.setImage(pixmap)

class PoppyWindow(QMainWindow, form_poppy):
    def __init__(self, parent):
        super().__init__()
        self.setupUi(self)
        image_path = '/home/kkw/catkin_ws/src/qt_subscriber/src/qt_subscriber/back_images/background.jpg'
        self.set_background_image(image_path)
        icon_image_path = '/home/kkw/catkin_ws/src/qt_subscriber/src/qt_subscriber/back_images/dressme_icon.png'
        self.set_icon_image(icon_image_path)
        button_image_path = '/home/kkw/catkin_ws/src/qt_subscriber/src/qt_subscriber/back_images/button_icon.png'
        self.set_button_image(self.moveButton, button_image_path, "이동", 100, 30)
        prev_image_path = '/home/kkw/catkin_ws/src/qt_subscriber/src/qt_subscriber/back_images/prev_image.png'
        self.set_button_image(self.ToMain1, prev_image_path, "", 40, 40)

        self.parent = parent
        self.initUi()
        self.image_subscriber = ImageSubscriber(self)
        

        # Initialize ROS publisher
        self.pub = rospy.Publisher('user_request', String, queue_size=10)

    def initUi(self):
        self.label = self.findChild(QLabel, "label")
        self.label.setFixedSize(320, 240)  # Ensure you have a QLabel named "label" in your UI
        self.ToMain1.clicked.connect(self.returnToMain)
        self.moveButton.clicked.connect(self.ToMove)
        self.livingroom.clicked.connect(self.groupboxRadFunction)
        self.bathroom.clicked.connect(self.groupboxRadFunction)
        self.dressroom.clicked.connect(self.groupboxRadFunction)
        self.startpoint.clicked.connect(self.groupboxRadFunction)

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
        resized_image = image.scaled(200, 100, Qt.IgnoreAspectRatio)
        # Find the QLabel by name
        icon_label = QLabel(self)
        
        # Set the pixmap to the QLabel
        pixmap = QPixmap.fromImage(resized_image)
        icon_label.setPixmap(pixmap)
        icon_label.setGeometry(600, 0, 200, 100)


    def ToMove(self):
        location = None
        if self.livingroom.isChecked():
            location = "livingroom"
        elif self.bathroom.isChecked():
            location = "bathroom"
        elif self.dressroom.isChecked():
            location = "dressroom"
        elif self.startpoint.isChecked():
            location = "startpoint"
        elif self.stop.isChecked():
            location = "stop"

        if location == "stop":
            print("정지합니다")
            self.pub.publish(location)
        elif location:
            print(f"Moving to {location}")
            self.pub.publish(location)  # Publish the location to the ROS topic

    def groupboxRadFunction(self):
        pass  # This method can be removed if not needed

    def setImage(self, pixmap):
        self.label.setPixmap(pixmap)

    def returnToMain(self):
        self.close()
        self.parent.show()