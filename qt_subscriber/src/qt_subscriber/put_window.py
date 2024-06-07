#!/usr/bin/env python

import os
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from PyQt5.QtWidgets import *
from PyQt5 import uic, QtGui
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap, QImage
from std_msgs.msg import Int32
from sensor_msgs.msg import CompressedImage
from qt_subscriber.srv import main_service_msg, second_service_msg, second_service_msgRequest, basic_service, SegImageService, SegImageServiceResponse

script_dir = os.path.dirname(os.path.abspath(__file__))
form_put = uic.loadUiType(os.path.join(script_dir, 'put.ui'))[0]

class ImageSubscriber2:
    def __init__(self, window):
        self.window = window
        self.bridge = CvBridge()
        self.subscriber = rospy.Subscriber('/camera/arm/compressed', CompressedImage, self.image_callback)

    def image_callback(self, msg):
        # Decode compressed image message
        img_np = self.bridge.compressed_imgmsg_to_cv2(msg)
        img_np = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
        img_resized = cv2.resize(img_np, (320, 240))
        # Convert numpy array to QImage
        qImg = QtGui.QImage(img_resized.data, img_resized.shape[1], img_resized.shape[0], img_resized.shape[1] * 3, QtGui.QImage.Format_RGB888)
        # Convert QImage to QPixmap
        pixmap = QtGui.QPixmap.fromImage(qImg)
        # Update QLabel in window with the new image
        self.window.setImage(pixmap)

class PutWindow(QMainWindow, form_put):
    def __init__(self, parent):
        super().__init__()
        self.setupUi(self)
        image_path = '/home/kkw/catkin_ws/src/qt_subscriber/src/qt_subscriber/back_images/background.jpg'
        self.set_background_image(image_path)
        icon_image_path = '/home/kkw/catkin_ws/src/qt_subscriber/src/qt_subscriber/back_images/dressme_icon.png'
        self.set_icon_image(icon_image_path)
        prev_image_path = '/home/kkw/catkin_ws/src/qt_subscriber/src/qt_subscriber/back_images/prev_image.png'
        button_image_path = '/home/kkw/catkin_ws/src/qt_subscriber/src/qt_subscriber/back_images/button_icon.png'
        self.set_button_image(self.ToMain3, prev_image_path, "", 40, 40)
        self.set_button_image(self.noneButton, button_image_path, "옷걸이 꺼내기", 120, 30)
        self.set_button_image(self.pubButton, button_image_path, "캡처", 120, 30)
        self.set_button_image(self.hangButton, button_image_path, "옷 걸기", 120, 30)
        
        self.parent = parent
        self.initUi()
        self.bridge = CvBridge()
        self.image_subscriber = ImageSubscriber2(self)
        self.service_client = rospy.ServiceProxy('robotarm_action_service', main_service_msg)
        self.sec_service_client = rospy.ServiceProxy('second_service', second_service_msg)
        self.trd_service_client = rospy.ServiceProxy('third_service', basic_service)
        self.seg_image_service = rospy.Service('seg_image_service', SegImageService, self.handle_service)
        self.pixmap = None
        self.returnservice = None
        self.pub = rospy.Publisher('capture_start_topic', Int32, queue_size=10)
        self.toStartMode()

    def initUi(self):
        self.ToMain3.clicked.connect(self.returnToMain)
        self.noneButton.clicked.connect(self.put_mode)
        self.pubButton.clicked.connect(self.pub_1)
        self.groupBox_2.hide()
        self.retake_button.clicked.connect(self.retake_action)
        self.save_button.clicked.connect(self.save_action)
        self.hangButton.clicked.connect(self.hang)

        self.label = self.findChild(QLabel, "label")
        self.label.setFixedSize(320, 240)

        button_image_path = '/home/kkw/catkin_ws/src/qt_subscriber/src/qt_subscriber/back_images/button_icon.png'
        self.set_button_image(self.retake_button, button_image_path, "다시 찍기", 100, 30)
        self.set_button_image(self.save_button, button_image_path, "저장", 100, 30)

    def set_button_image(self, button, image_path, text, width, height):
        # Get the original position and size of the button
        button_geometry = button.geometry()

        # Create a label for the icon
        icon_label = QLabel(button.parentWidget())
        icon_label.setPixmap(QPixmap(image_path).scaled(width, height, Qt.IgnoreAspectRatio))
        icon_label.setGeometry(button_geometry.x(), button_geometry.y(), width, height)
        icon_label.setAttribute(Qt.WA_TransparentForMouseEvents)

        # Create a label for the text
        text_label = QLabel(text, button.parentWidget())
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

    def setImage(self, pixmap):
        self.label.setPixmap(pixmap)

    def handle_service(self, request):
        """
        Callback function to handle service requests.
        """
        try:
            # Decode compressed image message
            image = self.bridge.compressed_imgmsg_to_cv2(request.image, desired_encoding='bgr8')
            # Convert BGR to RGB
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            class_name = request.class_name
            # Display image in a dialog window
            self.show_image_dialog(image, class_name)
            while True :
                if self.returnservice is not None :
                    break
            if self.returnservice :
                self.returnservice = None
                return SegImageServiceResponse(success= True)
            else :
                self.returnservice = None
                return SegImageServiceResponse(success= False)

            
        except Exception as e:
            rospy.logerr(f"Error processing service request: {e}")
            # return SegImageServiceResponse(success=False)
    
    def show_image_dialog(self, image, class_name):
        """
        Display a dialog window with the received image and buttons.
        """
        self.groupBox_2.show()
        # Display image
        q_image = QtGui.QImage(image.data, image.shape[1], image.shape[0], image.shape[1] * 3, QtGui.QImage.Format_RGB888)

        self.pixmap = QtGui.QPixmap.fromImage(q_image)
        self.label_2.setPixmap(self.pixmap)
        
        # Display class name
        self.label_3.setText(class_name)
        self.label_3.setStyleSheet("color: rgb(109, 101, 90);")

    def retake_action(self):
        
        # Hide groupBox_2
        self.returnservice = False
        self.groupBox_2.hide()
        

    def save_action(self):
        if self.pixmap is not None:
            # Save image to /catkin_ws/images with class name
            class_name = self.label_3.text()
            image_path = os.path.join('/home/kkw/catkin_ws/images', f'{class_name}.png')
           
            image = self.pixmap.toImage()
            image = image.convertToFormat(QtGui.QImage.Format_RGB888)
            width = image.width()
            height = image.height()
            
            ptr = image.bits()
            ptr.setsize(image.byteCount())
            
            arr = np.array(ptr).reshape(height, width, 3)  # 4 for RGBA
            arr = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)  # Convert from RGBA to RGB
            
            cv2.imwrite(image_path, arr)
            rospy.loginfo(f'Image saved as {image_path}')
        else:
            rospy.logwarn("No image to save.")
        # Hide groupBox_2
        self.returnservice = True
        self.groupBox_2.hide()

    def hang(self):
        rospy.wait_for_service('third_service')
        try:
            response = self.trd_service_client(1)
            if isinstance(response.success, bool):
                if response.success:
                    QMessageBox.information(self, 'Service Call', '옷 수납 완료')
                    print("success third service")
                else:
                    QMessageBox.warning(self, 'Service Call', '실패!!')
                    print("Failed sec service")
            else:
                QMessageBox.warning(self, 'Service Call', 'Invalid response: success field is not a boolean.')
        except rospy.ServiceException as e:
            QMessageBox.critical(self, 'Service Call', f'Service call failed: {e}')

    def toStartMode(self):
        rospy.wait_for_service('robotarm_action_service')
        try:
            response = self.service_client("start_mode")
            if isinstance(response.success, bool):
                if response.success:
                    QMessageBox.information(self, 'Service Call', '옷 등록 모드를 시작합니다!')
                    print("success first service")
                else:
                    QMessageBox.warning(self, 'Service Call', 'Failed to activate start mode.')
            else:
                QMessageBox.warning(self, 'Service Call', 'Invalid response: success field is not a boolean.')
        except rospy.ServiceException as e:
            QMessageBox.critical(self, 'Service Call', f'Service call failed: {e}')

    def toEndMode(self):
        rospy.wait_for_service('robotarm_action_service')
        try:
            response = self.service_client("end_mode")
            if isinstance(response.success, bool):
                if response.success:
                    QMessageBox.information(self, 'Service Call', '옷 등록 모드를 종료합니다!')
                else:
                    QMessageBox.warning(self, 'Service Call', 'Failed to activate end mode.')
            else:
                QMessageBox.warning(self, 'Service Call', 'Invalid response: success field is not a boolean.')
        except rospy.ServiceException as e:
            QMessageBox.critical(self, 'Service Call', f'Service call failed: {e}')
    
    def put_mode(self):
        mode = 'put_mode'
        print("put_mode btn clicked")
        rospy.wait_for_service('second_service')
        try:
            request = second_service_msgRequest()
            request.mode = mode
            request.class_name = 'Hanger'
            response = self.sec_service_client(request)
            if isinstance(response.success, bool):
                if response.success :
                    QMessageBox.information(self, 'Service Call', '등록할 옷을 옷걸이에 걸고 보여주세요. 버튼을 누르면 캡처합니다.')
                    print("success sec service")
                else:
                    QMessageBox.warning(self, 'Service Call', '실패')
                    print("sec fir fail")
            else :
                QMessageBox.warning(self, 'Service Call', '실패')
                print("sec sec fail")
        except rospy.ServiceException as e:
            QMessageBox.critical(self, 'Service Call', f'Service call failed: {e}')
            print("sec thr fail")

    def pub_1(self):
        self.pub.publish(1)

    def returnToMain(self):
        self.seg_image_service.shutdown()
        self.seg_image_service = None
        self.toEndMode()
        self.close()
        self.parent.show()
        
    

        