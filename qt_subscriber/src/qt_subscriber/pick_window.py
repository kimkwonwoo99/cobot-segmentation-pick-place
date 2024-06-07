#!/usr/bin/env python

import os
import glob
import rospy
import cv2
from cv_bridge import CvBridge
from PyQt5.QtWidgets import *
from PyQt5 import uic, QtGui, QtCore
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap, QImage
from sensor_msgs.msg import CompressedImage
from qt_subscriber.srv import main_service_msg, second_service_msg, second_service_msgRequest, basic_service


script_dir = os.path.dirname(os.path.abspath(__file__))
form_pick = uic.loadUiType(os.path.join(script_dir, 'pick.ui'))[0]

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


class PickWindow(QMainWindow, form_pick):
    def __init__(self, parent):
        super().__init__()
        self.setupUi(self)

        image_path = '/home/kkw/catkin_ws/src/qt_subscriber/src/qt_subscriber/back_images/background.jpg'
        self.set_background_image(image_path)
        icon_image_path = '/home/kkw/catkin_ws/src/qt_subscriber/src/qt_subscriber/back_images/dressme_icon.png'
        self.set_icon_image(icon_image_path)
        button_image_path = '/home/kkw/catkin_ws/src/qt_subscriber/src/qt_subscriber/back_images/button_icon.png'
        self.set_button_image(self.hangButton, button_image_path, "옷걸이 넣기", 120, 30)
        prev_image_path = '/home/kkw/catkin_ws/src/qt_subscriber/src/qt_subscriber/back_images/prev_image.png'
        self.set_button_image(self.ToMain2, prev_image_path, "", 40, 40)
        
        self.parent = parent
        self.currentPageIndex1 = 0
        self.currentPageIndex2 = 0
        self.image_directory = '/home/kkw/catkin_ws/images'  # Change to your actual image directory
        self.initUi()
        self.image_subscriber = ImageSubscriber2(self)
        # Initialize ROS service client
        self.service_client = rospy.ServiceProxy('robotarm_action_service', main_service_msg)
        self.sec_service_client = rospy.ServiceProxy('second_service', second_service_msg)
        self.trd_service_client = rospy.ServiceProxy('third_service', basic_service)
        self.toStartMode()


    def initUi(self):
        # Retrieve the tab widget and stacked widgets from the UI
 
        self.tabWidget = self.findChild(QTabWidget, "tabWidget")
        self.stackedWidget1 = self.tabWidget.findChild(QStackedWidget, "stackedWidget1")
        self.stackedWidget2 = self.tabWidget.findChild(QStackedWidget, "stackedWidget2")
        
        # Set up titles for existing pages in stackedWidget1 and stackedWidget2
        self.setup_stackedWidgetTitles()

        self.nextpage.clicked.connect(self.next_page)
        self.prevpage.clicked.connect(self.prev_page)
        self.nextpage_2.clicked.connect(self.next_page_2)
        self.prevpage_2.clicked.connect(self.prev_page_2)
        self.dicision.clicked.connect(self.pick_mode)
        self.dicision_2.clicked.connect(self.pick_mode_2)
        self.hangButton.clicked.connect(self.hang)

        self.label = self.findChild(QLabel, "label")
        self.label.setFixedSize(320, 240)
        self.ToMain2.clicked.connect(self.returnToMain)

        self.load_images()
        prev_image_path = '/home/kkw/catkin_ws/src/qt_subscriber/src/qt_subscriber/back_images/prev_image.png'
        next_image_path = '/home/kkw/catkin_ws/src/qt_subscriber/src/qt_subscriber/back_images/next_image.png'
        button_image_path = '/home/kkw/catkin_ws/src/qt_subscriber/src/qt_subscriber/back_images/button_icon.png'
        self.set_button_image(self.nextpage, next_image_path, "", 40, 40)
        self.set_button_image(self.prevpage, prev_image_path, "", 40, 40)
        self.set_button_image(self.nextpage_2, next_image_path, "", 40, 40)
        self.set_button_image(self.prevpage_2, prev_image_path, "", 40, 40)
        self.set_button_image(self.dicision, button_image_path, "결정", 100, 30)
        self.set_button_image(self.dicision_2, button_image_path, "결정", 100, 30)

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


    def setup_stackedWidgetTitles(self):
        titles1 = ["shirt", "T-shirt", "pants", "shorts", "skirt"]
        titles2 = ["red", "blue", "pink", "grey", "black", "white"]

        for i, title in enumerate(titles1):
            page = self.stackedWidget1.widget(i)
            if page.layout() is None or not isinstance(page.layout(), QVBoxLayout):
                layout = QVBoxLayout(page)
                page.setLayout(layout)
            titleLabel = QLabel(title)
            titleLabel.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop)  # Align top-left
            layout = page.layout()
            layout.insertWidget(0, titleLabel)
            print(f"Setup stackedWidget1 page {i} with title {title}")

        for i, title in enumerate(titles2):
            page = self.stackedWidget2.widget(i)
            if page.layout() is None or not isinstance(page.layout(), QVBoxLayout):
                layout = QVBoxLayout(page)
                page.setLayout(layout)
            titleLabel = QLabel(title)
            titleLabel.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop)  # Align top-left
            layout = page.layout()
            layout.insertWidget(0, titleLabel)
            print(f"Setup stackedWidget2 page {i} with title {title}")

    def load_images(self):
        # Find all image files in the directory
        image_files = glob.glob(os.path.join(self.image_directory, '*'))
        image_files = [f for f in image_files if f.lower().endswith(('.png', '.jpg', '.jpeg'))]

        if not image_files:
            print("No images found in the directory.")
            return

        # Dictionaries for mapping page titles to their indices
        type_map = {
            "Shirt": 0,
            "Tshirt": 1,
            "Pants": 2,
            "Shorts": 3,
            "Skirt": 4
        }
        color_map = {
            "Red": 0,
            "Blue": 1,
            "Pink": 2,
            "Grey": 3,
            "Black": 4,
            "White": 5
        }

        # Iterate over image files and add them to the appropriate page
        for image_file in image_files:
            base_name = os.path.basename(image_file)
            type_color = base_name.split('.')[0]
            try:
                color, type_ = type_color.split('-')
            except ValueError:
                print(f"Filename {base_name} does not match the expected format.")
                continue

            if type_ in type_map and color in color_map:
                type_index = type_map[type_]
                color_index = color_map[color]

                # Add to stackedWidget1
                type_page = self.stackedWidget1.widget(type_index).layout()
                if type_page is None:
                    print(f"Layout is None for type page index {type_index}")
                self.add_image_to_layout(image_file, type_page)

                # Add to stackedWidget2
                color_page = self.stackedWidget2.widget(color_index).layout()
                if color_page is None:
                    print(f"Layout is None for color page index {color_index}")
                self.add_image_to_layout(image_file, color_page)

    def add_image_to_layout(self, image_file, layout):
        if layout is None:
            print("Layout is None. Cannot add widget.")
            return

        # Open the image using OpenCV
        img = cv2.imread(image_file)
        if img is None:
            print(f"Error loading image {image_file}")
            return

        # Resize the image while preserving aspect ratio
        resized_img = cv2.resize(img, (100, 100), interpolation=cv2.INTER_AREA)

        # Convert BGR to RGB
        resized_img_rgb = cv2.cvtColor(resized_img, cv2.COLOR_BGR2RGB)

        # Convert OpenCV image to QImage
        qImg = QtGui.QImage(resized_img_rgb.data, resized_img_rgb.shape[1], resized_img_rgb.shape[0], resized_img_rgb.shape[1] * 3, QtGui.QImage.Format_RGB888)

        # Convert QImage to QPixmap
        pixmap = QtGui.QPixmap.fromImage(qImg)

        # Create a horizontal layout for image and radio button
        h_layout = QHBoxLayout()

        # Create a label to display the image
        image_label = QLabel()
        image_label.setPixmap(pixmap)
        h_layout.addWidget(image_label)

        # Create a radio button for the image filename
        radio_button = QRadioButton(os.path.basename(image_file))
        h_layout.addWidget(radio_button)

        # Get the page title label from the layout
        page_title_label = layout.itemAt(0).widget()

        # Remove the page title label from its current position
        layout.removeWidget(page_title_label)

        # Insert the page title label at the top left
        layout.insertWidget(0, page_title_label)

        # Add the horizontal layout to the page layout
        layout.addLayout(h_layout)

    def hang(self):
        rospy.wait_for_service('third_service')
        try:
            response = self.trd_service_client(1)
            if isinstance(response.success, bool):
                if response.success:
                    QMessageBox.information(self, 'Service Call', '옷걸이 수납 완료')
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
                    QMessageBox.information(self, 'Service Call', '옷 꺼내기 모드를 시작합니다!')
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
                    QMessageBox.information(self, 'Service Call', '옷 꺼내기 모드를 종료합니다!')
                else:
                    QMessageBox.warning(self, 'Service Call', 'Failed to activate end mode.')
            else:
                QMessageBox.warning(self, 'Service Call', 'Invalid response: success field is not a boolean.')
        except rospy.ServiceException as e:
            QMessageBox.critical(self, 'Service Call', f'Service call failed: {e}')


        

    def pick_mode(self):
        selected_radio = self.get_selected_radio(self.stackedWidget1, self.stackedWidget1)
        mode = 'pick_mode'
        print("pick_mode btn clicked")
        if selected_radio:
            file_name = selected_radio.text()
            # Remove the '.png' extension if present
            if file_name.endswith('.png'):
                file_name = file_name[:-4]
            print(f"Selected item: {file_name}")
            rospy.wait_for_service('second_service')
            try:
                request = second_service_msgRequest()
                request.mode = mode
                request.class_name = file_name
                response = self.sec_service_client(request)
                if isinstance(response.success, bool):
                    if response.success :
                        QMessageBox.information(self, 'Service Call', '옷을 빼고 빈 옷걸이를 걸어주세요')
                        print("success second service")
                    else:
                        QMessageBox.warning(self, 'Service Call', '실패')
                        print("second first fail")
                else :
                    QMessageBox.warning(self, 'Service Call', '실패')
                    print("second second fail")
            except rospy.ServiceException as e:
                QMessageBox.critical(self, 'Service Call', f'Service call failed: {e}')
                print("second third fail")
        else:
            QMessageBox.warning(self, 'Selection Required', '먼저 옷을 선택해주세요')

    def pick_mode_2(self):
        selected_radio = self.get_selected_radio(self.stackedWidget2, self.stackedWidget2)
        mode = 'pick_mode'
        print("pick_mode btn clicked")
        if selected_radio:
            file_name = selected_radio.text()
            # Remove the '.png' extension if present
            if file_name.endswith('.png'):
                file_name = file_name[:-4]
            print(f"Selected item: {file_name}")
            rospy.wait_for_service('second_service')
            try:
                request = second_service_msgRequest()
                request.mode = mode
                request.class_name = file_name
                response = self.sec_service_client(request)
                if isinstance(response.success, bool):
                    if response.success :
                        QMessageBox.information(self, 'Service Call', '옷을 빼고 빈 옷걸이를 걸어주세요')
                        print("success second service")
                    else:
                        QMessageBox.warning(self, 'Service Call', '실패')
                        print("second first fail")
                else :
                    QMessageBox.warning(self, 'Service Call', '실패')
                    print("second second fail")
            except rospy.ServiceException as e:
                QMessageBox.critical(self, 'Service Call', f'Service call failed: {e}')
                print("second third fail")
        else:
            QMessageBox.warning(self, 'Selection Required', '먼저 옷을 선택해주세요')
        
    


    def get_selected_radio(self, stacked_widget, current_stacked_widget):
        for i in range(stacked_widget.count()):
            page = stacked_widget.widget(i)
            layout = page.layout()
            if layout:
                for j in range(layout.count()):
                    item = layout.itemAt(j)
                    if isinstance(item, QHBoxLayout):
                        for k in range(item.count()):
                            widget = item.itemAt(k).widget()
                            if isinstance(widget, QRadioButton):
                                # Uncheck radio buttons in other stacked widgets
                                if stacked_widget != current_stacked_widget:
                                    widget.setChecked(False)
                                # Return the selected radio button in the current stacked widget
                                if widget.isChecked():
                                    return widget
        return None
       



    def next_page(self):
        self.currentPageIndex1 += 1
        if self.currentPageIndex1 >= self.stackedWidget1.count():
            self.currentPageIndex1 = 0
        self.stackedWidget1.setCurrentIndex(self.currentPageIndex1)

    def next_page_2(self):
        self.currentPageIndex2 += 1
        if self.currentPageIndex2 >= self.stackedWidget2.count():
            self.currentPageIndex2 = 0
        self.stackedWidget2.setCurrentIndex(self.currentPageIndex2)

    def prev_page(self):
        self.currentPageIndex1 -= 1
        if self.currentPageIndex1 < 0:
            self.currentPageIndex1 = self.stackedWidget1.count() - 1
        self.stackedWidget1.setCurrentIndex(self.currentPageIndex1)

    def prev_page_2(self):
        self.currentPageIndex2 -= 1
        if self.currentPageIndex2 < 0:
            self.currentPageIndex2 = self.stackedWidget2.count() - 1
        self.stackedWidget2.setCurrentIndex(self.currentPageIndex2)

    def setImage(self, pixmap):
        self.label.setPixmap(pixmap)


    def returnToMain(self):
        self.toEndMode()
        self.close()
        self.parent.show()
