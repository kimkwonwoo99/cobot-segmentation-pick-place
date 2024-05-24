#!/usr/bin/env python

import rospy, tf, cv2
import numpy as np
from vision_seg.msg import aruco_center
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32
from ultralytics import YOLO

model = YOLO("/home/choi/catkin_ws/src/dressme/vision_seg/src/dressme_final.pt")

class Segment(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.camera_matrix = np.array([[527.43921935, 0, 317.20828396], [0, 531.15346504, 240.4016465], [0, 0, 1]])  # 카메라 행렬로 설정
        self.dist_coeffs = np.array([[0.20891653, -1.69077085, -0.00999435, -0.00851377, 5.10335346]])  # 왜곡 계수로 설정
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.seg_xy_publisher = rospy.Publisher('/seg_xy', aruco_center, queue_size=10)
        self.last_detect_position = {i: None for i in range(9)}
        self.image_sub = None
        self.image_state = False

    def matrix_to_transform(self, transform_mat):
        translation_vector = transform_mat[:3, 3]
        rotation_matrix = transform_mat[:3, :3]
        quaternion = tf.transformations.quaternion_from_matrix(transform_mat)
        return translation_vector, quaternion

    def image_callback(self, msg):
        global model
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv2_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        undistorted_img = cv2.undistort(cv2_img, self.camera_matrix, self.dist_coeffs)
        results = model(undistorted_img)
        
        H,W,_ = undistorted_img.shape
        
        if results:
            names = []
            x_points = []
            y_points = []
            for result in results:
                if result.masks:  # 이 조건을 확인
                    for box in result.boxes:
                        boxes = box.cpu().numpy()
                        class_id = int(boxes.cls[0])
                        class_conf = boxes.conf
                        
                        box_x = int(boxes.xyxy[0][0] + boxes.xyxy[0][2])/2
                        box_y = int(boxes.xyxy[0][1] + boxes.xyxy[0][3])/2
                        print(box_x, box_y)
                        names.append(str(class_id))
                        x_points.append(box_x)
                        y_points.append(box_y)
            
            if len(names) > 0:
                aruco_msg = aruco_center()
                aruco_msg.names = names
                aruco_msg.x_points = x_points
                aruco_msg.y_points = y_points

                self.seg_xy_publisher.publish(aruco_msg)
    def control_image_subscription(self, state):
        if state and self.image_sub is None:
            self.image_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.image_callback)
        elif not state and self.image_sub is not None:
            self.image_sub.unregister()
            self.image_sub = None

def callback(msg, segment_img):
    if msg.data == 1:
        if not segment_img.image_state:
            rospy.loginfo("Starting tf publisher")
            segment_img.image_state = True
            segment_img.control_image_subscription(True)
    elif msg.data == 2:
        if segment_img.image_state:
            rospy.loginfo("Stopping tf publisher")
            segment_img.image_state = False
            segment_img.control_image_subscription(False)

def segment_server(segment_img):
    rospy.init_node('yolo_segment_server')
    rospy.Subscriber("aruco_seg_start", Int32, callback, callback_args=segment_img)
    rospy.spin()

def main():
    segment_img = Segment()
    segment_server(segment_img)

if __name__ == "__main__":
    main()
