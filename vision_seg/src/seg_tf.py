#!/usr/bin/env python

import rospy
import tf
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32
from ultralytics import YOLO

segment_img = None
model = YOLO("bset7.pt")

class Segment(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.camera_matrix = np.array([[527.43921935, 0, 317.20828396], [0, 531.15346504, 240.4016465], [0, 0, 1]])  # 카메라 행렬로 설정
        self.dist_coeffs = np.array([[0.20891653, -1.69077085, -0.00999435, -0.00851377, 5.10335346]])  # 왜곡 계수로 설정
        self.tf_broadcaster = tf.TransformBroadcaster()
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
            for result in results:
                if result.masks:  # 이 조건을 확인
                    for j, mask in enumerate(result.masks.data):
                        mask = mask.cpu().numpy()
                        # RGBA 이미지 생성
                        rgba_img = np.zeros((H, W, 4), dtype=np.uint8)

                        # 마스크 리사이징과 투명도 적용
                        mask_resized = cv2.resize(mask, (W, H))
                        mask_3d = (mask_resized > 0.5).astype(np.uint8)  # 마스크 임계값 조정
                        
        

    def control_image_subscription(self, state):
        if state and self.image_sub is None:
            self.image_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.image_callback)
        elif not state and self.image_sub is not None:
            self.image_sub.unregister()
            self.image_sub = None

def callback(msg):
    if msg.data == 1:
        if not Segment.image_state:
            rospy.loginfo("Starting tf publisher")
            Segment.image_state = True
            Segment.control_image_subscription(True)
    elif msg.data == 2:
        if Segment.image_state:
            rospy.loginfo("Stopping tf publisher")
            Segment.image_state = False
            Segment.control_image_subscription(False)

def segment_server():
    rospy.init_node('yolo_segment_server')
    rospy.Subscriber("aruco_seg_start", Int32, callback)
    rospy.spin()

def main():
    global segment_img
    segment_img = Segment()
    segment_server()

if __name__ == "__main__":
    main()
