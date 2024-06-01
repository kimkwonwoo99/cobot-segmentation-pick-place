# !/usr/bin/env python

import rospy, tf, cv2
import numpy as np
import cv2.aruco as aruco
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32
from vision_seg.msg import aruco_center

class Aruco(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.MARKERLEN = 0.05  # marker length in meter
        self.DICT_GET = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)  # marker 5x5_1000
        self.ARUCO_PARAMETERS = cv2.aruco.DetectorParameters_create()
        self.camera_matrix = np.array([[542.93802581, 0, 329.25053673], [0, 541.67327024, 256.79448482], [0, 0, 1]])  # 카메라 행렬로 설정
        self.dist_coeffs = np.array([[0.19266232, -0.79141779, -0.00253703, 0.00613584, 1.04252319]])  # 왜곡 계수로 설정
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.aruco_xy_publisher = rospy.Publisher("aruco_cam_xy", aruco_center, queue_size=10)        
        self.last_detect_position = {}
        self.image_sub = None
        self.image_state = False
        self.find_two_markers = False

    def matrix_to_transform(self, transform_mat):
        translation_vector = transform_mat[:3, 3]
        quaternion = tf.transformations.quaternion_from_matrix(transform_mat)
        return translation_vector, quaternion

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv2_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(cv2_img, self.DICT_GET, parameters=self.ARUCO_PARAMETERS)

        if markerIds is not None and len(markerCorners) > 0:
            names = []
            x_points = []
            y_points = []
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, self.MARKERLEN, self.camera_matrix, self.dist_coeffs)
            for i, marker_id in enumerate(markerIds):
                #marker_centor
                marker_corners = markerCorners[i][0]
                top_left = marker_corners[0]
                bottom_right = marker_corners[2]

                # 마커의 중심 좌표 계산
                center_x = int((top_left[0] + bottom_right[0]) / 2)
                center_y = int((top_left[1] + bottom_right[1]) / 2)
                names.append(str(marker_id[0]))
                x_points.append(center_x)
                y_points.append(center_y)              
                # Pose estimation
                rmat, _ = cv2.Rodrigues(rvecs[i])
                transform_mat = np.eye(4)
                transform_mat[:3, :3] = rmat
                transform_mat[:3, 3] = tvecs[i].reshape(-1)
                original_translation, original_quaternion = self.matrix_to_transform(transform_mat)

                self.last_detect_position[marker_id[0]] = original_translation, original_quaternion
            
            if len(names) > 0 :
                aruco_msg = aruco_center()
                aruco_msg.names = names
                aruco_msg.x_points = x_points
                aruco_msg.y_points = y_points
                self.find_two_markers = True
                self.aruco_xy_publisher.publish(aruco_msg)
            else :
                self.find_two_markers = False

        if self.image_state and self.find_two_markers:  # Check the state before broadcasting
            for marker_id, transform in self.last_detect_position.items():
                if transform is not None:
                    pos, quat = transform
                    self.tf_broadcaster.sendTransform(
                        pos,
                        quat,
                        rospy.Time.now(),
                        "marker_{}".format(marker_id),
                        "robotarm/cam_lens_link"
                    )

    def control_image_subscription(self, state):
        if state and self.image_sub is None:
            self.image_sub = rospy.Subscriber("/camera/arm/compressed", CompressedImage, self.image_callback)
        elif not state and self.image_sub is not None:
            self.image_sub.unregister()
            self.image_sub = None

def callback(msg):
    if msg.data == 1:
        if not aruco.image_state:
            rospy.loginfo("Starting tf publisher")
            aruco.image_state = True
            aruco.control_image_subscription(True)
    elif msg.data == 2:
        if aruco.image_state:
            rospy.loginfo("Stopping tf publisher")
            aruco.image_state = False
            aruco.control_image_subscription(False)

def matrix_calculation_server():
    rospy.init_node('matrix_calculation_server')
    rospy.Subscriber("aruco_start", Int32, callback)
    rospy.spin()

def main():
    global aruco
    aruco = Aruco()
    matrix_calculation_server()

if __name__ == "__main__":
    main()