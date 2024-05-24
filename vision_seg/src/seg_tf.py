# !/usr/bin/env python

import rospy, tf, cv2
import numpy as np
import cv2.aruco as aruco
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32
from vision_seg.msg import aruco_center

class Tf_publisher(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.aruco_xy_sub = None
        self.seg_xy_sub = None
        self.server_state = False

    def matrix_to_transform(self, transform_mat):
        translation_vector = transform_mat[:3, 3]
        quaternion = tf.transformations.quaternion_from_matrix(transform_mat)
        return translation_vector, quaternion

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv2_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


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
            

        if self.image_state:  # Check the state before broadcasting
            for marker_id, transform in self.last_detect_position.items():
                if transform is not None:
                    pos, quat = transform
                    self.tf_broadcaster.sendTransform(
                        pos,
                        quat,
                        rospy.Time.now(),
                        "marker_{}".format(marker_id),
                        "cam_lens_link"
                    )

    def control_xy_subscribtion(self, state):
        if state and (self.seg_xy_sub is None or self.aruco_xy_sub is None):
            self.seg_xy_sub = rospy.Subscriber("seg_cam_xy", aruco_center, self.seg_callback)
            self.aruco_xy_sub = rospy.Subscriber("arco_cam_xy", aruco_center, self.aruco_xy_callback)
        elif not state and self.seg_xy_sub is not None and self.aruco_xy_sub is not None:
            self.seg_xy_sub.unregister()
            self.aruco_xy_sub.unregister()
            self.seg_xy_sub = None
            self.aruco_xy_sub = None

def callback(msg, tf_publisher):
    if msg.data == 1:
        if not tf_publisher.server_state:
            rospy.loginfo("Starting tf publisher")
            tf_publisher.server_state = True
            tf_publisher.control_xy_subscribtion(True)
    elif msg.data == 2:
        if aruco.image_state:
            rospy.loginfo("Stopping tf publisher")
            tf_publisher.server_state = False
            tf_publisher.control_xy_subscribtion(False)

def tf_server(tf_publisher):
    rospy.init_node('yolo_tf_server')
    rospy.Subscriber("aruco_seg_start", Int32, callback, callback_args=tf_publisher)
    rospy.spin()

def main():
    tf_publisher = Tf_publisher()
    tf_server(tf_publisher)

if __name__ == "__main__":
    main()
