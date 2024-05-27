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
        self.aruco_all_pose = None
        self.seg_xy_sub = None
        self.server_state = False

    def matrix_to_transform(self, transform_mat):
        translation_vector = transform_mat[:3, 3]
        quaternion = tf.transformations.quaternion_from_matrix(transform_mat)
        return translation_vector, quaternion

    def aruco_xy_callback(self, msg):
        if len(msg.names) == 2:  # Ensure that two aruco markers are detected
            if self.aruco_all_pose is None:
                self.aruco_all_pose = [[] for _ in range(2)]
            for i in range(len(msg.names)):
                marker_id = msg.names[i]
                x = msg.x_points[i]
                y = msg.y_points[i]
                
                listener = tf.TransformListener()
                marker_frame = "marker_{}".format(marker_id)
                listener.waitForTransform("cam_lens_link", marker_frame, rospy.Time(0), rospy.Duration(4.0))
                if listener.canTransform("cam_lens_link", marker_frame, rospy.Time(0)):
                    (trans, rot) = listener.lookupTransform("cam_lens_link", marker_frame, rospy.Time(0))
                    self.aruco_all_pose[i] = [marker_id, x, y, trans[0], trans[1], trans[2]]
                    print("done")
                print(trans)
                

    def seg_xy_callback(self, msg):
        if self.aruco_all_pose is not None :
            print(self.aruco_all_pose)
            for i in range(len(msg.names)):
                print(msg)
                
                aruco_id = msg.names[i]
                x = (msg.x_points[i] - self.aruco_all_pose[0][1]) / (self.aruco_all_pose[1][1] - self.aruco_all_pose[0][1])
                y = (msg.y_points[i] - self.aruco_all_pose[0][2]) / (self.aruco_all_pose[1][2] - self.aruco_all_pose[0][2])
                tf_x = self.aruco_all_pose[0][3] + x * (self.aruco_all_pose[1][3] - self.aruco_all_pose[0][3])
                tf_y = self.aruco_all_pose[0][4] + y * (self.aruco_all_pose[1][4] - self.aruco_all_pose[0][4])
                tf_z = (self.aruco_all_pose[0][5] + self.aruco_all_pose[1][5]) / 2
                print(aruco_id, x, y, tf_x, tf_y, tf_z)
                seg_translation, seg_quaternion = self.matrix_to_transform(np.array([[1, 0, 0, tf_x], [0, 1, 0, tf_y], [0, 0, 1, tf_z], [0, 0, 0, 1]]))
                self.tf_broadcaster.sendTransform(
                    seg_translation,
                    seg_quaternion,
                    rospy.Time.now(),
                    "seg_{}".format(aruco_id),
                    "cam_lens_link"
                )
                
    def control_xy_subscribtion(self, state):
        if state:
            self.aruco_xy_sub = rospy.Subscriber("arco_cam_xy", aruco_center, self.aruco_xy_callback)
            self.seg_xy_sub = rospy.Subscriber("seg_cam_xy", aruco_center, self.seg_xy_callback)
            
        else:
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
