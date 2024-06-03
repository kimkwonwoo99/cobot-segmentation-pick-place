# !/usr/bin/env python

import rospy, tf, cv2
import numpy as np
import cv2.aruco as aruco
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32
from vision_seg.msg import *
from vision_seg.srv import *

class Tf_publisher(object):
    def __init__(self):
        self.control_xy_subscription()
        self.seg_xy_sub = None
        self.aruco_xy_sub = None
        self.seg_pose = None
        self.aruco_all_pose = []
        

    def matrix_to_transform(self, transform_mat):
        translation_vector = transform_mat[:3, 3]
        quaternion = tf.transformations.quaternion_from_matrix(transform_mat)
        return translation_vector, quaternion

    def aruco_xy_callback(self, msg):
        if len(msg.names) > 0:
            self.aruco_all_pose = []
            for i in range(len(msg.names)):
                marker_id = msg.names[i]
                x = msg.x_points[i]
                y = msg.y_points[i]
                self.aruco_all_pose.append([marker_id, x, y])

    def seg_xy_callback(self, msg):
        if len(msg.names) > 0:
            marker_id = msg.names
            x = msg.x_points
            y = msg.y_points
            self.seg_pose = [marker_id, x, y]
            
            
    def control_xy_subscription(self):
        self.aruco_xy_sub = rospy.Subscriber("aruco_cam_xy", aruco_center, self.aruco_xy_callback)
        self.seg_xy_sub = rospy.Subscriber("seg_cam_xy", seg_center, self.seg_xy_callback)
            
    def find_recent_aruco(self) :
        min_val = None
        recent_id = None
        print(self.seg_pose)
        print(self.aruco_all_pose)
        for i in range(len(self.aruco_all_pose)) :
            # x_diff = self.aruco_all_pose[i][1] - self.seg_pose[1]
            y_diff = self.aruco_all_pose[i][1] - self.seg_pose[1]
            
            
            if min_val is None :
                min_val = abs(y_diff)
                recent_id = self.aruco_all_pose[i][0]
            else :
                tmp_val = abs(y_diff)
                min_val = min(min_val, tmp_val)
                if tmp_val == min_val :
                    recent_id = self.aruco_all_pose[i][0]
                    
        print(recent_id)
        return recent_id

def recent_tf_return(req, tf_publisher):
    if req.class_name is not None :
        response = find_tf_serviceResponse()
        response.return_name = str(tf_publisher.find_recent_aruco())
        print("i return recent_tf_service", response.return_name)
    return response
    
    
def tf_server(tf_publisher):
    rospy.init_node('yolo_tf_server')
    rospy.Service("recent_tf_service", find_tf_service, lambda req: recent_tf_return(req, tf_publisher))
    rospy.spin()

def main():
    tf_publisher = Tf_publisher()
    tf_server(tf_publisher)

if __name__ == "__main__":
    main()
