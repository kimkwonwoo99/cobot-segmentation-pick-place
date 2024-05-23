#!/usr/bin/env python
# -*- coding:utf-8 -*-
"""[summary]
This file obtains the joint angle of the manipulator in ROS,
and then sends it directly to the real manipulator using `pymycobot` API.
This file is [slider_control.launch] related script.
Passable parameters:
    port: serial prot string. Defaults is '/dev/ttyUSB0'
    baud: serial prot baudrate. Defaults is 115200.
"""
import math
import time
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from pymycobot import MyCobotSocket


state =True
mc = None

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "%s", data.position)
    global state
    if state :
        data_list = []
        for index, value in enumerate(data.position):
            radians_to_angles = round(math.degrees(value), 2)
            data_list.append(radians_to_angles)
        rospy.loginfo(rospy.get_caller_id() + "%s", data_list)
        mc.send_angles(data_list, 40)
        
        
def callback1_state(data):
    global state
    if data.data == 1:
        state = True
    else :
        state = False
    
def callback1_gripper(data):
    # mc.set_gripper_mode(0)
    # time.sleep(3) 
    for i in range(3):
        mc.set_gripper_mode(0)
        time.sleep(3)
        mc.set_eletric_gripper(0)
        mc.set_gripper_value(10 + data.data,50)
        time.sleep(3)
        mc.set_eletric_gripper(1)
        mc.set_gripper_value(90 + data.data,50)
        time.sleep(3)
        rospy.loginfo("Task iteration %d completed", i+1)
def listener():
    global mc
   
    rospy.init_node("control_slider", anonymous=True)
    rospy.Subscriber("joint_states", JointState, callback)
    rospy.Subscriber("state_check", Int32 , callback1_state)
    rospy.Subscriber("gripper_check", Int32 , callback1_gripper)
    mc = MyCobotSocket("172.20.10.2")
    time.sleep(0.05)
    mc.set_fresh_mode(1)
    time.sleep(0.05)
    # spin() simply keeps python from exiting until this node is stopped
    print("spin ...")
    rospy.spin()


if __name__ == "__main__":
    listener()