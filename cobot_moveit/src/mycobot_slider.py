#!/usr/bin/env python
# -*- coding:utf-8 -*-
import math, time, rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from pymycobot.mycobot import MyCobot

from cobot_moveit.srv import cali_service_msg, grip_service_msg 

reword_list = [0, 0, 0, 0, 0, 0]  # 6개의 요소를 포함하는 리스트로 수정
state = True
mc = None
data_list = None
def pid() :
    global mc, data_list, state
    tolerance = 0.5
    new_angles = list(data_list)  # 현재 각도를 복사하여 새로운 각도 리스트 생성
    
    while True :
        comp_list = mc.get_angles()
        print("get_value", comp_list)
        
        errors = [abs(comp_list[i] - data_list[i]) for i in range(6)]
        if all(error <= tolerance for error in errors) :
            print("pid complete!")
            break
        
        for i in range(6) :
            if errors[i] > tolerance :
                correction = (data_list[i] - comp_list[i]) / 2
                new_angles[i] += correction
        
        mc.send_angles(new_angles, 40)
        time.sleep(0.1)
       
def callback(data):
    global state, mc, data_list, reword_list
    
    if state and data is not None:
        tmp_list = [0,0,0,0,0,0]
        data_list = []
        for index, value in enumerate(data.position):
            radians_to_angles = round(math.degrees(value), 2)
            data_list.append(radians_to_angles)
        print("data_list", data_list)
        for i in range(6):
            tmp_list[i] = data_list[i] + reword_list[i]
        print("sub_angles", tmp_list)
        mc.send_angles(tmp_list, 40)
        time.sleep(0.03)
        get_angle_list = mc.get_angles()
        print("get_angles", get_angle_list)
        for i in range(6):
            reword_list[i] = 0  # 각 데이터 샘플마다 reword_list 초기화
            reword_list[i] = round((tmp_list[i] - get_angle_list[i]) * 0.5,3)
        
def state_callback(data):
    global state
    if data.data == 1:
        state = True
        print("move stop")

def cali_callback(data):
    if data.signal == 1 :
        print("i receive cali_service")
        global state
        state = False
        pid()
    return True
    
def gripper_callback(data):
    print("i receive gripper_service")
    mc.set_gripper_mode(0)
    mc.set_eletric_gripper(0)
    mc.set_gripper_value(data.value,50)
    
    while True:
        gripper_value = mc.get_gripper_value()
        if gripper_value == data:
            break
    rospy.loginfo("gripper value is %d", data.value)
    return True

def listener():
    global mc
    port = rospy.get_param("~port", "/dev/ttyACM0")
    baud = rospy.get_param("~baud", 115200)
    print(port, baud)
    mc = MyCobot(port, baud)
    rospy.init_node("control_slider", anonymous=True)
    rospy.Subscriber("joint_states", JointState, callback)
    rospy.Subscriber("state_check", Int32 , state_callback)
    rospy.Service("calibrate_service", cali_service_msg , cali_callback)
    rospy.Service("gripper_service", grip_service_msg , gripper_callback)
    time.sleep(0.02)
    mc.set_fresh_mode(1)
    time.sleep(0.03)
    print("spin ...")
    rospy.spin()

if __name__ == "__main__":
    listener()