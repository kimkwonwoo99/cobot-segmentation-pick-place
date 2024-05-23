#!/usr/bin/env python
# -*- coding:utf-8 -*-
import math
import time
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from pymycobot import MyCobotSocket
from pymycobot.mycobot import MyCobot
import tensorflow as tf

state = True
mc = None

model_path = '/home/choi/catkin_ws/src/mycobot_ros/mycobot_320/new_mycobot_320/scripts/my_model3'

# 모델 로드
model = tf.saved_model.load(model_path)

tensor_data = tf.constant([[34.32, -7.53, 78.53, -71.04, -124.35, 89.94]], dtype=tf.float32)

prediction = model(tensor_data)

prediction_list = prediction.numpy().tolist()

print(prediction_list)