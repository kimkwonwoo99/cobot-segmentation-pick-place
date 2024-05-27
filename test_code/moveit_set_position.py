#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
import geometry_msgs.msg
import time

def move_to_pose():
    # ROS 노드 초기화
    rospy.init_node('move_to_pose_node', anonymous=True)
    # MoveIt! 초기화
    moveit_commander.roscpp_initialize(sys.argv)
    
    # 로봇 모델 로딩
    robot = moveit_commander.RobotCommander()

    # 로봇 팔 그룹 초기화
    arm_group = moveit_commander.MoveGroupCommander("arm_group")
    
    current_joint_values = arm_group.get_current_joint_values()
    print("Current joint values:", current_joint_values)
    
    # 목표 포즈 생성
    pose_goal = geometry_msgs.msg.Pose()
    # target_pose = PoseStamped()
    pose_goal.position.x = 0
    pose_goal.position.y = -0.17
    pose_goal.position.z = 0.37
    pose_goal.orientation.x = 0
    pose_goal.orientation.y = -0.7071080795300609
    pose_goal.orientation.z = 0.707105478539869
    pose_goal.orientation.w = 0
    
    arm_group.set_pose_target(pose_goal)
    plan = arm_group.go(wait=True)  # 이동이 완료될 때까지 대기
    
    if plan:
        print("Movement successful!")
    else:
        print("Movement failed!")
    
    arm_group.stop()
    arm_group.clear_pose_targets()


if __name__ == '__main__':
    try:
        move_to_pose()
    except rospy.ROSInterruptException:
        pass