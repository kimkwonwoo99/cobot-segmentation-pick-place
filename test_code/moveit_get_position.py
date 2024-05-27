import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
import math

def print_current_pose():
    # ROS 노드 초기화
    rospy.init_node('print_current_pose_node', anonymous=True)

    # MoveIt! 초기화
    moveit_commander.roscpp_initialize(sys.argv)

    # 로봇 모델 로딩
    robot = moveit_commander.RobotCommander()

    # 로봇 팔 그룹 초기화
    arm_group = moveit_commander.MoveGroupCommander("arm_group")

    # 현재 조인트 값 받아오기
    current_joint_values = arm_group.get_current_joint_values()
    print("Current joint values:", current_joint_values)

    current_joint_values_degrees = [math.degrees(joint) for joint in current_joint_values]
    print("Current joint values (degrees):", current_joint_values_degrees)
    
    
    current_pose = arm_group.get_current_pose()
    print("Current pose:", current_pose.pose)

    # 종료 전에 MoveIt! 초기화 해제
    moveit_commander.roscpp_shutdown()
if __name__ == "__main__":
    try:
        print_current_pose()
    except rospy.ROSInterruptException:
        pass
