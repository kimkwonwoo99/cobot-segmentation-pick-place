#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
import geometry_msgs.msg
import time
import tf
import queue
from cobot_moveit.srv import *


#옷장 바라보는 방향
pose_closet_orientation = ({
    'x' : -0.7071080795300609,
    'y' : 0,
    'z' : 0.7071080795300609,
    'w' : 0
})
#사람 바라보는 방향
pose_person_orientation = ({
    'x' : 0,
    'y' : 0.7071080795300609,
    'z' : 0,
    'w' : 0.7071080795300609
})
#초기상태
pose_basic_orientation = ({
    'x' : 0.0,
    'y' : 0.7071080795300609,
    'z' : -0.7071080795300609,
    'w' : 1.0
})
pose_find_closet_position = ({
    'x' : 0.0,
    'y' : -0.08,
    'z' : 0.4
})
pose_put_cloth_position = ({
    'x' : 0.25,
    'y' : -0.08,
    'z' : 0.45
})
pose_basic_position = ({
    'x' : 0.0,
    'y' : -0.292,
    'z' : 0.523
})

aruco_seg_start_pub = None
move_start_pub = None
calibrate_service = None
gripper_service = None
start_state = True
robotarm_state = 'end_mode'
hanger_position = queue.Queue()

def cali_service_start() :
    global calibrate_service
    rospy.wait_for_service('calibrate_service')
    try :
        calibrate_service = rospy.ServiceProxy('calibrate_service', cali_service_msg)
        calibrate_service(1)
        
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def move_to_pose(pose, ori):
    time.sleep(1)
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()

    arm_group = moveit_commander.MoveGroupCommander("arm_group")
    arm_group.set_planner_id("RRTConnectkConfigDefault")
    # 계획 시도 횟수 설정
    arm_group.set_num_planning_attempts(30)
    
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position = geometry_msgs.msg.Point(**pose)
    pose_goal.orientation = geometry_msgs.msg.Quaternion(**ori)

    arm_group.set_pose_target(pose_goal)
    plan = arm_group.go(wait=True)  # 이동이 완료될 때까지 대기
    if plan:
        print("Movement successful!")
    else:
        print("Movement failed!")
    arm_group.stop()
    arm_group.clear_pose_targets()
    time.sleep(1)

def robotarm_main_callback(request) :
    print("i receive robotarm_action_service")
    global robotarm_state
    if request.data == 'start_mode' :
        move_cobot_and_calib(pose_find_closet_position, pose_closet_orientation)
        robotarm_state = request.data
        aruco_seg_start_pub.publish(1)
        return True
    elif request.data == 'end_mode' :
        move_cobot_and_calib(pose_basic_position, pose_basic_orientation)
        robotarm_state = request.data
        aruco_seg_start_pub.publish(2)
        return True

def move_cobot_and_calib(xyz, ori) :
    state_check_pub.publish(1)
    move_to_pose(xyz, ori)
    cali_service_start()
    print("i return cali_service")

def gripper_move(val) :
    global gripper_service
    rospy.wait_for_service('gripper_service')
    try :
        gripper_service = rospy.ServiceProxy('gripper_service', grip_service_msg)
        gripper_service(val)
        print("i return gripper_service")
        
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
    
def find_tf_position(name) :
    global hanger_position
    listener = tf.TransformListener()
    marker_frame = name
    listener.waitForTransform("base", marker_frame, rospy.Time(0), rospy.Duration(4.0))
    if listener.canTransform("base", marker_frame, rospy.Time(0)):
        (trans, rot) = listener.lookupTransform("base", marker_frame, rospy.Time(0))
        pose = {
            'x': trans[0],
            'y': trans[1],
            'z': trans[2]
        }
        hanger_position.put(pose)
        return pose
    return None

def pick_callback(request) :
    
    find_position  = find_tf_position(request.name)
    tmp_position = [find_position['x'] + 0.05, find_position['y'], find_position['z']]
    move_cobot_and_calib(tmp_position, pose_closet_orientation)
    
    gripper_move(100)
    
    move_cobot_and_calib(find_position, pose_closet_orientation)
    
    gripper_move(0)
    
    move_cobot_and_calib(tmp_position, pose_closet_orientation)
    
    move_cobot_and_calib(pose_put_cloth_position, pose_person_orientation)
    
    gripper_move(50)
    
    return True
    
def last_callback(request) :
    global hanger_position
    hanger_pose = hanger_position.get()
    if request.modename == 'put_in_mode' :
        gripper_move(0)
        
        move_to_pose(pose_put_cloth_position, pose_closet_orientation)
        
        move_to_pose(hanger_pose, pose_closet_orientation)
        
        gripper_move(100)
        
        tmp_pose = [hanger_pose['x'] + 0.05, hanger_pose['y'], hanger_pose['z']]
        move_to_pose(tmp_pose, pose_closet_orientation)
        
        gripper_move(50)
        
    hanger_position.queue.clear()
    move_to_pose(pose_find_closet_position, pose_closet_orientation)

def start_callback(request) :
    move_cobot_and_calib(pose_find_closet_position, pose_closet_orientation)
    
    
def main():
    global state_check_pub, start_state, second_state, aruco_seg_start_pub

    # ROS 노드 초기화
    rospy.init_node('dressme_moveit_node', anonymous=True)
    state_check_pub = rospy.Publisher("state_check", Int32, queue_size=10)
    aruco_seg_start_pub = rospy.Publisher("aruco_seg_start", Int32, queue_size=10)
    
    main_server = rospy.Service("robotarm_action_service", main_service_msg, robotarm_main_callback)
    second_service = rospy.Service("pick_service", main_service_msg, pick_callback)
    thrid_service = rospy.Service("last_service", main_service_msg, last_callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass