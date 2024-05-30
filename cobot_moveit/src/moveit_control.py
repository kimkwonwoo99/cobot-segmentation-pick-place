#!/usr/bin/env python
import sys, rospy, moveit_commander, time, tf, queue
import geometry_msgs.msg
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
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
    'w' : 0.0
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
start_state = False
hanger_position = queue.Queue()
second_put_mode = None

def cali_service_start() :
    global calibrate_service
    rospy.wait_for_service('calibrate_service')
    try :
        calibrate_service = rospy.ServiceProxy('calibrate_service', basic_service)
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



def move_cobot_and_calib(xyz, ori) :
    state_check_pub.publish(1)
    move_to_pose(xyz, ori)
    cali_service_start()
    print("i return cali_service")

def gripper_move(val) :
    global gripper_service
    rospy.wait_for_service('gripper_service')
    try :
        gripper_service = rospy.ServiceProxy('gripper_service', basic_service)
        gripper_service(val)
        print("i return gripper_service")
        
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
    
def get_tf_position(name) :
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
        return pose
    return None

def third_callback(request) :
    global hanger_position
    
    hanger_pose = hanger_position.get()
    gripper_move(30)
    
    
    #hanger_pose로 이동하기(아직 미구현)
    move_cobot_and_calib(hanger_pose, pose_closet_orientation)
    
    # gripper_move(100)
    
    #밖으로 빠져나오기(미구현)
    
    gripper_move(75)
    hanger_position.queue.clear()
    move_cobot_and_calib(pose_find_closet_position, pose_closet_orientation)
    
    rospy.loginfo("Returning success: %s (type: %s)", True, type(True))        
    return basic_serviceResponse(True)  # 수신완료 리턴

def second_callback(request) :
    global hanger_position, second_put_mode
    print("i return second_service")
    find_pose = get_tf_position(request.class_name)
    print(find_pose)
    tmp_pose = {
        'x': find_pose['x'] + 0.1,
        'y': find_pose['y'],
        'z': find_pose['z']
    }
    print(tmp_pose)
    move_cobot_and_calib(tmp_pose, pose_closet_orientation)
    gripper_move(100)
    hanger_position.put(tmp_pose)
    # #오브젝트 집을곳까지 가는 함수(? 아직 미구현)
    # #마지막 위치 큐에 저장
    
    gripper_move(30)
    
    move_cobot_and_calib(tmp_pose, pose_closet_orientation)
    move_cobot_and_calib(pose_put_cloth_position, pose_person_orientation)
    gripper_move(75)
    # #여기까지가 집고 사람한테 
    
    # if request.mode == 'put' :
    #     second_put_mode = True
    
    rospy.loginfo("Returning success: %s (type: %s)", True, type(True))        
    return second_service_msgResponse(True)  # 수신완료 리턴
    
def robotarm_main_callback(request) :
    print("i receive robotarm_action_service")      #GUI에서 동작 요청 수신 시
    global start_state
    if request.mode == 'start_mode' :       #시작, 카메라 촬영 포지션 이동
        move_cobot_and_calib(pose_find_closet_position, pose_closet_orientation)
        aruco_seg_start_pub.publish(1)      #aruco, seg 시작 명령
        start_state = True
        success = True
    elif request.mode == 'end_mode' :       #종료, 초기 표지션 이동
        move_cobot_and_calib(pose_basic_position, pose_basic_orientation)
        aruco_seg_start_pub.publish(2)      #aruco, seg 종료 명령
        start_state = False
        success = True
    else:
        success = False  # 요청이 잘못된 경우
    
    rospy.loginfo("Returning success: %s (type: %s)", success, type(success))        
    return main_service_msgResponse(success)  # 수신완료 리턴

def main():
    global state_check_pub, start_state, aruco_seg_start_pub, second_put_mode

    # ROS 노드 초기화
    rospy.init_node('dressme_moveit_node', anonymous=True)
    state_check_pub = rospy.Publisher("state_check", Int32, queue_size=10)
    aruco_seg_start_pub = rospy.Publisher("aruco_seg_start", Int32, queue_size=10)
    
    main_server = rospy.Service("robotarm_action_service", main_service_msg, robotarm_main_callback)
    second_service = rospy.Service("second_service", second_service_msg, second_callback)
    third_service = rospy.Service("third_service",basic_service, third_callback)
    
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass