#!/usr/bin/env python3

import rospy
import copy
import numpy as np

from geometry_msgs.msg import Pose, Twist

from pallet_pick_and_place.commander import RobotMoveGroup

def run():

    rospy.init_node("moveit_controller_move_test")
    robot_mg = RobotMoveGroup()

    while not robot_mg.go_home():
        pass

    intermediate_pose1 = Pose()
    intermediate_pose1.position.x = 0.2574481493687531
    intermediate_pose1.position.y = -0.14781143771263844
    intermediate_pose1.position.z = 0.4985763045591675
    intermediate_pose1.orientation.x = 0.8191431715663031
    intermediate_pose1.orientation.y = 3.087110082639039e-05
    intermediate_pose1.orientation.z = 9.515720642708427e-06
    intermediate_pose1.orientation.w = 0.5735891067939874

    while not robot_mg.go_to_pose_goal(intermediate_pose1, cartesian_path=True):
        pass

    first_joints = [ -0.30189576745033264, 0.4360671937465668, -0.6037091016769409, 1.6461249589920044, -1.1191291809082031, -1.0034152269363403] #[-0.32576847076416016, 0.42281055450439453, -0.5853232145309448, 1.6089653968811035, -1.0507993698120117, -0.9496491551399231]
#  # Before Before Tray Pose

    second_joints = [ 0.05091829225420952, 0.41245147585868835, -0.6433296203613281, 1.3506475687026978, -1.2932360172271729, -1.033941388130188] # [0.016644733026623726, 0.3857109546661377, -0.6455429196357727, 1.3351178169250488, -1.226372480392456, -0.9939045310020447]
# # Before Tray Pose
    
    third_joints = [ -0.07511338591575623, 0.268636018037796, -0.6330300569534302, 1.1462182998657227, -1.0075321197509766, -0.7503083944320679] #[-0.13599137961864471, 0.2455126792192459, -0.6220073103904724, 1.0669569969177246, -0.9239978194236755, -0.6391714811325073]
      #  

    #fourth_joints = [-0.15570715069770813, 0.22719959914684296, -0.6151481866836548, 1.0167491436004639, -0.9018002152442932, -0.5770068764686584]


    while not robot_mg.go_to_joint_state(first_joints, cartesian_path=True):
        pass


    while not robot_mg.go_to_joint_state(second_joints, cartesian_path=True):
        pass

    while not robot_mg.go_to_joint_state(third_joints, cartesian_path=True):
        pass

    #robot_mg.go_to_joints(fourth_joints)

    # # check movements with joint targets

    # # robot_mg.go_home()

    intermediate_pose2 = robot_mg.get_current_pose()

    intermediate_pose2.position.y -= 0.33

    while not robot_mg.go_to_pose_goal(intermediate_pose2, cartesian_path=True):
        pass

    # # # robot_mg.set_acceleration()


    # # # robot_mg.set_velocity()

    intermediate_pose3 = robot_mg.get_current_pose()

    intermediate_pose3.position.z += 0.1

    while not robot_mg.go_to_pose_goal(intermediate_pose3, cartesian_path=True):
        pass

    intermediate_pose4 = robot_mg.get_current_pose()

    intermediate_pose4.orientation.x = 0.9407525201331457
    intermediate_pose4.orientation.y = -2.345129415937732e-06
    intermediate_pose4.orientation.z = 6.719047727794245e-05
    intermediate_pose4.orientation.w = 0.33909392702181407

    while not robot_mg.go_to_pose_goal(intermediate_pose4, cartesian_path=True):
        pass

    while not robot_mg.go_to_pose_goal(intermediate_pose3, cartesian_path=True):
        pass

    while not robot_mg.go_to_pose_goal(intermediate_pose2, cartesian_path=True):
        pass

    while not robot_mg.go_to_joint_state(third_joints, cartesian_path=True):
        pass

    while not robot_mg.go_to_joint_state(second_joints, cartesian_path=True):
        pass

    while not robot_mg.go_to_joint_state(first_joints, cartesian_path=True):
        pass

    while not robot_mg.go_to_pose_goal(intermediate_pose1, cartesian_path=True):
        pass

    while not robot_mg.go_home():
        pass

    #print(current_pose)


    # target_joint = [0, 0, 0, 0, 0, 0]
    # rospy.loginfo("Testing motion with joint space interpolation...")
    # if robot_mg.go_to_joint_state(target_joint, cartesian_path=False):
    #     rospy.loginfo("Run successful.")
    # else:
    #     rospy.logerr("Test failed!")

    # print(robot_mg.get_current_pose())

    # # check movements with pose targets
    # pose c
    # rospy.loginfo("Testing motion with cartesian interpolation...")
    # if robot_mg.go_to_pose_goal(pose, cartesian_path=True):
    #     rospy.loginfo("Run successful.")
    # else:
    #     rospy.logerr("Test failed!")
    # # check pose trajectory control
    # pose_list = []
    # for i in range(4):
    #     pose_list.append(copy.deepcopy(pose))
    #     pose_list[-1].position.y += 0.05 * (i + 1)
    # rospy.loginfo("Testing cartesian position trajectory controller...")
    # robot_mg.send_cartesian_pos_trajectory(pose_list, wait=True)
    # check velocity trajectory control

if __name__ == "__main__":
    run()