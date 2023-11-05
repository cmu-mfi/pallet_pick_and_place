#!/usr/bin/env python3

import rospy
import copy
import numpy as np

from geometry_msgs.msg import Pose, Twist

from pallet_pick_and_place.commander import RobotMoveGroup

def run():

    rospy.init_node("moveit_controller_move_test")
    robot_mg = RobotMoveGroup()
    print(robot_mg.get_current_joints())
    print(robot_mg.get_current_pose())

    # check movements with joint targets

    robot_mg.go_home()

    current_pose = robot_mg.get_current_pose()

    current_pose.position.y -= 0.3

    robot_mg.set_acceleration()
    robot_mg.set_velocity()
    robot_mg.go_to_pose_goal(current_pose, cartesian_path=True)

    print(current_pose)


    target_joint = [0, 0, 0, 0, -np.pi/2, 0]
    rospy.loginfo("Testing motion with joint space interpolation...")
    if robot_mg.go_to_joint_state(target_joint, cartesian_path=False):
        rospy.loginfo("Run successful.")
    else:
        rospy.logerr("Test failed!")

    print(robot_mg.get_current_pose())

    # check movements with pose targets
    pose = Pose()
    pose.position.x = 0.3
    pose.position.y = 0
    pose.position.z = 0.5
    pose.orientation.x = 1.0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 0
    rospy.loginfo("Testing motion with cartesian interpolation...")
    if robot_mg.go_to_pose_goal(pose, cartesian_path=True):
        rospy.loginfo("Run successful.")
    else:
        rospy.logerr("Test failed!")
    # check pose trajectory control
    pose_list = []
    for i in range(4):
        pose_list.append(copy.deepcopy(pose))
        pose_list[-1].position.y += 0.05 * (i + 1)
    rospy.loginfo("Testing cartesian position trajectory controller...")
    robot_mg.send_cartesian_pos_trajectory(pose_list, wait=True)


if __name__ == "__main__":
    run()