#!/usr/bin/env python3

import rospy
import copy
import numpy as np

from geometry_msgs.msg import Pose, Twist

from pallet_pick_and_place.commander import RobotMoveGroup

from autolab_core import RigidTransform

def run():

    rospy.init_node("pilot_tray1")
    robot_mg = RobotMoveGroup()

    print("Returning to Home")
    while not robot_mg.go_home():
        pass

    T_hook_ee = RigidTransform.load('hook_ee.tf')

    T_tray_world = RigidTransform.load('tray_1.tf')

    start_T_ee_world = RigidTransform.from_pose_msg(robot_mg.get_current_pose(), from_frame='ee')

    start_T_hook_world = start_T_ee_world * T_hook_ee

    T_hook_rot_70 = RigidTransform(
        rotation=RigidTransform.x_axis_rotation(np.deg2rad(-70)),
        from_frame='hook', to_frame='hook'
    )

    T_hook_rot_42 = RigidTransform(
        rotation=RigidTransform.x_axis_rotation(np.deg2rad(-42)),
        from_frame='hook', to_frame='hook'
    )

    T_hook_rot_40 = RigidTransform(
        rotation=RigidTransform.x_axis_rotation(np.deg2rad(-40)),
        from_frame='hook', to_frame='hook'
    )

    T_hook_world_target_70 = start_T_hook_world * T_hook_rot_70

    T_hook_world_target_42 = start_T_hook_world * T_hook_rot_42

    T_hook_world_target_70.translation = T_tray_world.translation + np.array([0,-0.08,0.2])

    T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

    goal_pose = T_ee_world_target.pose_msg

    print("Rotating end effector to -70 degrees and moving -8cm in y and +20cm in z from tray location")

    while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
        pass

    T_hook_world_target_70.translation = T_tray_world.translation + np.array([0,-0.08,0])

    T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

    goal_pose = T_ee_world_target.pose_msg

    print("Moving down 20cm")

    while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
        pass

    T_hook_world_target_70.translation = T_tray_world.translation

    T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

    goal_pose = T_ee_world_target.pose_msg

    print("Moving to the left 8cm")

    while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
        pass


    T_hook_world_target_42.translation = T_tray_world.translation

    T_ee_world_target = T_hook_world_target_42 * T_hook_ee.inverse()

    goal_pose = T_ee_world_target.pose_msg

    print("Rotating end-effector 28 degrees around x")

    while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
        pass

    T_ee_world_target = RigidTransform.from_pose_msg(robot_mg.get_current_pose(), from_frame='ee')

    T_ee_world_target.translation[1] -= 0.33

    goal_pose = T_ee_world_target.pose_msg

    print("Moving to the right 33cm")

    while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
        pass

    T_hook_world_target_40 = start_T_hook_world * T_hook_rot_40

    T_ee_world_target = RigidTransform.from_pose_msg(robot_mg.get_current_pose(), from_frame='ee')

    T_ee_world_target.translation[2] += 0.1

    T_hook_world_target_40.translation = T_ee_world_target.translation
    
    goal_pose = T_hook_world_target_40.pose_msg

    print("Moving up 10cm while rotating 2 degrees around x")

    while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
        pass


if __name__ == "__main__":
    run()