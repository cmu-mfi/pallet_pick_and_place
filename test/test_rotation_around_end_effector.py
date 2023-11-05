#!/usr/bin/env python3

import rospy
import copy
import numpy as np

from geometry_msgs.msg import Pose, Twist

from pallet_pick_and_place.commander import RobotMoveGroup

from autolab_core import RigidTransform

def run():

    rospy.init_node("moveit_controller_move_test")
    robot_mg = RobotMoveGroup()

    while not robot_mg.go_home():
        pass

    T_lego_ee = RigidTransform.load('lego_ee.tf')

    T_ee_world = RigidTransform.from_pose_msg(robot_mg.get_current_pose(), from_frame='ee')

    T_lego_world = T_ee_world * T_lego_ee

    T_lego_rot = RigidTransform(
        rotation=RigidTransform.x_axis_rotation(np.deg2rad(30)),
        from_frame='lego', to_frame='lego'
    )

    T_lego_world_target = T_lego_world * T_lego_rot

    print(T_lego_world_target)

    T_ee_world_target = T_lego_world_target * T_lego_ee.inverse()

    print(T_ee_world_target)

    goal_pose = T_ee_world_target.pose_msg

    while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
        pass

    

if __name__ == "__main__":
    run()