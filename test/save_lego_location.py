#!/usr/bin/env python3

import rospy
import copy
import numpy as np

from geometry_msgs.msg import Pose, Twist

from pallet_pick_and_place.commander import RobotMoveGroup

from autolab_core import RigidTransform

def run():

    rospy.init_node("moveit_controller_move_test")
    robot_mg = RobotMoveGroup('yk_builder')
    
    T_hook_ee = RigidTransform.load('lego_ee.tf')

    T_ee_world = RigidTransform.from_pose_msg(robot_mg.get_current_pose(), from_frame='ee')

    T_hook_world = T_ee_world * T_lego_ee

    T_hook_world.save('lego_tray2.tf')

if __name__ == "__main__":
    run()