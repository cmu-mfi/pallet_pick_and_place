#!/usr/bin/env python3

import rospy
import copy
import numpy as np

from geometry_msgs.msg import Pose, Twist

from pallet_pick_and_place_package.commander import RobotMoveGroup

from autolab_core import RigidTransform

def run():

    rospy.init_node("save_lego_pose")

    namespace = rospy.get_param("save_lego_pose/namespace")
    root_pwd = rospy.get_param("save_lego_pose/root_pwd")

    robot_mg = RobotMoveGroup(namespace)
    
    T_lego_ee = RigidTransform.load(root_pwd+'/config/lego_ee.tf')

    T_ee_world = RigidTransform.from_pose_msg(robot_mg.get_current_pose(), from_frame='ee')

    T_lego_world = T_ee_world * T_lego_ee

    T_lego_world.save(root_pwd+'/config/yk_creator_lego_tray_2.tf')

if __name__ == "__main__":
    run()
