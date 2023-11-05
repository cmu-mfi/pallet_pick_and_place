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

if __name__ == "__main__":
    run()