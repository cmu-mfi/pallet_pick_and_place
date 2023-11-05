#!/usr/bin/env python3

import rospy
import copy
import numpy as np

from geometry_msgs.msg import Pose, Twist

from pallet_pick_and_place.commander import RobotMoveGroup

from autolab_core import RigidTransform

LEGO_BLOCK_HEIGHT=0.0096 #z
LEGO_BLOCK_WIDTH=0.0158 #x
LEGO_BLOCK_LENGTH=0.0318 #y

def run():

    rospy.init_node("test_lego_build")

    namespace = rospy.get_param("test_lego_build/namespace")
    root_pwd = rospy.get_param("test_lego_build/root_pwd")

    robot_mg = RobotMoveGroup(namespace)
    
    T_lego_ee = RigidTransform.load(root_pwd+'/config/lego_ee.tf')

    T_lego1_tray2_world = RigidTransform.load(root_pwd+'/config/lego_tray2.tf')

    T_lego2_tray2_world = T_lego1_tray2_world.copy()
    T_lego2_tray2_world.translation += np.array([LEGO_BLOCK_LENGTH * 3 / 2, LEGO_BLOCK_WIDTH, 0])
    T_lego3_tray2_world = T_lego1_tray2_world.copy()
    T_lego3_tray2_world.translation += np.array([0, 2*LEGO_BLOCK_WIDTH, 0])
    T_lego4_tray2_world = T_lego1_tray2_world.copy()
    T_lego4_tray2_world.translation += np.array([LEGO_BLOCK_LENGTH * 3 / 2, 3*LEGO_BLOCK_WIDTH,  0])
    T_lego5_tray2_world = T_lego1_tray2_world.copy()
    T_lego5_tray2_world.translation += np.array([0, 4*LEGO_BLOCK_WIDTH, 0])

    lego_pick_location = [T_lego1_tray2_world, T_lego2_tray2_world, T_lego3_tray2_world, T_lego4_tray2_world, T_lego5_tray2_world]

    pick_offset_above_location = RigidTransform(translation=np.array([0.01,0.003,-0.05]), from_frame='lego', to_frame='lego')

    T_lego_pick_rotation_point_offset = RigidTransform(translation=np.array([0,-LEGO_BLOCK_LENGTH/2,LEGO_BLOCK_HEIGHT*4]), from_frame='lego', to_frame='lego')
    T_lego_pick_rotation = RigidTransform(rotation=RigidTransform.x_axis_rotation(np.deg2rad(-15)), translation=np.array([0,-LEGO_BLOCK_LENGTH/2,LEGO_BLOCK_HEIGHT*4]), from_frame='lego', to_frame='lego')


    T_lego3_tray1_world = RigidTransform.load(root_pwd+'/config/lego_tray1.tf')

    T_lego1_tray1_world = T_lego3_tray1_world.copy()
    T_lego1_tray1_world.translation += np.array([-LEGO_BLOCK_LENGTH * 3/4, -LEGO_BLOCK_LENGTH * 5/4, 0])
    T_lego2_tray1_world = T_lego3_tray1_world.copy()
    T_lego2_tray1_world.translation += np.array([0, -LEGO_BLOCK_LENGTH, 0])
    T_lego4_tray1_world = T_lego3_tray1_world.copy()
    T_lego4_tray1_world.translation += np.array([0, LEGO_BLOCK_LENGTH, 0])
    T_lego5_tray1_world = T_lego3_tray1_world.copy()
    T_lego5_tray1_world.translation += np.array([-LEGO_BLOCK_LENGTH * 3/4, LEGO_BLOCK_LENGTH * 5/4, 0])

    pos_ninety_deg_rotation = RigidTransform(rotation=RigidTransform.z_axis_rotation(np.deg2rad(90)), from_frame='lego', to_frame='lego')
    neg_ninety_deg_rotation = RigidTransform(rotation=RigidTransform.z_axis_rotation(np.deg2rad(-90)), from_frame='lego', to_frame='lego')

    T_lego1_tray1_world = T_lego1_tray1_world * neg_ninety_deg_rotation
    T_lego5_tray1_world = T_lego5_tray1_world * neg_ninety_deg_rotation

    lego_place_location = [T_lego1_tray1_world, T_lego2_tray1_world, T_lego3_tray1_world, T_lego4_tray1_world, T_lego5_tray1_world]

    place_offset_above_location = RigidTransform(translation=np.array([0.01,-0.003,-0.05]), from_frame='lego', to_frame='lego')

    T_lego_place_rotation_point_offset = RigidTransform(translation=np.array([0,-LEGO_BLOCK_LENGTH/2,0]), from_frame='lego', to_frame='lego')
    T_lego_place_rotation = RigidTransform(rotation=RigidTransform.x_axis_rotation(np.deg2rad(-15)), translation=np.array([0,-LEGO_BLOCK_LENGTH/2,0]), from_frame='lego', to_frame='lego')


    print("Returning to Home")
    while not robot_mg.go_home():
        pass

    for i in range(2):
        if i == 0:
            pick_locations = lego_pick_location
            current_pick_offset_above_location = pick_offset_above_location.copy()
            place_locations = lego_place_location
            current_place_offset_above_location = place_offset_above_location.copy()

        else:
            pick_locations = lego_place_location[::-1]
            current_pick_offset_above_location = place_offset_above_location.copy()
            place_locations = lego_pick_location[::-1]
            current_place_offset_above_location = pick_offset_above_location.copy()
        for j in range(5):
            current_pick_location = pick_locations[j]
            current_place_location = place_locations[j]

            ### Pick Up Routine

            pre_pick_location = current_pick_location * current_pick_offset_above_location
            T_ee_world_target = pre_pick_location * T_lego_ee.inverse()

            goal_pose = T_ee_world_target.pose_msg

            while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True, acc_scaling = 0.5, velocity_scaling=0.5):
                pass

            T_ee_world_target = current_pick_location * T_lego_ee.inverse()

            goal_pose = T_ee_world_target.pose_msg

            while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                pass

            rotation_point = current_pick_location * T_lego_pick_rotation
            
            T_ee_world_target = rotation_point * T_lego_pick_rotation_point_offset.inverse() * T_lego_ee.inverse()

            goal_pose = T_ee_world_target.pose_msg

            while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                pass

            T_ee_world_target = pre_pick_location * T_lego_ee.inverse()

            goal_pose = T_ee_world_target.pose_msg

            while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                pass

            ### Place Routine

            pre_place_location = current_place_location * current_place_offset_above_location
            T_ee_world_target = pre_place_location * T_lego_ee.inverse()

            goal_pose = T_ee_world_target.pose_msg

            while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True, acc_scaling = 0.5, velocity_scaling=0.5):
                pass

            T_ee_world_target = current_place_location * T_lego_ee.inverse()

            goal_pose = T_ee_world_target.pose_msg

            while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                pass

            rotation_point = current_place_location * T_lego_place_rotation
            
            T_ee_world_target = rotation_point * T_lego_place_rotation_point_offset.inverse() * T_lego_ee.inverse()

            goal_pose = T_ee_world_target.pose_msg

            while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                pass

            T_ee_world_target = pre_place_location * T_lego_ee.inverse()

            goal_pose = T_ee_world_target.pose_msg

            while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                pass

        print("Returning to Home")
        while not robot_mg.go_home():
            pass

    # T_lego1_tray2_world = RigidTransform.load(root_pwd+'/config/lego_tray2.tf')

    # T_lego1_tray2_world.translation -= np.array([LEGO_BLOCK_LENGTH * 5 / 4, 0, 0])

    # T_lego2_tray2_world = T_lego1_tray2_world.copy()
    # T_lego2_tray2_world.translation += np.array([LEGO_BLOCK_LENGTH * 3 / 2, LEGO_BLOCK_WIDTH, 0])

    # T_lego3_tray2_world = T_lego1_tray2_world.copy()
    # T_lego3_tray2_world.translation += np.array([0, 2*LEGO_BLOCK_WIDTH, 0])

    # T_lego4_tray2_world = T_lego1_tray2_world.copy()
    # T_lego4_tray2_world.translation += np.array([LEGO_BLOCK_LENGTH * 3 / 2, 3*LEGO_BLOCK_WIDTH,  0])

    # T_lego5_tray2_world = T_lego1_tray2_world.copy()
    # T_lego5_tray2_world.translation += np.array([0, 4*LEGO_BLOCK_WIDTH, 0])

    # lego_pick_location = [T_lego1_tray2_world, T_lego2_tray2_world, T_lego3_tray2_world, T_lego4_tray2_world, T_lego5_tray2_world]

    # T_lego_pick_rotation_point_offset = RigidTransform(translation=np.array([0,-LEGO_BLOCK_LENGTH/2,LEGO_BLOCK_HEIGHT*4]), from_frame='lego', to_frame='lego')

    # T_lego_pick_rotation = RigidTransform(rotation=RigidTransform.x_axis_rotation(np.deg2rad(-15)), translation=np.array([0,-LEGO_BLOCK_LENGTH/2,LEGO_BLOCK_HEIGHT*4]), from_frame='lego', to_frame='lego')

    # T_lego_place_rotation_point_offset = RigidTransform(translation=np.array([0,-LEGO_BLOCK_LENGTH/2,0]), from_frame='lego', to_frame='lego')

    # T_lego_place_rotation = RigidTransform(rotation=RigidTransform.x_axis_rotation(np.deg2rad(-15)), translation=np.array([0,-LEGO_BLOCK_LENGTH/2,0]), from_frame='lego', to_frame='lego')

    # print("Returning to Home")
    # while not robot_mg.go_home():
    #     pass

    # for location in lego_pick_location:

    #     pre_pickup_location = location.copy()
    #     pre_pickup_location.translation[0] += 0.003
    #     pre_pickup_location.translation[1] += 0.01
    #     pre_pickup_location.translation[2] += 0.05
    #     T_ee_world_target = pre_pickup_location * T_lego_ee.inverse()

    #     goal_pose = T_ee_world_target.pose_msg

    #     while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
    #         pass

    #     pick_location = location.copy()
    #     T_ee_world_target = pick_location * T_lego_ee.inverse()

    #     goal_pose = T_ee_world_target.pose_msg

    #     while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
    #         pass

    #     rotation_point = pick_location.copy() * T_lego_pick_rotation
        
    #     T_ee_world_target = rotation_point * T_lego_pick_rotation_point_offset.inverse() * T_lego_ee.inverse()

    #     goal_pose = T_ee_world_target.pose_msg

    #     while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
    #         pass

    #     T_ee_world_target = pre_pickup_location * T_lego_ee.inverse()

    #     goal_pose = T_ee_world_target.pose_msg

    #     while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
    #         pass

    #     place_location = location.copy()
    #     T_ee_world_target = place_location * T_lego_ee.inverse()

    #     goal_pose = T_ee_world_target.pose_msg

    #     while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
    #         pass

    #     rotation_point = place_location.copy() * T_lego_place_rotation
        
    #     T_ee_world_target = rotation_point * T_lego_place_rotation_point_offset.inverse() * T_lego_ee.inverse()

    #     goal_pose = T_ee_world_target.pose_msg

    #     while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
    #         pass

    #     T_ee_world_target = pre_pickup_location * T_lego_ee.inverse()

    #     goal_pose = T_ee_world_target.pose_msg

    #     while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
    #         pass

    # print("Returning to Home")
    # while not robot_mg.go_home():
    #     pass


if __name__ == "__main__":
    run()