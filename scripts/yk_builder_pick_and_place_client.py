#!/usr/bin/env python3

import requests
import time
import rospy
import copy
import numpy as np

from std_msgs.msg import Int16
from geometry_msgs.msg import Pose, Twist, Pose2D

from pallet_pick_and_place.commander import RobotMoveGroup

from autolab_core import RigidTransform
import numpy as np
import rospy
import cv2
import sys
import cv2.aruco as aruco
import os

LEGO_BLOCK_HEIGHT=0.0096 #z
LEGO_BLOCK_WIDTH=0.0158 #x
LEGO_BLOCK_LENGTH=0.0318 #y


def listener():
    url = 'http://localhost:5000/command'
    params = {'current': '1'}
    
    rospy.init_node("yk_builder_pick_and_place_client")
    namespace = rospy.get_param("yk_builder_pick_and_place_client/namespace")
    root_pwd = rospy.get_param("yk_builder_pick_and_place_client/root_pwd")

    print(namespace)
    print(root_pwd)
    robot_mg = RobotMoveGroup(namespace)
    rate = rospy.Rate(1)
    

    while not rospy.is_shutdown():
        response = requests.get(url, params=params)
        last_task = response.json()
        
        print("Last Task: ", last_task)
        

        try:
            if(last_task['name']=='yk_task' and last_task['status']=='START'):
                print("Move from worker station to yk station")
                requests.post(url, json={'name': 'yk_task', 'status': 'WIP'})
                """
                KEVIN CODE

                """

                toggle_pub = rospy.Publisher('/toggle', Int16, queue_size=1)

                ############### TRAY PICKUP CODE FROM AMR  ###############################################

                print("Returning to Home")
                while not robot_mg.go_home():
                    pass

                T_hook_ee = RigidTransform.load(root_pwd+'/config/hook_ee.tf')

                T_tray_world = RigidTransform.load(root_pwd+'/config/amr_tray.tf')

                amr_pose = rospy.wait_for_message('/amr_pose', Pose2D)

                amr_center = [amr_pose.x, amr_pose.y, 0.205]

                amr_center_rigid_transform = RigidTransform(translation=amr_center, from_frame='amr', to_frame='world')
                amr_rotation = RigidTransform(rotation=RigidTransform.z_axis_rotation(amr_pose.theta), from_frame='amr', to_frame='amr')
                tray_amr_offset = RigidTransform.load(root_pwd+'/config/tray_amr.tf')

                print(amr_center_rigid_transform)

                new_T_tray_world = amr_center_rigid_transform * amr_rotation * tray_amr_offset

                print(new_T_tray_world)

                T_ee_world = RigidTransform(rotation = RigidTransform.z_axis_rotation(amr_pose.theta), from_frame='world', to_frame='world') * T_tray_world * T_hook_ee.inverse()

                T_tray_world.rotation = T_ee_world.rotation
                T_tray_world.translation = new_T_tray_world.translation

                print(T_tray_world)

                target_joint = robot_mg.get_current_joints()
                target_joint[0] -= np.pi*3/4

                while not robot_mg.go_to_joint_state(target_joint, cartesian_path=False):
                    pass


                T_hook_rot_28 = RigidTransform(
                    rotation=RigidTransform.x_axis_rotation(np.deg2rad(28)),
                    from_frame='hook', to_frame='hook'
                )

                T_hook_rot_30 = RigidTransform(
                    rotation=RigidTransform.x_axis_rotation(np.deg2rad(30)),
                    from_frame='hook', to_frame='hook'
                )

                T_hook_world_target_70 = T_tray_world.copy()

                T_hook_world_target_42 = T_tray_world * T_hook_rot_28

                T_hook_world_target_70.translation = T_tray_world.translation + np.array([0,0.04,0.07])

                T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

                goal_pose = T_ee_world_target.pose_msg

                print("Rotating end effector to -70 degrees and moving -4cm in x and +7cm in z from tray location")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass

                T_hook_world_target_70.translation = T_tray_world.translation + np.array([0,0.04,0])

                T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

                goal_pose = T_ee_world_target.pose_msg

                print("Moving down 4cm")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass

                T_hook_world_target_70.translation = T_tray_world.translation

                T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

                goal_pose = T_ee_world_target.pose_msg

                print("Moving forward 8cm")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass

                #import pdb; pdb.set_trace()

                T_hook_world_target_42.translation = T_tray_world.translation

                T_ee_world_target = T_hook_world_target_42 * T_hook_ee.inverse()

                goal_pose = T_ee_world_target.pose_msg

                print("Rotating end-effector 28 degrees around x")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass

                T_hook_world_target_40 = T_tray_world * T_hook_rot_30

                T_ee_world_target = RigidTransform.from_pose_msg(robot_mg.get_current_pose(), from_frame='ee')

                T_ee_world_target.translation[2] += 0.1

                T_hook_world_target_40.translation = T_ee_world_target.translation
                
                goal_pose = T_hook_world_target_40.pose_msg

                print("Moving up 10cm while rotating 2 degrees around x")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass

                T_intermediate_hook_world = RigidTransform.load(root_pwd+'/config/intermediate_hook_pose.tf')

                T_ee_world_target = T_intermediate_hook_world * T_hook_ee.inverse()

                goal_pose = T_ee_world_target.pose_msg

                print("Moving to intermediate_hook_pose")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass


                target_joint = robot_mg.get_current_joints()
                target_joint[0] += np.pi
                #target_joint[5] -= np.pi

                while not robot_mg.go_to_joint_state(target_joint, cartesian_path=False):
                    pass

                T_hook_ee = RigidTransform.load(root_pwd+'/config/hook_ee_2.tf')

                T_tray_world = RigidTransform.load(root_pwd+'/config/tray_2.tf')


                T_hook_rot_28 = RigidTransform(
                    rotation=RigidTransform.x_axis_rotation(np.deg2rad(28)),
                    from_frame='hook', to_frame='hook'
                )

                T_hook_rot_30 = RigidTransform(
                    rotation=RigidTransform.x_axis_rotation(np.deg2rad(30)),
                    from_frame='hook', to_frame='hook'
                )

                T_hook_world_target_70 = T_tray_world.copy()

                T_hook_world_target_42 = T_tray_world * T_hook_rot_28

                T_hook_world_target_40 = T_tray_world * T_hook_rot_30

                T_hook_world_target_40.translation = T_tray_world.translation + np.array([-0.33,0,0.1])

                T_ee_world_target = T_hook_world_target_40 * T_hook_ee.inverse()

                goal_pose = T_ee_world_target.pose_msg

                print("Moving -33cm in x and +10cm in z from tray location")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass

                T_hook_world_target_42.translation = T_tray_world.translation + np.array([-0.33,0,0])

                T_ee_world_target = T_hook_world_target_42 * T_hook_ee.inverse()

                goal_pose = T_ee_world_target.pose_msg

                print("Moving down 10cm and rotating 2 degrees")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass

                T_hook_world_target_42.translation = T_tray_world.translation

                T_ee_world_target = T_hook_world_target_42 * T_hook_ee.inverse()

                goal_pose = T_ee_world_target.pose_msg

                print("Moving forward 33cm")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass


                T_hook_world_target_70.translation = T_tray_world.translation

                T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

                goal_pose = T_ee_world_target.pose_msg

                print("Rotating end-effector 28 degrees around x")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass


                T_hook_world_target_70.translation = T_tray_world.translation + np.array([-0.08,0,0])

                T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

                goal_pose = T_ee_world_target.pose_msg

                print("Moving backward 8cm")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass

                T_hook_world_target_70.translation = T_tray_world.translation + np.array([-0.08,0,0.1])
                
                T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

                goal_pose = T_ee_world_target.pose_msg

                print("Moving up 10cm")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass

                print("Returning to Home")
                while not robot_mg.go_home():
                    pass

                lin_act_msg = Int16()
                lin_act_msg.data = 2
                toggle_pub.publish(lin_act_msg)


                ############### LEGO BUILDING CODE  ###############################################

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
                ################################### TRAY RETURN #######################################

                toggle_pub.publish(lin_act_msg)

                T_hook_ee = RigidTransform.load(root_pwd+'/config/hook_ee_2.tf')

                T_tray_world = RigidTransform.load(root_pwd+'/config/amr_tray.tf')

                amr_pose = rospy.wait_for_message('/amr_pose', Pose2D)

                amr_center = [amr_pose.x, amr_pose.y, 0.205]

                amr_center_rigid_transform = RigidTransform(translation=amr_center, from_frame='amr', to_frame='world')
                amr_rotation = RigidTransform(rotation=RigidTransform.z_axis_rotation(amr_pose.theta), from_frame='amr', to_frame='amr')
                tray_amr_offset = RigidTransform.load(root_pwd+'/config/tray_amr.tf')

                new_T_tray_world = amr_center_rigid_transform * amr_rotation * tray_amr_offset

                T_ee_world = RigidTransform(rotation = RigidTransform.z_axis_rotation(amr_pose.theta), from_frame='world', to_frame='world') * T_tray_world * T_hook_ee.inverse()

                T_tray_world.rotation = T_ee_world.rotation
                T_tray_world.translation = new_T_tray_world.translation

                print(T_tray_world)

                T_amr_tray_world = T_tray_world.copy()

                target_joint = robot_mg.get_current_joints()
                target_joint[0] += np.pi*3/4

                while not robot_mg.go_to_joint_state(target_joint, cartesian_path=False):
                    pass


                T_tray_world = RigidTransform.load(root_pwd+'/config/tray_2.tf')

                T_hook_rot_28 = RigidTransform(
                    rotation=RigidTransform.x_axis_rotation(np.deg2rad(28)),
                    from_frame='hook', to_frame='hook'
                )

                T_hook_rot_30 = RigidTransform(
                    rotation=RigidTransform.x_axis_rotation(np.deg2rad(30)),
                    from_frame='hook', to_frame='hook'
                )

                T_hook_world_target_70 = T_tray_world.copy()

                T_hook_world_target_42 = T_tray_world * T_hook_rot_28

                T_hook_world_target_40 = T_tray_world * T_hook_rot_30

                T_hook_world_target_70.translation = T_tray_world.translation + np.array([-0.08,0,0.1])
                
                T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

                goal_pose = T_ee_world_target.pose_msg

                print("Rotating end effector to -70 degrees and moving -8cm in x and +10cm in z from tray pick up location")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass


                T_hook_world_target_70.translation = T_tray_world.translation + np.array([-0.08,0,0])

                T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

                goal_pose = T_ee_world_target.pose_msg

                print("Moving down 10cm")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass

                T_hook_world_target_70.translation = T_tray_world.translation

                T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

                goal_pose = T_ee_world_target.pose_msg

                print("Moving forward 8cm")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass

                T_hook_world_target_42.translation = T_tray_world.translation

                T_ee_world_target = T_hook_world_target_42 * T_hook_ee.inverse()

                goal_pose = T_ee_world_target.pose_msg

                print("Rotating end-effector 28 degrees around x")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass


                T_hook_world_target_42.translation = T_tray_world.translation + np.array([-0.33,0,0])

                T_ee_world_target = T_hook_world_target_42 * T_hook_ee.inverse()

                goal_pose = T_ee_world_target.pose_msg

                print("Moving backward 33cm")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass


                T_hook_world_target_40.translation = T_tray_world.translation + np.array([-0.33,0,0.1])

                T_ee_world_target = T_hook_world_target_40 * T_hook_ee.inverse()

                goal_pose = T_ee_world_target.pose_msg

                print("Moving up 10cm while rotating 2 degrees around x")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass


                target_joint = robot_mg.get_current_joints()
                target_joint[0] -= np.pi

                while not robot_mg.go_to_joint_state(target_joint, cartesian_path=False):
                    pass

                T_hook_ee = RigidTransform.load(root_pwd+'/config/hook_ee.tf')

                T_tray_world = T_amr_tray_world.copy()

                T_hook_rot_28 = RigidTransform(
                    rotation=RigidTransform.x_axis_rotation(np.deg2rad(28)),
                    from_frame='hook', to_frame='hook'
                )

                T_hook_rot_30 = RigidTransform(
                    rotation=RigidTransform.x_axis_rotation(np.deg2rad(30)),
                    from_frame='hook', to_frame='hook'
                )

                T_hook_world_target_70 = T_tray_world.copy()

                T_hook_world_target_42 = T_tray_world * T_hook_rot_28

                T_hook_world_target_40 = T_tray_world * T_hook_rot_30


                T_hook_world_target_40.translation = T_tray_world.translation + np.array([0, 0, 0.1])

                T_ee_world_target = T_hook_world_target_40 * T_hook_ee.inverse()
                
                goal_pose = T_ee_world_target.pose_msg

                print("Moving to 10cm above the amr.")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass

                T_hook_world_target_42.translation = T_tray_world.translation

                T_ee_world_target = T_hook_world_target_42 * T_hook_ee.inverse()

                goal_pose = T_ee_world_target.pose_msg

                print("Moving down 10cm while rotating 2cm to deposit the tray into the slot.")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass

                T_hook_world_target_70.translation = T_tray_world.translation

                T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

                goal_pose = T_ee_world_target.pose_msg

                print("Rotating 28 degrees around x")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass


                T_hook_world_target_70.translation = T_tray_world.translation + np.array([0.015,0,0])

                T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

                goal_pose = T_ee_world_target.pose_msg

                print("Move Right 1.5cm to jiggle the tray into the slot.")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass

                T_hook_world_target_70.translation = T_tray_world.translation + np.array([-0.015,0,0])

                T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

                goal_pose = T_ee_world_target.pose_msg

                print("Move Left 1.5cm to jiggle the tray into the slot.")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass

                T_hook_world_target_70.translation = T_tray_world.translation

                T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

                goal_pose = T_ee_world_target.pose_msg

                print("Move back to the center.")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass

                T_hook_world_target_70.translation = T_tray_world.translation + np.array([0,0.05,0])

                T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

                goal_pose = T_ee_world_target.pose_msg

                print("Moving back 5cm")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass


                T_hook_world_target_70.translation = T_tray_world.translation + np.array([0,0.05,0.08])

                T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

                goal_pose = T_ee_world_target.pose_msg

                print("Moving up 8cm")

                while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
                    pass
                

                print("Returning to Home")
                while not robot_mg.go_home():
                    pass


                requests.post(url, json={'name': 'yk_task', 'status': 'FINISHED'})
        except Exception as error:
            # handle the exception
            print("An exception occurred:", error)
            
        # wait 5 seconds
        rate.sleep()
    
if __name__ == '__main__': 
    listener()
