#!/usr/bin/env python3

import requests
import time
import rospy
import copy
import numpy as np

from std_msgs.msg import Int16
from geometry_msgs.msg import Pose, Twist, Pose2D

from pallet_pick_and_place_package.commander import RobotMoveGroup

from autolab_core import RigidTransform
import numpy as np
import rospy
import sys
import os

from pallet_pick_and_place.srv import PalletPickAndPlace,PalletPickAndPlaceResponse

class PalletPickAndPlaceServer:

    def __init__(self, namespace, root_pwd):
        self.namespace = namespace
        self.root_pwd = root_pwd
        self.robot_mg = RobotMoveGroup(namespace)
        self.service = rospy.Service('/'+self.namespace+'/pallet_pick_and_place', PalletPickAndPlace, self.callback)
        self.toggle_pub = rospy.Publisher('/toggle', Int16, queue_size=1)
        if namespace == 'yk_builder':
            self.toggle_offset = 0
        elif namespace == 'yk_creator':
            self.toggle_offset = 2 
        self.T_hook_ee = RigidTransform.load(root_pwd+'/config/hook_ee.tf')
        self.T_tray_amr = RigidTransform.load(root_pwd+'/config/tray_amr.tf')
        self.T_hook_world_amr = RigidTransform.load(root_pwd+'/config/hook_world_amr.tf')
        self.amr_x_limits = [-0.55, -0.5]
        self.amr_y_limits = [-0.33, -0.25]
        self.T_hook_world_tray1 = RigidTransform.load(root_pwd+'/config/hook_world_tray'+str(1+self.toggle_offset)+'.tf')
        self.T_hook_world_tray2 = RigidTransform.load(root_pwd+'/config/hook_world_tray'+str(2+self.toggle_offset)+'.tf')
        self.T_hook_world_intermediate_pos1 = RigidTransform.load(root_pwd+'/config/hook_world_intermediate_pos1.tf')
        self.T_hook_world_intermediate_pos2 = RigidTransform.load(root_pwd+'/config/hook_world_intermediate_pos2.tf')
        self.T_hook_rot_28 = RigidTransform(
                    rotation=RigidTransform.x_axis_rotation(np.deg2rad(28)),
                    from_frame='hook', to_frame='hook'
                )
        self.T_hook_rot_30 = RigidTransform(
                    rotation=RigidTransform.x_axis_rotation(np.deg2rad(30)),
                    from_frame='hook', to_frame='hook'
                )

    def callback(self, req):
        try:
            print("Returning to Home")
            while not self.robot_mg.go_home():
                pass

            amr_pose = rospy.wait_for_message('/'+self.namespace+'/amr_pose', Pose2D)

            amr_center = [amr_pose.x, amr_pose.y, 0.205]

            amr_center_rigid_transform = RigidTransform(translation=amr_center, from_frame='amr', to_frame='world')
            amr_rotation = RigidTransform(rotation=RigidTransform.z_axis_rotation(amr_pose.theta), from_frame='amr', to_frame='amr')

            print(amr_center_rigid_transform)

            new_T_tray_world = amr_center_rigid_transform * amr_rotation * self.T_tray_amr

            print(new_T_tray_world)

            T_ee_world = RigidTransform(rotation = RigidTransform.z_axis_rotation(amr_pose.theta), from_frame='world', to_frame='world') * self.T_hook_world_amr * self.T_hook_ee.inverse()

            T_hook_world_amr_pickup = RigidTransform(rotation = T_ee_world.rotation, translation = new_T_tray_world.translation, from_frame='hook', to_frame='world')

            print(T_hook_world_amr_pickup)

            if amr_center[0] < self.amr_x_limits[0] or amr_center[0] > self.amr_x_limits[1] or amr_center[1] < self.amr_y_limits[0] or amr_center[1] > self.amr_y_limits[1]:
                print("AMR is not docked within a suitable picking or placing location. Please have the AMR redock.")
                return PalletPickAndPlaceResponse(False)
            else:
                if req.load:
                    pass
                else:
                    ########################## PICKING UP TRAY FROM AMR ##########################
                    target_joint = self.robot_mg.get_current_joints()
                    target_joint[0] -= np.pi*3/4

                    while not self.robot_mg.go_to_joint_state(target_joint, cartesian_path=False):
                        pass

                    T_hook_world_amr_pos1 = T_hook_world_amr_pickup.copy()

                    T_hook_world_amr_pos1.translation = T_hook_world_amr_pickup.translation + np.array([0,0.04,0.07])

                    T_ee_world_amr_pos1 = T_hook_world_amr_pos1 * self.T_hook_ee.inverse()
                    
                    amr_pos1_goal_pose = T_ee_world_amr_pos1.pose_msg

                    print("Rotating end effector to -70 degrees and moving -4cm in x and +7cm in z from tray pick up location")
                    while not self.robot_mg.go_to_pose_goal(amr_pos1_goal_pose, cartesian_path=True):
                        pass

                    T_hook_world_amr_pos2 = T_hook_world_amr_pickup.copy()

                    T_hook_world_amr_pos2.translation = T_hook_world_amr_pickup.translation + np.array([0,0.04,0])

                    T_ee_world_amr_pos2 = T_hook_world_amr_pos2 * self.T_hook_ee.inverse()

                    amr_pos2_goal_pose = T_ee_world_amr_pos2.pose_msg

                    print("Moving down 4cm")
                    while not self.robot_mg.go_to_pose_goal(amr_pos2_goal_pose, cartesian_path=True):
                        pass

                    T_hook_world_amr_pos3 = T_hook_world_amr_pickup.copy()

                    T_ee_world_amr_pos3 = T_hook_world_amr_pos3 * self.T_hook_ee.inverse()

                    amr_pos3_goal_pose = T_ee_world_amr_pos3.pose_msg

                    print("Moving forward 8cm")
                    while not self.robot_mg.go_to_pose_goal(amr_pos3_goal_pose, cartesian_path=True):
                        pass

                    #import pdb; pdb.set_trace();

                    T_hook_world_amr_pos4 = T_hook_world_amr_pickup * self.T_hook_rot_28

                    T_ee_world_amr_pos4 = T_hook_world_amr_pos4 * self.T_hook_ee.inverse()

                    amr_pos4_goal_pose = T_ee_world_amr_pos4.pose_msg

                    print("Rotating end-effector 28 degrees around x")

                    while not self.robot_mg.go_to_pose_goal(amr_pos4_goal_pose, cartesian_path=True):
                        pass

                    T_hook_world_amr_pos5 = T_hook_world_amr_pickup * self.T_hook_rot_30

                    T_hook_world_amr_pos5.translation[2] += 0.1
                    
                    T_ee_world_amr_pos5 = T_hook_world_amr_pos5 * self.T_hook_ee.inverse()

                    amr_pos5_goal_pose = T_ee_world_amr_pos5.pose_msg

                    print("Moving up 10cm while rotating 2 degrees around x")

                    while not self.robot_mg.go_to_pose_goal(amr_pos5_goal_pose, cartesian_path=True):
                        pass

                    T_ee_world_intermediate_pos1 = self.T_hook_world_intermediate_pos1 * self.T_hook_ee.inverse()

                    intermediate_pos1_goal_pose = T_ee_world_intermediate_pos1.pose_msg

                    print("Moving to intermediate hook pose 1")

                    while not self.robot_mg.go_to_pose_goal(intermediate_pos1_goal_pose, cartesian_path=True):
                        pass

                    
                    ########################## PLACING TRAY ON TABLE ##########################

                    if req.location == 1:
                        pass
                    elif req.location == 2: 

                        T_ee_world_intermediate_pos2 = self.T_hook_world_intermediate_pos2 * self.T_hook_ee.inverse()

                        intermediate_pos2_goal_pose = T_ee_world_intermediate_pos2.pose_msg
                        print("Moving to intermediate hook pose 2")

                        while not self.robot_mg.go_to_pose_goal(intermediate_pos2_goal_pose, cartesian_path=True):
                            pass

                        target_joint = self.robot_mg.get_current_joints()
                        target_joint[0] += np.pi*5/4

                        while not self.robot_mg.go_to_joint_state(target_joint, cartesian_path=False):
                            pass

                        T_hook_world_tray2_pos1 = self.T_hook_world_tray2 * self.T_hook_rot_30
                        T_hook_world_tray2_pos1.translation = T_hook_world_tray2_pos1.translation + np.array([-0.33,0,0.2])

                        T_ee_world_tray2_pos1 = T_hook_world_tray2_pos1 * self.T_hook_ee.inverse()

                        tray2_pos1_goal_pose = T_ee_world_tray2_pos1.pose_msg

                        print("Moving -33cm in x and +20cm in z from tray location")

                        while not self.robot_mg.go_to_pose_goal(tray2_pos1_goal_pose, cartesian_path=True):
                            pass

                        T_hook_world_tray2_pos2 = self.T_hook_world_tray2 * self.T_hook_rot_28
                        T_hook_world_tray2_pos2.translation = T_hook_world_tray2_pos2.translation + np.array([-0.33,0,0])

                        T_ee_world_tray2_pos2 = T_hook_world_tray2_pos2 * self.T_hook_ee.inverse()

                        tray2_pos2_goal_pose = T_ee_world_tray2_pos2.pose_msg

                        print("Moving down 20cm and rotating 2 degrees")

                        while not self.robot_mg.go_to_pose_goal(tray2_pos2_goal_pose, cartesian_path=True):
                            pass

                        # import pdb; pdb.set_trace();

                        T_hook_world_tray2_pos3 = self.T_hook_world_tray2 * self.T_hook_rot_28

                        T_ee_world_tray2_pos3 = T_hook_world_tray2_pos3 * self.T_hook_ee.inverse()

                        tray2_pos3_goal_pose = T_ee_world_tray2_pos3.pose_msg

                        print("Moving forward 33cm")

                        while not self.robot_mg.go_to_pose_goal(tray2_pos3_goal_pose, cartesian_path=True):
                            pass


                        T_hook_world_tray2_pos4 = self.T_hook_world_tray2.copy()

                        T_ee_world_tray2_pos4 = T_hook_world_tray2_pos4 * self.T_hook_ee.inverse()

                        tray2_pos4_goal_pose = T_ee_world_tray2_pos4.pose_msg

                        print("Rotating end-effector 28 degrees around x")

                        while not self.robot_mg.go_to_pose_goal(tray2_pos4_goal_pose, cartesian_path=True):
                            pass

                        T_hook_world_tray2_pos5 = self.T_hook_world_tray2.copy()

                        T_hook_world_tray2_pos5.translation = T_hook_world_tray2_pos5.translation + np.array([-0.08,0,0])
                        T_ee_world_tray2_pos5 = T_hook_world_tray2_pos5 * self.T_hook_ee.inverse()

                        tray2_pos5_goal_pose = T_ee_world_tray2_pos5.pose_msg

                        print("Moving backward 8cm")

                        while not self.robot_mg.go_to_pose_goal(tray2_pos5_goal_pose, cartesian_path=True):
                            pass

                        T_hook_world_tray2_pos6 = self.T_hook_world_tray2.copy()

                        T_hook_world_tray2_pos6.translation = T_hook_world_tray2_pos6.translation + np.array([-0.08,0,0.1])
                        T_ee_world_tray2_pos6 = T_hook_world_tray2_pos6 * self.T_hook_ee.inverse()

                        tray2_pos6_goal_pose = T_ee_world_tray2_pos6.pose_msg

                        print("Moving up 10cm")

                        while not self.robot_mg.go_to_pose_goal(tray2_pos6_goal_pose, cartesian_path=True):
                            pass

                        print("Returning to Home")
                        while not robot_mg.go_home():
                            pass

                        lin_act_msg = Int16()
                        lin_act_msg.data = 2 + self.toggle_offset
                        toggle_pub.publish(lin_act_msg)

            return PalletPickAndPlaceResponse(True)
        except Exception as error:
            print("An exception occurred:", error)
            return PalletPickAndPlaceResponse(False)

def main(args):
    
    rospy.init_node('pallet_pick_and_place_server', anonymous=True)
    namespace = rospy.get_param("pallet_pick_and_place_server/namespace")
    print(namespace)
    root_pwd = rospy.get_param("pallet_pick_and_place_server/root_pwd")
    print(root_pwd)
    PPAP = PalletPickAndPlaceServer(namespace, root_pwd)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)


        

#                 toggle_pub.publish(lin_act_msg)

#                 T_hook_ee = RigidTransform.load(root_pwd+'/config/hook_ee_2.tf')

#                 T_tray_world = RigidTransform.load(root_pwd+'/config/amr_tray.tf')

#                 amr_pose = rospy.wait_for_message('/amr_pose', Pose2D)

#                 amr_center = [amr_pose.x, amr_pose.y, 0.205]

#                 amr_center_rigid_transform = RigidTransform(translation=amr_center, from_frame='amr', to_frame='world')
#                 amr_rotation = RigidTransform(rotation=RigidTransform.z_axis_rotation(amr_pose.theta), from_frame='amr', to_frame='amr')
#                 tray_amr_offset = RigidTransform.load(root_pwd+'/config/tray_amr.tf')

#                 new_T_tray_world = amr_center_rigid_transform * amr_rotation * tray_amr_offset

#                 T_ee_world = RigidTransform(rotation = RigidTransform.z_axis_rotation(amr_pose.theta), from_frame='world', to_frame='world') * T_tray_world * T_hook_ee.inverse()

#                 T_tray_world.rotation = T_ee_world.rotation
#                 T_tray_world.translation = new_T_tray_world.translation

#                 print(T_tray_world)

#                 T_amr_tray_world = T_tray_world.copy()

#                 target_joint = robot_mg.get_current_joints()
#                 target_joint[0] += np.pi*3/4

#                 while not robot_mg.go_to_joint_state(target_joint, cartesian_path=False):
#                     pass


#                 T_tray_world = RigidTransform.load(root_pwd+'/config/tray_2.tf')

#                 T_hook_rot_28 = RigidTransform(
#                     rotation=RigidTransform.x_axis_rotation(np.deg2rad(28)),
#                     from_frame='hook', to_frame='hook'
#                 )

#                 T_hook_rot_30 = RigidTransform(
#                     rotation=RigidTransform.x_axis_rotation(np.deg2rad(30)),
#                     from_frame='hook', to_frame='hook'
#                 )

#                 T_hook_world_target_70 = T_tray_world.copy()

#                 T_hook_world_target_42 = T_tray_world * T_hook_rot_28

#                 T_hook_world_target_40 = T_tray_world * T_hook_rot_30

#                 T_hook_world_target_70.translation = T_tray_world.translation + np.array([-0.08,0,0.1])
                
#                 T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

#                 goal_pose = T_ee_world_target.pose_msg

#                 print("Rotating end effector to -70 degrees and moving -8cm in x and +10cm in z from tray pick up location")

#                 while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
#                     pass


#                 T_hook_world_target_70.translation = T_tray_world.translation + np.array([-0.08,0,0])

#                 T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

#                 goal_pose = T_ee_world_target.pose_msg

#                 print("Moving down 10cm")

#                 while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
#                     pass

#                 T_hook_world_target_70.translation = T_tray_world.translation

#                 T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

#                 goal_pose = T_ee_world_target.pose_msg

#                 print("Moving forward 8cm")

#                 while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
#                     pass

#                 T_hook_world_target_42.translation = T_tray_world.translation

#                 T_ee_world_target = T_hook_world_target_42 * T_hook_ee.inverse()

#                 goal_pose = T_ee_world_target.pose_msg

#                 print("Rotating end-effector 28 degrees around x")

#                 while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
#                     pass


#                 T_hook_world_target_42.translation = T_tray_world.translation + np.array([-0.33,0,0])

#                 T_ee_world_target = T_hook_world_target_42 * T_hook_ee.inverse()

#                 goal_pose = T_ee_world_target.pose_msg

#                 print("Moving backward 33cm")

#                 while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
#                     pass


#                 T_hook_world_target_40.translation = T_tray_world.translation + np.array([-0.33,0,0.1])

#                 T_ee_world_target = T_hook_world_target_40 * T_hook_ee.inverse()

#                 goal_pose = T_ee_world_target.pose_msg

#                 print("Moving up 10cm while rotating 2 degrees around x")

#                 while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
#                     pass


#                 target_joint = robot_mg.get_current_joints()
#                 target_joint[0] -= np.pi

#                 while not robot_mg.go_to_joint_state(target_joint, cartesian_path=False):
#                     pass

#                 T_hook_ee = RigidTransform.load(root_pwd+'/config/hook_ee.tf')

#                 T_tray_world = T_amr_tray_world.copy()

#                 T_hook_rot_28 = RigidTransform(
#                     rotation=RigidTransform.x_axis_rotation(np.deg2rad(28)),
#                     from_frame='hook', to_frame='hook'
#                 )

#                 T_hook_rot_30 = RigidTransform(
#                     rotation=RigidTransform.x_axis_rotation(np.deg2rad(30)),
#                     from_frame='hook', to_frame='hook'
#                 )

#                 T_hook_world_target_70 = T_tray_world.copy()

#                 T_hook_world_target_42 = T_tray_world * T_hook_rot_28

#                 T_hook_world_target_40 = T_tray_world * T_hook_rot_30


#                 T_hook_world_target_40.translation = T_tray_world.translation + np.array([0, 0, 0.1])

#                 T_ee_world_target = T_hook_world_target_40 * T_hook_ee.inverse()
                
#                 goal_pose = T_ee_world_target.pose_msg

#                 print("Moving to 10cm above the amr.")

#                 while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
#                     pass

#                 T_hook_world_target_42.translation = T_tray_world.translation

#                 T_ee_world_target = T_hook_world_target_42 * T_hook_ee.inverse()

#                 goal_pose = T_ee_world_target.pose_msg

#                 print("Moving down 10cm while rotating 2cm to deposit the tray into the slot.")

#                 while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
#                     pass

#                 T_hook_world_target_70.translation = T_tray_world.translation

#                 T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

#                 goal_pose = T_ee_world_target.pose_msg

#                 print("Rotating 28 degrees around x")

#                 while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
#                     pass


#                 T_hook_world_target_70.translation = T_tray_world.translation + np.array([0.015,0,0])

#                 T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

#                 goal_pose = T_ee_world_target.pose_msg

#                 print("Move Right 1.5cm to jiggle the tray into the slot.")

#                 while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
#                     pass

#                 T_hook_world_target_70.translation = T_tray_world.translation + np.array([-0.015,0,0])

#                 T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

#                 goal_pose = T_ee_world_target.pose_msg

#                 print("Move Left 1.5cm to jiggle the tray into the slot.")

#                 while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
#                     pass

#                 T_hook_world_target_70.translation = T_tray_world.translation

#                 T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

#                 goal_pose = T_ee_world_target.pose_msg

#                 print("Move back to the center.")

#                 while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
#                     pass

#                 T_hook_world_target_70.translation = T_tray_world.translation + np.array([0,0.05,0])

#                 T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

#                 goal_pose = T_ee_world_target.pose_msg

#                 print("Moving back 5cm")

#                 while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
#                     pass


#                 T_hook_world_target_70.translation = T_tray_world.translation + np.array([0,0.05,0.08])

#                 T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

#                 goal_pose = T_ee_world_target.pose_msg

#                 print("Moving up 8cm")

#                 while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
#                     pass
                

#                 print("Returning to Home")
#                 while not robot_mg.go_home():
#                     pass

#         except Exception as error:
#             # handle the exception
#             print("An exception occurred:", error)
            
#         # wait 5 seconds
#         rate.sleep()
    
# if __name__ == '__main__': 
#     listener()
