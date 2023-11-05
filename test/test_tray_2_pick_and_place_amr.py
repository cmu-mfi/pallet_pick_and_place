#!/usr/bin/env python3

import rospy
import copy
import numpy as np

from geometry_msgs.msg import Pose, Twist

from pallet_pick_and_place.commander import RobotMoveGroup

from autolab_core import RigidTransform
import numpy as np
import rospy
import cv2
import sys
import cv2.aruco as aruco

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from autolab_core import Point, CameraIntrinsics, RigidTransform

def get_object_center_point_in_world(image_x, image_y, depth_image, intrinsics, transform):

    object_center = Point(np.array([image_x, image_y]), 'azure_kinect_overhead')
    object_depth = depth_image[image_y, image_x]

    return transform * intrinsics.deproject_pixel(object_depth, object_center)


AZURE_KINECT_INTRINSICS = 'azure_kinect_overhead.intr'
AZURE_KINECT_EXTRINSICS = 'azure_kinect_overhead_to_world.tf'

def run():

    rospy.init_node("pilot_tray2_amr_pickup")
    robot_mg = RobotMoveGroup()

    print("Returning to Home")
    while not robot_mg.go_home():
        pass

    T_hook_ee = RigidTransform.load('hook_ee.tf')

    T_amr_tray_world = RigidTransform.load('amr_tray.tf')

    bridge = CvBridge()
    intrinsics = CameraIntrinsics.load(AZURE_KINECT_INTRINSICS)
    azure_kinect_to_world_transform = RigidTransform.load(AZURE_KINECT_EXTRINSICS) 
    matrix_coefficients = np.array([1943.6480712890625, 0.0, 2045.5838623046875, 0.0, 1943.1328125, 1576.270751953125, 0.0, 0.0, 1.0])

    amr_angle = 90

    while np.abs(amr_angle) > 10:
        rgb_image_message = rospy.wait_for_message('/rgb/image_rect_color', Image)
        cv_image = bridge.imgmsg_to_cv2(rgb_image_message, desired_encoding='bgr8')
        depth_image_message = rospy.wait_for_message('/depth_to_rgb/hw_registered/image_rect', Image)
        depth_image = bridge.imgmsg_to_cv2(depth_image_message, desired_encoding='passthrough')

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  # Change grayscale
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)  # Use 5x5 dictionary to find markers
        parameters = aruco.DetectorParameters()  # Marker detection parameters
        detector = aruco.ArucoDetector(dictionary, parameters)
        corners, ids, rejected_img_points = detector.detectMarkers(gray)

        centers = []

        for corner in corners:
            centers.append(np.rint(np.mean(corner.reshape((4,2)), axis=0)).astype(int))

        center_points_in_world = {}
        amr_center = []
        center_idx = 0
        for center in centers:
            center_point_in_world = get_object_center_point_in_world(center[0], center[1], depth_image, intrinsics, azure_kinect_to_world_transform)
            #print([center_point_in_world.x, center_point_in_world.y, center_point_in_world.z])
            center_points_in_world[ids[center_idx][0]] = [center_point_in_world.x, center_point_in_world.y, center_point_in_world.z]
            center_idx += 1
            amr_center.append([center_point_in_world.x, center_point_in_world.y, center_point_in_world.z])

        #position_and_angle = 
        #print(center_points_in_world)
        true_amr_center = np.mean(np.array(amr_center),axis=0)
        print(true_amr_center)
        adjusted_point_58 = np.array(center_points_in_world[58]) - true_amr_center
        #print(adjusted_point_58)
        angle = np.arctan2(adjusted_point_58[1], adjusted_point_58[0]) - np.pi/4
        amr_angle = angle * 180 / np.pi
        print(amr_angle)

        amr_center_rigid_transform = RigidTransform(translation=true_amr_center, from_frame='amr', to_frame='world')
        amr_rotation = RigidTransform(rotation=RigidTransform.z_axis_rotation(angle), from_frame='amr', to_frame='amr')
        tray_amr_offset = RigidTransform.load('tray_amr.tf') #RigidTransform(translation=np.array([0.147, 0.14, 0.05]), from_frame='tray', to_frame='amr')

        new_T_amr_tray_world = amr_center_rigid_transform * amr_rotation * tray_amr_offset

    T_ee_world = T_amr_tray_world * T_hook_ee.inverse()


    T_amr_tray_world.rotation = (T_ee_world * RigidTransform(rotation = new_T_amr_tray_world.rotation, from_frame='ee', to_frame='ee') * T_hook_ee).rotation
    T_amr_tray_world.translation = new_T_amr_tray_world.translation

    print(T_amr_tray_world)

    target_joint = robot_mg.get_current_joints()
    target_joint[0] += np.pi/2

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


    T_tray_2_world = RigidTransform.load('tray_2.tf')

    T_hook_world_target_70 = T_tray_2_world.copy()

    T_hook_world_target_42 = T_tray_2_world * T_hook_rot_28

    T_hook_world_target_40 = T_tray_2_world * T_hook_rot_30

    T_hook_world_target_70.translation = T_tray_2_world.translation + np.array([-0.08,0,0.1])

    T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

    goal_pose = T_ee_world_target.pose_msg

    print("Rotating end effector to -70 degrees and moving -8cm in x and +10cm in z from tray location")

    while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
        pass

    T_hook_world_target_70.translation = T_tray_2_world.translation + np.array([-0.08,0,0])

    T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

    goal_pose = T_ee_world_target.pose_msg

    print("Moving down 10cm")

    while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
        pass

    T_hook_world_target_70.translation = T_tray_2_world.translation

    T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

    goal_pose = T_ee_world_target.pose_msg

    print("Moving forward 8cm")

    while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
        pass

    import pdb; pdb.set_trace()

    T_hook_world_target_42.translation = T_tray_2_world.translation

    T_ee_world_target = T_hook_world_target_42 * T_hook_ee.inverse()

    goal_pose = T_ee_world_target.pose_msg

    print("Rotating end-effector 28 degrees around x")

    while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
        pass

    T_hook_world_target_42.translation = T_tray_2_world.translation + np.array([-0.33, 0, 0])

    T_ee_world_target = T_hook_world_target_42 * T_hook_ee.inverse()

    goal_pose = T_ee_world_target.pose_msg

    print("Moving backward 33cm")

    while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
        pass

    T_hook_world_target_40.translation = T_tray_2_world.translation + np.array([-0.33, 0, 0.1])

    T_ee_world_target = T_hook_world_target_40 * T_hook_ee.inverse()
    
    goal_pose = T_ee_world_target.pose_msg

    print("Moving up 10cm while rotating 2 degrees around x")

    while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
        pass


    target_joint = robot_mg.get_current_joints()
    target_joint[0] -= np.pi * 3 / 2

    while not robot_mg.go_to_joint_state(target_joint, cartesian_path=False):
        pass

    T_hook_world_target_70 = T_amr_tray_world.copy()

    T_hook_world_target_42 = T_amr_tray_world * T_hook_rot_28

    T_hook_world_target_40 = T_amr_tray_world * T_hook_rot_30

    T_hook_world_target_40.translation = T_amr_tray_world.translation + np.array([0, 0, 0.1])

    T_ee_world_target = T_hook_world_target_40 * T_hook_ee.inverse()
    
    goal_pose = T_ee_world_target.pose_msg

    print("Moving to a location 10cm above the deposit location.")

    while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
        pass

    T_hook_world_target_42.translation = T_amr_tray_world.translation

    T_ee_world_target = T_hook_world_target_42 * T_hook_ee.inverse()

    goal_pose = T_ee_world_target.pose_msg

    print("Moving down to the tray deposit location while rotating the end-effector 2 degrees around x")

    while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
        pass


    T_hook_world_target_70.translation = T_amr_tray_world.translation

    T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

    goal_pose = T_ee_world_target.pose_msg

    print("Rotating the end-effector 28 degrees around x")

    while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
        pass

    T_hook_world_target_70.translation = T_amr_tray_world.translation + np.array([0,0.05,0])

    T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

    goal_pose = T_ee_world_target.pose_msg

    print("Moving to the left 5cm")

    while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
        pass


    T_hook_world_target_70.translation = T_amr_tray_world.translation + np.array([0,0.05,0.08])

    T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

    goal_pose = T_ee_world_target.pose_msg

    print("Moving the end effector up 8cm")

    while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
        pass

    print("Returning to Home")
    while not robot_mg.go_home():
        pass

if __name__ == "__main__":
    run()