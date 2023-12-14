#!/usr/bin/env python3

import rospy
import copy

from geometry_msgs.msg import Pose, Twist

from pallet_pick_and_place.commander import RobotMoveGroup

import numpy as np
import cv2
import sys
import cv2.aruco as aruco

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from autolab_core import Point, CameraIntrinsics, RigidTransform

def get_object_center_point_in_world(image_x, image_y, depth_image, intrinsics, transform):

    object_center = Point(np.array([image_x, image_y]), 'azure_kinect_overhead')
    object_depth = np.mean(depth_image[image_y-1:image_y+1, image_x-1:image_x+1])
    
    return transform * intrinsics.deproject_pixel(object_depth, object_center)

AZURE_KINECT_INTRINSICS = 'azure_kinect_overhead.intr'
AZURE_KINECT_EXTRINSICS = 'azure_kinect_overhead_to_world.tf'

def run():

    rospy.init_node("test_amr_pick_and_place_tray2")

    namespace = rospy.get_param("test_amr_pick_and_place_tray2/namespace")
    root_pwd = rospy.get_param("test_amr_pick_and_place_tray2/root_pwd")

    robot_mg = RobotMoveGroup(namespace)
    
    print("Returning to Home")
    while not robot_mg.go_home():
        pass

    T_hook_ee = RigidTransform.load(root_pwd+'/config/hook_ee.tf')

    T_tray_world = RigidTransform.load(root_pwd+'/config/amr_tray.tf')

    bridge = CvBridge()

    intrinsics = CameraIntrinsics.load(root_pwd+'/config/'+AZURE_KINECT_INTRINSICS)
    azure_kinect_to_world_transform = RigidTransform.load(root_pwd+'/config/'+AZURE_KINECT_EXTRINSICS) 
    matrix_coefficients = np.array([1943.6480712890625, 0.0, 2045.5838623046875, 0.0, 1943.1328125, 1576.270751953125, 0.0, 0.0, 1.0])

    amr_angle = 90

    true_amr_centers = []
    angles = []

    while np.abs(amr_angle) > 5 or len(true_amr_centers) < 5:
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

        true_amr_center = np.mean(np.array(amr_center),axis=0)
        print(true_amr_center)
        adjusted_point_58 = np.array(center_points_in_world[58]) - true_amr_center
        #print(adjusted_point_58)
        angle = np.arctan2(adjusted_point_58[1], adjusted_point_58[0]) - np.pi/4
        amr_angle = angle * 180 / np.pi
        print(amr_angle)
        if true_amr_center[2] < 0.2 or np.abs(amr_angle) > 5 or np.any(np.isnan(true_amr_center)):
            amr_angle = 90
        else:
            true_amr_centers.append(true_amr_center)
            angles.append(angle)

    print(true_amr_centers)
    print(angles)

    avg_amr_center = np.mean(np.array(true_amr_centers),axis=0)
    avg_amr_angle = np.mean(angles)

    amr_center_rigid_transform = RigidTransform(translation=avg_amr_center, from_frame='amr', to_frame='world')
    amr_rotation = RigidTransform(rotation=RigidTransform.z_axis_rotation(-avg_amr_angle), from_frame='amr', to_frame='amr')
    tray_amr_offset = RigidTransform.load(root_pwd+'/config/tray_amr.tf')

    new_T_tray_world = amr_center_rigid_transform * amr_rotation * tray_amr_offset

    T_ee_world = T_tray_world * T_hook_ee.inverse()

    T_tray_world.rotation = (T_ee_world * RigidTransform(rotation = new_T_tray_world.rotation, from_frame='ee', to_frame='ee') * T_hook_ee).rotation
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

    T_hook_world_target_70.translation = T_tray_world.translation + np.array([0,0.05,0.08])

    T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

    goal_pose = T_ee_world_target.pose_msg

    print("Rotating end effector to -70 degrees and moving -8cm in x and +20cm in z from tray location")

    while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
        pass

    T_hook_world_target_70.translation = T_tray_world.translation + np.array([0,0.05,0])

    T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

    goal_pose = T_ee_world_target.pose_msg

    print("Moving down 8cm")

    while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
        pass

    T_hook_world_target_70.translation = T_tray_world.translation

    T_ee_world_target = T_hook_world_target_70 * T_hook_ee.inverse()

    goal_pose = T_ee_world_target.pose_msg

    print("Moving forward 8cm")

    while not robot_mg.go_to_pose_goal(goal_pose, cartesian_path=True):
        pass


    import pdb; pdb.set_trace()

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

if __name__ == "__main__":
    run()