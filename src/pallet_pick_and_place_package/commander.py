#!/usr/bin/env python3
from __future__ import print_function

import sys
import math
import numpy as np
from typing import List, Optional, Union
import pyquaternion as pyq

import rospy
import actionlib
import moveit_commander
import std_msgs.msg
import moveit_msgs.msg as mi_msg

from geometry_msgs.msg import Pose, PoseStamped, Twist

from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)

from moveit.core.kinematic_constraints import constructGoalConstraints
from moveit_commander.conversions import pose_to_list
from moveit_msgs.srv import GetMotionPlan

from enum import Enum


## Uncomment to use with Gazebo
# MOVEIT_CONTROLLER = "pos_joint_traj_controller"
# POSE_CONTROLLER = "pose_based_cartesian_traj_controller"
# TWIST_CONTROLLER = "twist_controller"
# avail_controllers = [MOVEIT_CONTROLLER]


def _joints_close(goal: List, actual: List, tolerance: float):
    """
    Check if the joint values in two lists are within a tolerance of each other
    """
    for index in range(len(goal)):
        if abs(actual[index] - goal[index]) > tolerance:
            err = goal[index] - actual[index]
            rospy.logwarn(
                f"Joint error ({err} rad) is greater than the tolerance ({tolerance} rad)"
            )
            return False
    return True


def _poses_close(
    goal: Union[Pose, PoseStamped],
    actual: Union[Pose, PoseStamped],
    pos_tolerance: float,
    orient_tolerance: float,
):
    """
    Check if the actual and goal poses are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    """

    goal = goal.pose if isinstance(goal, PoseStamped) else goal
    actual = actual.pose if isinstance(actual, PoseStamped) else actual

    x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
    x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
    # Euclidean distance
    d = math.dist((x1, y1, z1), (x0, y0, z0))
    # angle between orientations
    quat_1 = pyq.Quaternion(qx0, qy0, qz0, qw0)
    quat_2 = pyq.Quaternion(qx1, qy1, qz1, qw1)
    phi = 2 * min(
        pyq.Quaternion.distance(quat_1, quat_2),
        pyq.Quaternion.distance(quat_1, -quat_2)
    )

    result = True
    if d > pos_tolerance:
        rospy.logwarn(
            f"Position error ({d} m) is greater than the tolerance ({pos_tolerance} m)"
        )
        result = False
    if phi > orient_tolerance:
        rospy.logwarn(
            f"Orientation error ({phi} rad) is greater than the tolerance ({orient_tolerance} rad)"
        )
        result = False
    return result


class RobotMoveGroup(object):

    JOINTS = [
        "joint_1_s",
        "joint_2_l",
        "joint_3_u",
        "joint_4_r",
        "joint_5_b",
        "joint_6_t",
    ]

    def __init__(self, namespace: str = '/', verbose: bool = False) -> None:

        self._verbose = verbose
        # initialize `moveit_commander` and a `rospy` node
        moveit_commander.roscpp_initialize(sys.argv)
        # set a default timeout threshold non-motion requests
        self.timeout = rospy.Duration(5)

        # setup moveit publishers, services, and actions
        self.get_plan = rospy.ServiceProxy("/" + namespace+"/plan_kinematic_path", GetMotionPlan)
        rospy.wait_for_service("/" + namespace+"/plan_kinematic_path", self.timeout)
        self.execute_plan = actionlib.SimpleActionClient(
            "/" + namespace+"/execute_trajectory", mi_msg.ExecuteTrajectoryAction
        )

        # self.trajectory_client = actionlib.SimpleActionClient(
        #         "{}/follow_cartesian_trajectory".format(target_controller),
        #         FollowCartesianTrajectoryAction,
        # )


        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        self.robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for maintaining the robot's internal understanding of the environment
        self.scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface to a
        ## planning group (group of joints). This interface is used to plan and execute motions
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.move_group.set_max_acceleration_scaling_factor(0.03)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher(
            "/" + namespace+"/move_group/display_planned_path", mi_msg.DisplayTrajectory, queue_size=20
        )

        # Get the name of the reference frame for this robot
        self.planning_frame = self.move_group.get_planning_frame()
        # Get the name of the end-effector link for this group:
        self.eef_frame = self.move_group.get_end_effector_link()
        # Get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()

        self.HOME_JOINTS = [0, 0, 0, 0, -np.pi/2, 0]

        if self._verbose:
            print("============ Planning frame: %s" % self.planning_frame)
            print("============ End effector link: %s" % self.eef_frame)
            print(
                "============ Available Planning Groups:", self.robot.get_group_names()
            )
            print("============ Printing robot state")
            print(self.robot.get_current_state())
            print("")

    def go_home(self, 
        cartesian_path: bool = False,
        tolerance: float = 0.001,
        velocity_scaling: float = 0.01,
        acc_scaling: float = 0.01,
        wait: bool = True):
        return self.go_to_joint_state(self.HOME_JOINTS, cartesian_path, tolerance, velocity_scaling, acc_scaling, wait)


    def get_current_pose(self, end_effector_link: str = "", stamped: bool = False) -> Union[Pose, PoseStamped]:
        """
        position in metres. orientation in radians
        """
        if stamped:
            return self.move_group.get_current_pose(end_effector_link=end_effector_link)
        else:
            return self.move_group.get_current_pose(end_effector_link=end_effector_link).pose

    def get_current_joints(self, in_degrees: bool = False) -> List:
        """
        joint angle values in radians (or) degrees
        """
        joint_state = self.move_group.get_current_joint_values()
        if in_degrees:
            joint_state = [np.rad2deg(joint) for joint in joint_state]
        return joint_state

    def go_to_joints(self, joint_goal: List[float], wait:bool = True):

        self.move_group.set_max_velocity_scaling_factor(0.03)
        self.move_group.set_joint_value_target(joint_goal)
        ret_val = self.move_group.go(wait)
        return ret_val

    def go_to_joint_state(
        self,
        joint_goal: List[float],
        cartesian_path: bool = False,
        tolerance: float = 0.001,
        velocity_scaling: float = 0.05,
        acc_scaling: float = 0.05,
        wait: bool = True,
    ) -> bool:
        # Check if MoveIt planner is running
        #rospy.wait_for_service("/plan_kinematic_path", self.timeout)
        # Create a motion planning request with all necessary goals and constraints
        mp_req = mi_msg.MotionPlanRequest()
        mp_req.pipeline_id = "pilz_industrial_motion_planner"
        mp_req.planner_id = "LIN" if cartesian_path else "PTP"
        mp_req.group_name = "manipulator"
        mp_req.num_planning_attempts = 1

        constraints = mi_msg.Constraints()
        for joint_no in range(len(self.JOINTS)):
            constraints.joint_constraints.append(mi_msg.JointConstraint())
            constraints.joint_constraints[-1].joint_name = self.JOINTS[joint_no]
            constraints.joint_constraints[-1].position = joint_goal[joint_no]
            constraints.joint_constraints[-1].tolerance_above = tolerance
            constraints.joint_constraints[-1].tolerance_below = tolerance

        mp_req.goal_constraints.append(constraints)
        mp_req.max_velocity_scaling_factor = velocity_scaling
        mp_req.max_acceleration_scaling_factor = acc_scaling

        mp_res = self.get_plan(mp_req).motion_plan_response
        if mp_res.error_code.val != mp_res.error_code.SUCCESS:
            rospy.logerr(
                "Planner failed to generate a valid plan to the goal joint_state"
            )
            return False
        if (
            len(mp_res.trajectory.joint_trajectory.points) > 1
            and mp_res.trajectory.joint_trajectory.points[-1].time_from_start
            == mp_res.trajectory.joint_trajectory.points[-2].time_from_start
        ):
            mp_res.trajectory.joint_trajectory.points.pop(-2)
            rospy.logwarn(
                "Duplicate time stamp in the planned trajectory. Second last way-point was removed."
            )
        goal = mi_msg.ExecuteTrajectoryGoal(trajectory=mp_res.trajectory)
        self.execute_plan.wait_for_server()
        self.execute_plan.send_goal(goal)
        if wait:
            self.execute_plan.wait_for_result()
            # Calling ``stop()`` ensures that there is no residual movement
            self.move_group.stop()
            return _joints_close(joint_goal, self.get_current_joints(), tolerance)
        return True

    def go_to_pose_goal(
        self,
        pose_goal: Pose,
        cartesian_path=True,
        pos_tolerance: float = 0.001,
        orient_tolerance: float = 0.01,
        velocity_scaling: float = 0.1,
        eef_frame: Optional[str] = None,
        acc_scaling: float = 0.1,
        wait: bool = True,
    ) -> bool:

        if eef_frame is None:
            eef_frame = self.eef_frame
        # Check if MoveIt planner is running
        #rospy.wait_for_service("/plan_kinematic_path", self.timeout)
        # Create a motion planning request with all necessary goals and constraints
        mp_req = mi_msg.MotionPlanRequest()
        mp_req.pipeline_id = "pilz_industrial_motion_planner"
        mp_req.planner_id = "LIN" if cartesian_path else "PTP"
        mp_req.group_name = "manipulator"
        mp_req.num_planning_attempts = 5

        mp_req_pose_goal = PoseStamped(
            header=std_msgs.msg.Header(frame_id=self.planning_frame), pose=pose_goal
        )

        constraints = constructGoalConstraints(
            eef_frame, mp_req_pose_goal, pos_tolerance, orient_tolerance
        )
        mp_req.goal_constraints.append(constraints)
        mp_req.max_velocity_scaling_factor = velocity_scaling
        mp_req.max_acceleration_scaling_factor = acc_scaling

        mp_res = self.get_plan(mp_req).motion_plan_response
        if mp_res.error_code.val != mp_res.error_code.SUCCESS:
            rospy.logerr("Planner failed to generate a valid plan to the goal pose")
            return False
        goal = mi_msg.ExecuteTrajectoryGoal(trajectory=mp_res.trajectory)
        self.execute_plan.wait_for_server()
        self.execute_plan.send_goal(goal)
        if wait:
            self.execute_plan.wait_for_result()
            # Calling ``stop()`` ensures that there is no residual movement
            self.move_group.stop()
            return _poses_close(
                pose_goal, self.get_current_pose(eef_frame), pos_tolerance, orient_tolerance
            )
        return True

    def send_cartesian_pos_trajectory(
        self, pose_list: List[Pose], wait: bool = False
    ):
        """
        Sends a cartesian position trajectory to the robot
        """

        (plan, fraction) = self.move_group.compute_cartesian_path(pose_list, 0.01, 0.0)
        self.move_group.execute(plan)

        # goal = FollowCartesianTrajectoryGoal()

        # for i, pose in enumerate(pose_list):
        #     point = CartesianTrajectoryPoint()
        #     point.pose = pose
        #     point.time_from_start = rospy.Duration(t_durations[i])
        #     goal.trajectory.points.append(point)

        # self.trajectory_client.send_goal(goal)
        # if wait:
        #     self.trajectory_client.wait_for_result()
        #     result = self.trajectory_client.get_result()
        #     rospy.loginfo(f"Trajectory execution finished in state {result.error_code}")
