import copy
from typing import List, Union
from geometry_msgs.msg import Pose, Vector3, Quaternion
from tf.transformations import quaternion_from_euler, quaternion_multiply


def pose_msg_to_list(pose: Pose) -> List:
    pos = pose.position
    quat = pose.orientation

    pos = [pos.x, pos.y, pos.z]
    quat = [quat.x, quat.y, quat.z, quat.w]

    return pos + quat


def offset_pose(pose: Pose, trans_offset: List = None, rot_offset: List = None) -> Pose:
    pose = copy.deepcopy(pose)
    if trans_offset is not None:
        assert len(trans_offset) == 3, "trans_offset should be of length 3"
        pose.position.x += trans_offset[0]
        pose.position.y += trans_offset[1]
        pose.position.z += trans_offset[2]
    if rot_offset is not None:
        raise NotImplementedError
    return pose


def offset_pose_relative(poseA: Pose, poseB: Pose, cartesianOnly: bool = True) -> Pose:
    pose = Pose()
    pose.position.x = poseA.position.x - poseB.position.x
    pose.position.y = poseA.position.y - poseB.position.y
    pose.position.z = poseA.position.z - poseB.position.z
    if not cartesianOnly:
        return NotImplementedError
    return pose


def offset_joint(joint_state: List, joint_offset: List) -> List:
    assert len(joint_state) == len(
        joint_offset
    ), "joint_state and joint_offset should be of same length"

    joint_state = copy.deepcopy(joint_state)
    for i in range(len(joint_state)):
        joint_state[i] += joint_offset[i]
    return joint_state


def make_pose(pose: List) -> Pose:
    assert len(pose) == 7, "pose must be of size 7. (xyz + quaternion)"
    return Pose(Vector3(*pose[:3]), Quaternion(*pose[3:]))


def rotate_tool(pose: Union[Pose, List], euler_offset: List) -> List:
    if isinstance(pose, Pose):
        pose = pose_msg_to_list(pose)
    else:
        assert len(pose) == 7
    assert len(euler_offset) == 3

    quat_1 = pose[3:]
    quat_2 = quaternion_from_euler(*euler_offset, axes='rzyx')
    quat_total = quaternion_multiply(quat_1, quat_2).tolist()

    return pose[:3] + quat_total
