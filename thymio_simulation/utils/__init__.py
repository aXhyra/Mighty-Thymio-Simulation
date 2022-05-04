import tf_transformations
from math import sqrt, atan2, pi


def pose3d_to_2d(pose3):
    quaternion = (
        pose3.orientation.x,
        pose3.orientation.y,
        pose3.orientation.z,
        pose3.orientation.w
    )

    roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)

    pose2 = (
        pose3.position.x,  # x position
        pose3.position.y,  # y position
        yaw  # theta orientation
    )

    return pose2


def euclidean_distance(start_pose, current_pose):
    start_pose_x = start_pose[0]
    start_pose_y = start_pose[1]

    current_pose_x = current_pose[0]
    current_pose_y = current_pose[1]

    return sqrt(pow((start_pose_x - current_pose_x), 2) +
                pow((start_pose_y - current_pose_y), 2))
