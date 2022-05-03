import tf_transformations


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
