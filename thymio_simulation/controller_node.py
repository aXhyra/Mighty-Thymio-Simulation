import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import copy

import sys
import math
import time

from .states import ThymioStates
from .utils import *


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Create attributes to store odometry pose and velocity
        self.odom_velocity = None
        self.timer = None
        self.odom_pose = None
        self.odom_velocity = None

        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call 
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.state = ThymioStates.FIRST_HALF
        self.counter = 0
        self.pose2d = None
        self.step = 0
        self.start_time = self.get_clock().now()

        # NOTE: we're using relative names to specify the topics (i.e., without a 
        # leading /). ROS resolves relative names by concatenating them with the 
        # namespace in which this node has been started, thus allowing us to 
        # specify which Thymio should be controlled.

    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1 / 60, self.update_callback)

    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_velocity = msg.twist.twist

        self.pose2d = pose3d_to_2d(self.odom_pose)

        # self.get_logger().info(
        #     "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*self.pose2d),
        #      throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        # )

    def update_callback(self):
        # Let's just set some hard-coded velocities in this example
        cmd_vel = Twist()
        # origin = (0., 0.)
        if self.pose2d is None:
            return

        if self.step == 0:
            self.start_time = self.get_clock().now()

        self.step += 1

        self.get_logger().debug(f'Time elapsed: {(self.get_clock().now() - self.start_time).nanoseconds / 1e9}',
                                throttle_duration_sec=0.5)
        self.get_logger().debug(f'Step: {self.step}', throttle_duration_sec=1)

        if self.state == ThymioStates.FIRST_HALF:
            cmd_vel.linear.x = 0.1  # [m/s]
            cmd_vel.angular.z = math.pi / 8  # [rad/s]
            time_elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if int(time_elapsed) >= 16 * 1.25:
                self.state = ThymioStates.SECOND_HALF
                self.start_time = self.get_clock().now()
        elif self.state == ThymioStates.SECOND_HALF:
            cmd_vel.linear.x = 0.1
            # Since the two circumferences of an 8 are not exactly equal, change the angular velocity slightly
            cmd_vel.angular.z = -math.pi / 9
            time_elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if int(time_elapsed) >= 18 * 1.25:
                self.state = ThymioStates.FIRST_HALF
                self.step = 0
                self.start_time = self.get_clock().now()

        # Publish the command
        self.vel_publisher.publish(cmd_vel)


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)

    # Create an instance of your node class
    node = ControllerNode()
    node.start()

    # Keep processing events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Ensure the Thymio is stopped before exiting
    node.stop()


if __name__ == '__main__':
    main()
