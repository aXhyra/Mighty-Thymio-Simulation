import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
import numpy as np
import copy

import sys
import math
import time

from .states import ThymioWallStates
from .utils import *
from .controllers import ProportionalController

MAX = 0.15


def normalize(value, min_value, max_value):
    if value < min_value:
        return min_value
    return 1 - ((value - min_value) / (max_value - min_value))


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Create attributes to store odometry pose and velocity
        self.initial_pose = None
        self.goal = None
        self.odom_velocity = None
        self.timer = None
        self.odom_pose = None
        self.odom_velocity = None

        # Create attributes to store proximity sensor values
        # front sensors:
        self.prox_l = None
        self.prox_cl = None
        self.prox_c = None
        self.prox_cr = None
        self.prox_r = None

        # back sensors:
        self.prox_rl = None
        self.prox_rr = None

        self.prox_rl_buf = np.zeros(3)
        self.prox_rr_buf = np.zeros(3)

        self.prox_rl_buf_idx = 0
        self.prox_rr_buf_idx = 0

        self.proportional_controller = ProportionalController(2)

        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Create a subscriber to the topic 'proximity/center', which will call
        # self.proximity_callback every time a message is received
        self.proximity_center_subscriber = self.create_subscription(Range,
                                                                    'proximity/center',
                                                                    self.proximity_c,
                                                                    10)

        # Create a subscriber to the topic 'proximity/center_left', which will call
        # self.proximity_callback every time a message is received
        self.proximity_center_left_subscriber = self.create_subscription(Range,
                                                                         'proximity/center_left',
                                                                         self.proximity_cl,
                                                                         10)

        # Create a subscriber to the topic 'proximity/center_right', which will call
        # self.proximity_callback every time a message is received
        self.proximity_center_right_subscriber = self.create_subscription(Range,
                                                                          'proximity/center_right',
                                                                          self.proximity_cr,
                                                                          10)

        # Create a subscriber to the topic 'proximity/left', which will call
        # self.proximity_callback every time a message is received
        self.proximity_left_subscriber = self.create_subscription(Range,
                                                                  'proximity/left',
                                                                  self.proximity_l,
                                                                  10)

        # Create a subscriber to the topic 'proximity/rear_left', which will call
        # self.proximity_callback every time a message is received
        self.proximity_rear_left_subscriber = self.create_subscription(Range,
                                                                       'proximity/rear_left',
                                                                       self.proximity_rl,
                                                                       10)

        # Create a subscriber to the topic 'proximity/rear_right', which will call
        # self.proximity_callback every time a message is received
        self.proximity_rear_right_subscriber = self.create_subscription(Range,
                                                                        'proximity/rear_right',
                                                                        self.proximity_rr,
                                                                        10)

        self.state = ThymioWallStates.NO_WALL
        self.prev_state = None
        self.pose2d = None

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

    def proximity_l(self, msg):
        self.prox_l = normalize(msg.range, 0, MAX)
        # self.get_logger().info("proximity/left: received distance {:.2f}".format(self.prox_l),
        #                        throttle_duration_sec=0.5)
        if self.state == ThymioWallStates.NO_WALL and self.prox_l > 0.7:
            self.get_logger().info("proximity: received left proximity: {:.2f}".format(self.prox_l),
                                   throttle_duration_sec=0.5)
            self.state = ThymioWallStates.WALL_L

    def proximity_cl(self, msg):
        self.prox_cl = normalize(msg.range, 0, MAX)
        # self.get_logger().info("proximity/center_left: received distance {:.2f}".format(self.prox_cl),
        #                        throttle_duration_sec=0.5)
        if self.state == ThymioWallStates.NO_WALL and self.prox_cl > 0.7:
            self.get_logger().info("proximity: received center left proximity: {:.2f}".format(self.prox_cl),
                                   throttle_duration_sec=0.5)
            self.state = ThymioWallStates.WALL_L

    def proximity_c(self, msg):
        # self.get_logger().info("proximity/center: received distance {:.2f}".format(normalize(msg.range, 0, MAX)),
        #                        throttle_duration_sec=0.5)
        self.prox_c = normalize(msg.range, 0, MAX)

    def proximity_cr(self, msg):
        self.prox_cr = normalize(msg.range, 0, MAX)
        # self.get_logger().info("proximity/center_right: received distance {:.2f}".format(self.prox_cr),
        #                        throttle_duration_sec=0.5)
        if self.state == ThymioWallStates.NO_WALL and self.prox_cr > 0.7:
            self.get_logger().info("proximity: received center right proximity: {:.2f}".format(self.prox_cr),
                                   throttle_duration_sec=0.5)
            self.state = ThymioWallStates.WALL_R

    def proximity_r(self, msg):
        self.prox_r = normalize(msg.range, 0, MAX)
        # self.get_logger().info("proximity/right: received distance {:.2f}".format(self.prox_r),
        #                        throttle_duration_sec=0.5)
        if self.state == ThymioWallStates.NO_WALL and self.prox_r > 0.7:
            self.get_logger().info("proximity: received right proximity: {:.2f}".format(self.prox_r),
                                   throttle_duration_sec=0.1)
            self.state = ThymioWallStates.WALL_R

    def proximity_rl(self, msg):
        # self.get_logger().info("proximity/right_left: received distance {:.2f}".format(msg.range),
        #                        throttle_duration_sec=0.5)
        self.prox_rl_buf[self.prox_rl_buf_idx] = normalize(msg.range, 0, MAX)
        self.prox_rl_buf_idx = (self.prox_rl_buf_idx + 1) % 3
        self.prox_rl = np.mean(self.prox_rl_buf)

    def proximity_rr(self, msg):
        # self.get_logger().info("proximity/right_right: received distance {:.2f}".format(msg.range),
        #                        throttle_duration_sec=0.5)
        self.prox_rr_buf[self.prox_rr_buf_idx] = normalize(msg.range, 0, MAX)
        self.prox_rr_buf_idx = (self.prox_rr_buf_idx + 1) % 3
        self.prox_rr = np.mean(self.prox_rr_buf)

    def get_linear_vel(self, goal_dist):
        new_vel = self.proportional_controller.step(goal_dist - euclidean_distance(self.initial_pose, self.pose2d))
        return new_vel

    def update_callback(self):
        # Let's just set some hard-coded velocities in this example
        cmd_vel = Twist()
        # origin = (0., 0.)
        if self.pose2d is None:
            return

        cmd_vel = Twist()

        if self.state == ThymioWallStates.NO_WALL:
            # Move forwards with constant velocity
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = 0.0
        elif self.state == ThymioWallStates.WALL_L:
            # Rotate counterclockwise with constant angular velocity
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = math.pi / 6
            self.prev_state = ThymioWallStates.WALL_L
            self.state = ThymioWallStates.ROTATE_TO_WALL
        elif self.state == ThymioWallStates.WALL_R:
            # Rotate clockwise with constant angular velocity
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = -math.pi / 6
            self.prev_state = ThymioWallStates.WALL_R
            self.state = ThymioWallStates.ROTATE_TO_WALL
        elif self.state == ThymioWallStates.ROTATE_TO_WALL:
            if self.prev_state == ThymioWallStates.WALL_L and abs(self.prox_cl - self.prox_cr) > 0.1:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = math.pi / 6
            elif self.prev_state == ThymioWallStates.WALL_R and abs(self.prox_cl - self.prox_cr) > 0.1:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = -math.pi / 6
            else:
                self.state = ThymioWallStates.DONE_ROTATE
                # Done point 2
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
        elif self.state == ThymioWallStates.DONE_ROTATE:
            # Beginning point 3
            if self.prox_cl <= 0.9 and self.prox_cr <= 0.9:
                cmd_vel.linear.x = 0.02
                cmd_vel.angular.z = 0.0
            else:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.state = ThymioWallStates.ROTATE_AWAY
        elif self.state == ThymioWallStates.ROTATE_AWAY:
            if self.prox_rl <= 0.2 and self.prox_rr <= 0.2:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = math.pi / 13
            elif abs(self.prox_rl - self.prox_rr) > 0.012:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = math.pi / 13
            else:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.state = ThymioWallStates.MOVING_AWAY
                self.initial_pose = self.pose2d
        elif self.state == ThymioWallStates.MOVING_AWAY:
            cmd_vel.linear.x = self.get_linear_vel(2)
            cmd_vel.angular.z = 0.0

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
