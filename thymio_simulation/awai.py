import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
import numpy as np

import sys
import math

from .utils import *
from thymio_simulation.utils.sensors import SensorTriggered

MAX = 0.15
TRIGGER_THRESHOLD = 1/5


class Awai(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Create attributes to store odometry pose and velocity
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

        self.prox_l_buf = np.zeros(3)
        self.prox_cl_buf = np.zeros(3)
        self.prox_c_buf = np.zeros(3)
        self.prox_cr_buf = np.zeros(3)
        self.prox_r_buf = np.zeros(3)

        self.prox_l_buf_idx = 0
        self.prox_cl_buf_idx = 0
        self.prox_c_buf_idx = 0
        self.prox_cr_buf_idx = 0
        self.prox_r_buf_idx = 0

        self.sensors_triggered = []

        self.all_triggered = False

        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 2)

        # Create a subscriber to the topic 'odom', which will call
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 2)

        # Create a subscriber to the topic 'proximity/center', which will call
        # self.proximity_callback every time a message is received
        self.proximity_center_subscriber = self.create_subscription(Range,
                                                                    'proximity/center',
                                                                    self.proximity_c,
                                                                    2)

        # Create a subscriber to the topic 'proximity/center_left', which will call
        # self.proximity_callback every time a message is received
        self.proximity_center_left_subscriber = self.create_subscription(Range,
                                                                         'proximity/center_left',
                                                                         self.proximity_cl,
                                                                         2)

        # Create a subscriber to the topic 'proximity/center_right', which will call
        # self.proximity_callback every time a message is received
        self.proximity_center_right_subscriber = self.create_subscription(Range,
                                                                          'proximity/center_right',
                                                                          self.proximity_cr,
                                                                          2)

        # Create a subscriber to the topic 'proximity/left', which will call
        # self.proximity_callback every time a message is received
        self.proximity_left_subscriber = self.create_subscription(Range,
                                                                  'proximity/left',
                                                                  self.proximity_l,
                                                                  2)

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

    def proximity_l(self, msg):
        self.prox_l_buf[self.prox_l_buf_idx] = normalize(msg.range, 0, MAX)
        self.prox_l_buf_idx = (self.prox_r_buf_idx + 1) % self.prox_l_buf.shape[0]

        self.prox_l = np.mean(self.prox_l_buf)

        if self.prox_l >= TRIGGER_THRESHOLD:
            self.sensors_triggered.append(SensorTriggered.LEFT)
        elif SensorTriggered.LEFT in self.sensors_triggered:
            self.sensors_triggered.remove(SensorTriggered.LEFT)

    def proximity_cl(self, msg):
        self.prox_cl_buf[self.prox_cl_buf_idx] = normalize(msg.range, 0, MAX)
        self.prox_cl_buf_idx = (self.prox_cl_buf_idx + 1) % self.prox_cl_buf.shape[0]

        self.prox_cl = np.mean(self.prox_cl_buf)

        if self.prox_cl >= TRIGGER_THRESHOLD:
            self.sensors_triggered.append(SensorTriggered.CENTER_LEFT)
        elif SensorTriggered.CENTER_LEFT in self.sensors_triggered:
            self.sensors_triggered.remove(SensorTriggered.CENTER_LEFT)

    def proximity_c(self, msg):
        self.prox_c_buf[self.prox_c_buf_idx] = normalize(msg.range, 0, MAX)
        self.prox_c_buf_idx = (self.prox_c_buf_idx + 1) % self.prox_c_buf.shape[0]

        self.prox_c = np.mean(self.prox_c_buf)

        # if self.prox_c > 0:
        #     self.sensors_triggered.append(SensorTriggered.CENTER)
        # elif SensorTriggered.CENTER in self.sensors_triggered:
        #     self.sensors_triggered.remove(SensorTriggered.CENTER)

    def proximity_cr(self, msg):
        self.prox_cr_buf[self.prox_cr_buf_idx] = normalize(msg.range, 0, MAX)
        self.prox_cr_buf_idx = (self.prox_cr_buf_idx + 1) % self.prox_cr_buf.shape[0]

        self.prox_cr = np.mean(self.prox_cr_buf)

        if self.prox_cr >= TRIGGER_THRESHOLD:
            self.sensors_triggered.append(SensorTriggered.CENTER_RIGHT)
        elif SensorTriggered.CENTER_RIGHT in self.sensors_triggered:
            self.sensors_triggered.remove(SensorTriggered.CENTER_RIGHT)

    def proximity_r(self, msg):
        self.prox_r_buf[self.prox_r_buf_idx] = normalize(msg.range, 0, MAX)
        self.prox_r_buf_idx = (self.prox_r_buf_idx + 1) % self.prox_r_buf.shape[0]

        self.prox_r = np.mean(self.prox_r_buf)

        if self.prox_r >= TRIGGER_THRESHOLD:
            self.sensors_triggered.append(SensorTriggered.RIGHT)
        elif SensorTriggered.RIGHT in self.sensors_triggered:
            self.sensors_triggered.remove(SensorTriggered.RIGHT)

    def update_callback(self):
        # Let's just set some hard-coded velocities in this example
        cmd_vel = Twist()
        # origin = (0., 0.)
        if self.pose2d is None:
            return

        cmd_vel = Twist()

        if not self.sensors_triggered:
            cmd_vel.linear.x = 0.2
            cmd_vel.angular.z = 0.0
            self.all_triggered = False
        else:
            cmd_vel.linear.x = 0.0
            if SensorTriggered.CENTER_LEFT in self.sensors_triggered and SensorTriggered.CENTER_RIGHT in self.sensors_triggered:
                self.all_triggered = True
                cmd_vel.angular.z = -math.pi / 2
            if self.all_triggered:
                cmd_vel.angular.z = math.pi / 2
            elif SensorTriggered.LEFT in self.sensors_triggered or SensorTriggered.CENTER_LEFT in self.sensors_triggered:
                cmd_vel.angular.z = -math.pi/2
            elif SensorTriggered.RIGHT in self.sensors_triggered or SensorTriggered.CENTER_RIGHT in self.sensors_triggered:
                cmd_vel.angular.z = math.pi/2

        # Publish the command
        self.vel_publisher.publish(cmd_vel)


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)

    # Create an instance of your node class
    node = Awai()
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
