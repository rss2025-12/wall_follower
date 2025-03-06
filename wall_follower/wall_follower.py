#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult
from wall_follower.visualization_tools import VisualizationTools
from collections import deque


class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("side", -1)
        self.declare_parameter("velocity", 1.0)
        self.declare_parameter("desired_distance", 1.0)

        # Fetch constants from the ROS parameter server
        # DO NOT MODIFY THIS! This is necessary for the tests to be able to test varying parameters!
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
        self.add_on_set_parameters_callback(self.parameters_callback)

	    # TODO: Initialize your publishers and subscribers here
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        self.line_pub = self.create_publisher(Marker, 'wall', 1)
        self.scan_subscriber = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.listener_callback, 10)

        # PID constants
        self.Kp = 8.0
        self.Kd = 0.15
        self.Kf = 25.0

        self.prev_e = 0
        self.prev_t = self.get_clock().now().nanoseconds / 1e9
        self.front_threshold = self.VELOCITY * 0.25

        # self.buffer = deque([], maxlen=5)

    # TODO: Write your callback functions here
    def listener_callback(self, msg):
        def deg_to_index(deg):
            return int((deg * math.pi / 180 - angle_min) / angle_increment)

        def front_distance_error(front_distance, width=0.1):
            """Sigmoid"""
            error = self.front_threshold - front_distance
            return 1 / (1 + np.exp(-error / width))

        print = self.get_logger().info

        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        ### Wall Approximation ###
        wall_start_angle, wall_end_angle = -100, -60
        if self.SIDE == 1:
            wall_start_angle, wall_end_angle = -wall_end_angle, -wall_start_angle
        wall_start, wall_end = deg_to_index(wall_start_angle), deg_to_index(wall_end_angle)
        wall_angles = np.array([angle_min + angle_increment * i for i in range(wall_start, wall_end)])
        detected_wall = np.array(ranges[wall_start:wall_end])

        ### Front Range ###
        front_angle = 2
        front_start, front_end = deg_to_index(-front_angle), deg_to_index(front_angle)
        front_angles = np.array([angle_min + angle_increment * i for i in range(front_start, front_end)])
        detected_front = np.array(ranges[front_start:front_end])
        front_distance = np.mean(detected_front)

        ### Line Approximation ###
        x = detected_wall * np.cos(wall_angles)
        y = detected_wall * np.sin(wall_angles)

        m, b = np.polyfit(x, y, deg=1)

        max_distance = 6
        mask = (x**2 + y**2) <= max_distance**2
        x_filtered = x[mask]
        y_filtered = y[mask]
        if x_filtered.size < 2:
            m, b = 0, self.DESIRED_DISTANCE + 5.0
        else:
            m, b = np.polyfit(x_filtered, y_filtered, deg=1)

        # wall_distance = abs(b) / np.sqrt(1 + m**2)
        wall_distance = abs(b)
        VisualizationTools.plot_line(x, m*x + b, self.line_pub, frame="/laser")

        ### PID ###
        e_side = (wall_distance - self.DESIRED_DISTANCE) * self.SIDE
        e_front = front_distance_error(front_distance) * -self.SIDE

        t = self.get_clock().now().nanoseconds / 1e9
        dt = t - self.prev_t
        d_side = (e_side - self.prev_e) / dt

        self.prev_e = e_side
        self.prev_t = t

        u = self.Kp * e_side + self.Kd * d_side + self.Kf * e_front

        print(f"{u}")

        ### Publishing ###
        acker = AckermannDriveStamped()
        acker.header.stamp = self.get_clock().now().to_msg()
        acker.drive.speed = self.VELOCITY
        acker.drive.acceleration = 0.0
        acker.drive.jerk = 0.0
        acker.drive.steering_angle = u
        acker.drive.steering_angle_velocity = 0.0
        self.drive_publisher.publish(acker)

    def parameters_callback(self, params):
        """
        DO NOT MODIFY THIS CALLBACK FUNCTION!

        This is used by the test cases to modify the parameters during testing.
        It's called whenever a parameter is set via 'ros2 param set'.
        """
        for param in params:
            if param.name == 'side':
                self.SIDE = param.value
                self.get_logger().info(f"Updated side to {self.SIDE}")
            elif param.name == 'velocity':
                self.VELOCITY = param.value
                self.get_logger().info(f"Updated velocity to {self.VELOCITY}")
            elif param.name == 'desired_distance':
                self.DESIRED_DISTANCE = param.value
                self.get_logger().info(f"Updated desired_distance to {self.DESIRED_DISTANCE}")
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
