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
import csv
import time
import os


class WallFollower(Node):
    def __init__(self):
        super().__init__("wall_follower_copy")
        # Declare parameters to make them available for use
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/vesc/high_level/input/nav_0")
        self.declare_parameter("side", 1)
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

        ### Constants ###
        self.angle_min = 0.1
        self.angle_increment = 0.1
        self.c = 1.0

        ### Turn logic ###
        self.turns = {
            # Opposite Same Front: Action #
            (1, 1, 1): 'straight',
            (1, 1, 0): 'straight',
            (1, 0, 1): 'same',
            (0, 1, 1): 'opp',
            (1, 0, 0): 'same',
            (0, 1, 0): 'straight',
            (0, 0, 1): 'same',
            (0, 0, 0): 'straight'
        }

        self.distance_formula = {
            'ninety': lambda m, b: abs(b),
            'min': lambda m, b: self.c * abs(b) / np.sqrt(m**2 + 1)
        }

        # Threshholds #
        self.front_threshold = self.DESIRED_DISTANCE * 1.0 + self.VELOCITY * 1.0
        self.same_threshold = self.DESIRED_DISTANCE * np.sqrt(2) + self.VELOCITY * 0.0
        self.opp_threshold = self.DESIRED_DISTANCE * 0.5 + self.VELOCITY * 0.0

        # Turn Detection Windows (Defined for the right side) #
        self.front_start, self.front_end = -5, 5
        self.same_start, self.same_end =  -70, -60
        self.opp_start, self.opp_end = -95, -85

        ### Wall Windows (Defined for the right side) ###
        self.wall_start, self.wall_end = -110, -60
        # self.wall_start_open, self.wall_end_open = -130, -100
        self.wall_start_front, self.wall_end_front = -100, 40

        ### PID constants ###
        # self.Kp = 2.2 # 2.5 # previous line
        self.Kp = 1.7 # new line
        self.Kd = 0.2 # 0.3 # previous line
        # self.Kd = 0.16 # new line

        self.prev_e = 0
        self.prev_t = self.get_clock().now().nanoseconds / 1e9

        self.c = 1

        self.csv_file = "distance_data.csv"
    
    def deg_to_index(self, deg):
        return int((deg * math.pi / 180 - self.angle_min) / self.angle_increment)


    def opp_close(self, ranges):
        """
        Check if the opposite window range (mean) is closer than right_threshold
        """
        start, end = self.opp_start, self.opp_end
        if self.SIDE == -1:
            start, end = -end, -start
        if np.min(ranges[self.deg_to_index(start):self.deg_to_index(end)]) < self.opp_threshold:
            return 1
        return 0


    def same_close(self, ranges):
        """
        Check if the front side (half) window has a mean range
        measurement greater than the front_side_threshold
        """
        start, end = self.same_start, self.same_end
        if self.SIDE == 1:
            start, end = -end, -start
        detected_same = np.array(ranges[self.deg_to_index(start):self.deg_to_index(end)])
        if np.mean(detected_same) < self.same_threshold:
            return 1
        return 0

    def front_close(self, ranges):
        """
        Check if the front range is less than the front_close_threshold
        """
        start, end = self.front_start, self.front_end
        detected_front = np.array(ranges[self.deg_to_index(start):self.deg_to_index(end)])
        front_distance = np.mean(detected_front)
        if front_distance < self.front_threshold:
            return 1
        return 0


    def wall_dist(self, ranges, wall_start, wall_end):
        """
        Wall approximation
        """
        ### Wall start and end based on turn logic ###
        if self.same_close(ranges) == 0:
            self.get_logger().info("Open")
            distance_formula = 'ninety'
            wall_start, wall_end = self.wall_start, self.wall_end
        elif self.front_close(ranges) == 1:
            self.get_logger().info("Front")
            distance_formula = 'min'
            wall_start, wall_end = self.wall_start_front, self.wall_end_front
        else:
            self.get_logger().info("Normal")
            distance_formula = 'ninety'
            wall_start, wall_end = self.wall_start, self.wall_end
        if self.SIDE == 1:
            wall_start, wall_end = -wall_end, -wall_start

        wall_start, wall_end = self.deg_to_index(wall_start), self.deg_to_index(wall_end)
        wall_angles = np.array([self.angle_min + self.angle_increment * i for i in range(wall_start, wall_end)])
        detected_wall = np.array(ranges[wall_start:wall_end])

        ### Line Approximation ###
        x = detected_wall * np.cos(wall_angles)
        y = detected_wall * np.sin(wall_angles)

        max_distance = 7
        mask = (x**2 + y**2) <= max_distance**2
        x_filtered = x[mask]
        y_filtered = y[mask]

        if x_filtered.size < 2:
            m, b = 0, self.DESIRED_DISTANCE + 5.0
        else:
            m, b = np.polyfit(x_filtered, y_filtered, deg=1)

        wall_distance = self.distance_formula[distance_formula](m, b)

        self.wall_follower_data(self.csv_file, self.c, self.distance_formula, wall_distance)

        VisualizationTools.plot_line(x, m*x + b, self.line_pub, frame="/laser")

        return wall_distance
    
    def wall_follower_data(csv_filename, c, distance_formula, wall_distance, interval=0.5):
        # Check if the file already exists so we can write header only once.
        file_exists = os.path.exists(csv_filename) and os.stat(csv_filename).st_size > 0
        
        with open(csv_filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # Write header if file is new or empty.
            if not file_exists:
                writer.writerow(["timestamp", "plot_distance"])
            
            if distance_formula == "min":
                plot_distance = wall_distance / c
            else:
                plot_distance = wall_distance
            
            # Get the current time stamp in a readable format.
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            
            # Write the new row to the CSV.
            writer.writerow([timestamp, plot_distance])
            
            # Wait for the specified interval before the next log.
            time.sleep(interval)

    def PID(self, wall_distance):
        """
        PID
        """
        e = self.Kp * (wall_distance - self.DESIRED_DISTANCE) * self.SIDE

        t = self.get_clock().now().nanoseconds / 1e9
        dt = t - self.prev_t
        d = (e - self.prev_e) / dt

        self.prev_e = e
        self.prev_t = t

        u = self.Kp * e + self.Kd * d

        return u

    def pub_PID(self, u):
        """
        Publish Ackermann steering command
        """
        acker = AckermannDriveStamped()
        acker.header.stamp = self.get_clock().now().to_msg()
        acker.drive.speed = self.VELOCITY
        acker.drive.acceleration = 0.0
        acker.drive.jerk = 0.0
        acker.drive.steering_angle = u
        acker.drive.steering_angle_velocity = 0.0
        self.drive_publisher.publish(acker)

        return


    def listener_callback(self, msg):
        print = self.get_logger().info
        ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

        wall_distance = self.wall_dist(ranges, self.wall_start, self.wall_end)
        u = self.PID(wall_distance)
        # turn = self.turns[self.opp_close(ranges), self.same_close(ranges), self.front_close(ranges)]

        self.pub_PID(u)


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
