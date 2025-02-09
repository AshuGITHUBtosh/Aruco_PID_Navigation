#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray  # Import the standard Float64MultiArray message type
import math

class RoverPosePublisher(Node):
    def __init__(self):
        super().__init__('rover_pose_publisher')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.publisher_ = self.create_publisher(Float64MultiArray, '/rover', 10)

        # Initialize the origin
        self.initial_x = None
        self.initial_y = None

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Set the origin if it's not set yet
        if self.initial_x is None and self.initial_y is None:
            self.initial_x = x
            self.initial_y = y

        # Calculate the position relative to the origin
        rel_x = x - self.initial_x
        rel_y = y - self.initial_y

        # Orientation in quaternion (x, y, z, w)
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Create a Float64MultiArray message
        pose_yaw_array = Float64MultiArray()
        pose_yaw_array.data = [rel_x, rel_y, yaw]

        # Publish the array
        self.publisher_.publish(pose_yaw_array)

def main(args=None):
    rclpy.init(args=args)
    rover_pose_publisher = RoverPosePublisher()

    try:
        rclpy.spin(rover_pose_publisher)
    except KeyboardInterrupt:
        pass

    rover_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
