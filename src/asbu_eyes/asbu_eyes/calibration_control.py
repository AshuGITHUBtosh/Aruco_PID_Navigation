import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np

class BotControlNode(Node):
    def __init__(self):
        super().__init__('bot_control_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)  # Reduced timer interval for slower movement
        self.angular_speed = np.deg2rad(10.0)  # Angular speed in radians per second (adjust as needed)
        self.forward_speed = 0.05  # Linear speed in meters per second (adjust as needed)
        self.spin_direction = 1  # 1 for clockwise, -1 for counter-clockwise

    def control_loop(self):
        twist_msg = Twist()

        # Set angular velocity for spinning
        twist_msg.angular.z = self.spin_direction * self.angular_speed

        # Set linear velocity for forward movement
        twist_msg.linear.x = self.forward_speed

        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    bot_control_node = BotControlNode()
    rclpy.spin(bot_control_node)
    bot_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
