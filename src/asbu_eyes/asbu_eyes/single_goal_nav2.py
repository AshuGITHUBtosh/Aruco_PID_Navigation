import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class SimpleNav(Node):
    def __init__(self):
        super().__init__('simple_nav')
        self.publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = 5.0
        goal_msg.pose.position.y = 1.0
        goal_msg.pose.orientation.w = 1.0

        self.publisher.publish(goal_msg)
        self.get_logger().info('Goal pose published.')

def main(args=None):
    rclpy.init(args=args)
    simple_nav = SimpleNav()
    rclpy.spin(simple_nav)
    simple_nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
