import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray  # Import for /rover topic
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton
from PyQt5.QtCore import QTimer
import math

class GoalNavigator(Node):
    def __init__(self):
        super().__init__('final_goal_node')

        # Action client for navigation
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Create publisher for final goal
        self.final_goal_pub = self.create_publisher(PoseStamped, '/final_goal', 10)
        
        # Create subscription for ArUco interrupt
        self.aruco_sub = self.create_subscription(
            String,
            '/aruco_interrupt',
            self.aruco_callback,
            10
        )
        
        # Subscribe to the current position of the robot (from /rover topic)
        self.pose_sub = self.create_subscription(
            Float64MultiArray,
            '/rover',
            self.pose_callback,
            QoSProfile(depth=10)
        )

        self.current_pose = None
        self.aruco_detected = False
        self.goal_pose = PoseStamped()
        
        # Start the GUI
        self.start_gui()
        
    def start_gui(self):
        app = QApplication([])
        window = QWidget()
        layout = QVBoxLayout()
        
        self.x_input = QLineEdit()
        self.y_input = QLineEdit()
        self.angle_input = QLineEdit()
        
        layout.addWidget(QLabel('X Coordinate'))
        layout.addWidget(self.x_input)
        
        layout.addWidget(QLabel('Y Coordinate'))
        layout.addWidget(self.y_input)
        
        layout.addWidget(QLabel('Angle about Z-axis'))
        layout.addWidget(self.angle_input)
        
        submit_button = QPushButton('Submit')
        submit_button.clicked.connect(self.submit_goal)
        layout.addWidget(submit_button)
        
        window.setLayout(layout)
        window.show()
        
        # Set a timer to close the GUI after 10 seconds
        self.timer = QTimer()
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(window.close)
        
        app.exec_()
        
    def submit_goal(self):
        x = float(self.x_input.text())
        y = float(self.y_input.text())
        angle = float(self.angle_input.text())
        
        # Convert angle to quaternion
        quaternion = self.euler_to_quaternion(0, 0, angle)
        
        self.goal_pose.header.frame_id = "map"
        self.goal_pose.pose.position.x = x
        self.goal_pose.pose.position.y = y
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x = quaternion[0]
        self.goal_pose.pose.orientation.y = quaternion[1]
        self.goal_pose.pose.orientation.z = quaternion[2]
        self.goal_pose.pose.orientation.w = quaternion[3]
        
        self.send_goal(self.goal_pose)
        
        # Start the timer to close the GUI after 10 seconds
        self.timer.start(10000)
        
    def euler_to_quaternion(self, roll, pitch, yaw):
        # Convert Euler angles to quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = [0, 0, 0, 0]
        q[0] = cr * cp * cy + sr * sp * sy
        q[1] = sr * cp * cy - cr * sp * sy
        q[2] = cr * sp * cy + sr * cp * sy
        q[3] = cr * cp * sy - sr * sp * cy
        
        return q
        
    def aruco_callback(self, msg):
        if not self.aruco_detected:
            self.aruco_detected = True
            aruco_data = msg.data.split(',')
            marker_id = int(aruco_data[0])
            aruco_pose = [float(val) for val in aruco_data[1:4]]
            aruco_rpy = [float(val) for val in aruco_data[4:]]

            # Calculate absolute position
            absolute_pose = self.calculate_absolute_position(aruco_pose, aruco_rpy)
            
            # Log the absolute pose
            self.get_logger().info(f"Calculated absolute pose: {absolute_pose}")
            
            # Update goal pose
            self.goal_pose.pose.position.x = float(absolute_pose[0])
            self.goal_pose.pose.position.y = float(absolute_pose[1])
            self.goal_pose.pose.position.z = float(absolute_pose[2])

            # Set orientation to face the marker
            orientation = Quaternion()
            orientation.x, orientation.y, orientation.z, orientation.w = self.euler_to_quaternion(0, 0, 0)
            self.goal_pose.pose.orientation = orientation

            self.send_goal(self.goal_pose)
            
    def calculate_absolute_position(self, aruco_pose, aruco_rpy):
        if self.current_pose is None:
            self.get_logger().warning("Current position unknown, unable to calculate absolute position")
            return [0.0, 0.0, 0.0]  # Ensure returned values are floats
        
        # Get the current position of the robot from /rover topic
        current_position = self.current_pose

        # Extract the robot's current x, y, and yaw
        robot_x = current_position[0]
        robot_y = current_position[1]
        robot_yaw = current_position[2]

        # Transform ArUco marker's relative position to global frame
        rel_x, rel_y, rel_z = aruco_pose

        abs_x = robot_x + (rel_x * math.cos(robot_yaw) - rel_y * math.sin(robot_yaw))
        abs_y = robot_y + (rel_x * math.sin(robot_yaw) + rel_y * math.cos(robot_yaw))
        abs_z = 0.0 + rel_z
        
        return [abs_x, abs_y, abs_z]

    def pose_callback(self, msg):
        # Update the current pose with the data from /rover topic
        self.current_pose = msg.data
        
    def send_goal(self, pose):
        self.final_goal_pub.publish(pose)
        self.get_logger().info(f"Published final goal: {pose}")

def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
