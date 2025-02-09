import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

class WebcamPublisher(Node):

    def __init__(self):
        super().__init__('webcam_publisher')
        self.cap = cv2.VideoCapture(2)  # Assuming webcam index is 2

        # Set the webcam resolution to 1280x800
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'camera_sensor/image_raw', 10)  # Adjust queue size if needed
        self.timer = self.create_timer(0.1, self.publish_frame)  # Adjust frame rate as needed

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame from webcam")
            return

        # Convert OpenCV frame to ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = self.get_clock().now().to_msg()

        self.image_pub.publish(image_msg)

def main():
    rclpy.init()
    node = WebcamPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
