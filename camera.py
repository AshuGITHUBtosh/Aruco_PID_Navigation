#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture('/dev/video2')
        if not self.cap.isOpened():
            self.get_logger().error('Error: Could not open camera.')
            rclpy.shutdown()
        self.timer = self.create_timer(0.01, self.timer_callback)  # Publish at 10 Hz

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Error: Could not read frame.')
            return

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridgeError: {e}')

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# import cv2

# def main():
#     # Open the default camera (index 0)
#     cap = cv2.VideoCapture('rtsp://admin:teamvyadh123@192.168.1.64/1')

#     if not cap.isOpened():
#         print("Error: Could not open camera.")
#         return

#     while True:
#         # Capture frame-by-frame
#         ret, frame = cap.read()
#         if not ret:
#             print("Error: Could not read frame.")
#             break

#         # Display the resulting frame in a window named 'Camera Feed'
#         cv2.imshow('Camera Feed', frame)

#         # Break the loop if 'q' is pressed
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     # Release the capture and close any open windows
#     cap.release()
#     cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()
