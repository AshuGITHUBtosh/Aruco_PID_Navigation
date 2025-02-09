#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv2.aruco as aruco
import numpy as np
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler

class ArucoMarkerDetector(Node):
    def __init__(self):
        super().__init__('aruco_marker_detector')

        # Initialize CvBridge
        self.bridge = CvBridge()

        # ArUco marker settings
        self.marker_size = 50  # cm

        # Load camera calibration files
        self.load_calibration()

        # Subscribe to the camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Create ArUco dictionary and parameters
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        self.parameters = aruco.DetectorParameters_create()

        # Create a publisher for marker data
        self.marker_publisher = self.create_publisher(String, '/aruco_interrupt', 10)

    def load_calibration(self):
        # Load camera calibration files
        calib_path = "/home/go4av05/build_ws/src/nav_aruco_pkg/nav_aruco_pkg/calibrationfiles/"
        self.cameraMatrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
        self.cameraDistortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')

    def rotationMatrixToEulerAngles(self, R):
        assert self.isRotationMatrix(R)
        sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6
        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0
        return np.array([x, y, z])

    def isRotationMatrix(self, R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    def image_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error("CvBridge Error: {0}".format(e))
            return

        frame_np = np.array(frame)
        gray_img = cv2.cvtColor(frame_np, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(image=gray_img, dictionary=self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            for i in range(len(ids)):
                marker_id = ids[i][0]
                # Estimate pose for each detected marker
                ret = aruco.estimatePoseSingleMarkers(corners[i], self.marker_size, self.cameraMatrix, self.cameraDistortion)
                rvec, tvec = ret[0][0], ret[1][0]

                # Ensure tvec has the correct shape
                if tvec.shape[0] == 1 and tvec.shape[1] == 3:
                    tvec = tvec[0]
                elif tvec.shape[0] != 3:
                    self.get_logger().error(f"Unexpected tvec shape: {tvec.shape}")
                    continue

                # Convert rotation vector to rotation matrix
                R, _ = cv2.Rodrigues(rvec)

                # Convert rotation matrix to Euler angles
                euler_angles = self.rotationMatrixToEulerAngles(R)

                # Convert radians to degrees
                euler_angles_degrees = np.degrees(euler_angles)

                roll, pitch, yaw = euler_angles_degrees
                x, y, z = tvec

                # Print x, y, z and roll, pitch, yaw in the required format
                self.get_logger().info(f"Marker ID {marker_id} - x: {x:.2f}, y: {y:.2f}, z: {z:.2f}, roll: {roll:.2f}, pitch: {pitch:.2f}, yaw: {yaw:.2f}")

                # Create the message in the desired format
                marker_data = f"{marker_id},{x:.2f},{y:.2f},{z:.2f},{roll:.2f},{pitch:.2f},{yaw:.2f}"

                # Publish the detected marker data
                marker_msg = String()
                marker_msg.data = marker_data
                self.marker_publisher.publish(marker_msg)
                self.get_logger().info(f"Published data for Marker ID {marker_id}")

                # Draw axis on the marker
                aruco.drawAxis(frame, self.cameraMatrix, self.cameraDistortion, rvec, tvec, 10)

                # Draw a square around the marker
                aruco.drawDetectedMarkers(frame, corners, ids)

        # Display annotated frame
        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

    def run(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerDetector()
    node.run()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
