import cv2
import numpy as np
import pyrealsense2 as rs

class RealSenseArucoDetector:
    def __init__(self):
        # Initialize the RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Configure the stream to 720p resolution
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

        # Start the pipeline
        self.pipeline.start(self.config)

    def calculate_quadrilateral_area(self, corners):
        # Calculate area of triangle 1
        x1_tl, y1_tl = corners[0]
        x2_tr, y2_tr = corners[1]
        x3_br, y3_br = corners[3]
        area_triangle1 = 0.5 * abs(x1_tl * (y2_tr - y3_br) + x2_tr * (y3_br - y1_tl) + x3_br * (y1_tl - y2_tr))

        # Calculate area of triangle 2
        x4_bl, y4_bl = corners[2]
        area_triangle2 = 0.5 * abs(x1_tl * (y4_bl - y3_br) + x4_bl * (y3_br - y1_tl) + x3_br * (y1_tl - y4_bl))

        # Calculate total area of the quadrilateral
        total_area = area_triangle1 + area_triangle2
        return total_area

    def annotate_aruco(self, frame, corners, marker_id, center_x, center_y, area):
        if not corners or len(corners) == 0:
            return

        # Convert center coordinates to integers
        center_x = int(center_x)
        center_y = int(center_y)

        # Annotate the numerical value of area in the center of the ArUco tag
        cv2.putText(frame, str(area), (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Draw a green outline on the ArUco marker
        cv2.polylines(frame, [np.int32(corners)], True, (0, 255, 0), 2)

        if not marker_id:
            return

        # Annotate the marker id in red color
        org = (int(corners[0][0][0]), int(corners[0][0][1]))
        cv2.putText(frame, str(marker_id), org, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        # Annotate a red circular dot at the centroid of the marker
        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

        # Annotate the corner coordinates in orange color
        for corner in corners:
            cv2.putText(frame, f"({corner[0]}, {corner[1]})", (int(corner[0]), int(corner[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 165, 255), 1)

    def detect_markers(self, frame):
        # Convert the color frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Initialize the ArUco dictionary and parameters
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        parameters = cv2.aruco.DetectorParameters_create()

        # Detect ArUco markers in the grayscale frame
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # Optionally, draw the detected markers and their IDs on the color image
        if ids is not None:
            for i in range(len(ids)):
                marker_id = ids[i][0]
                marker_corners = corners[i][0]

                # Calculate the center coordinates of the ArUco marker
                center_x = int(np.mean(marker_corners[:, 0]))
                center_y = int(np.mean(marker_corners[:, 1]))

                # Calculate the area of the ArUco marker
                area = self.calculate_quadrilateral_area(marker_corners)

                # Annotate the ArUco marker on the frame
                self.annotate_aruco(frame, marker_corners, marker_id, center_x, center_y, area)

        return frame

    def run(self):
        try:
            while True:
                # Wait for the next frame
                frames = self.pipeline.wait_for_frames()

                # Get the color frame
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                # Convert the color frame to a numpy array
                color_image = np.asanyarray(color_frame.get_data())

                # Detect ArUco markers
                annotated_frame = self.detect_markers(color_image)

                # Display the color image with detected markers and annotations
                cv2.imshow('RealSense with ArUco Detection', annotated_frame)

                # Exit the loop if 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            # Stop the pipeline and release resources
            self.pipeline.stop()
            cv2.destroyAllWindows()


if __name__ == '__main__':
    detector = RealSenseArucoDetector()
    detector.run()

