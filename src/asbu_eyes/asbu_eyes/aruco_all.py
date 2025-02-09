import cv2
import numpy as np

class ArucoDetector:
    def __init__(self):
        # Set camera frame dimensions
        self.frame_width = 640
        self.frame_height = 480

        # Create a video capture object
        try:
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        except Exception as e:
            print("Error occurred while accessing the camera:", e)
            exit()

        # Load the aruco dictionary
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)

        # Create aruco parameters
        self.parameters = cv2.aruco.DetectorParameters_create()

        # Initialize numpy array to store marker information
        self.marker_info = np.zeros((0, 12), dtype=np.int32)  # [marker-id, tl_x, tr_x, bl_x, br_x, tl_y, tr_y, bl_y, br_y, center_x, center_y, area]

    def detect_markers(self):
        # Reset the array for each frame
        self.marker_info = np.zeros((0, 12), dtype=np.int32)
        
        # Read frame from the camera
        ret, frame = self.cap.read()
        
        # Convert frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        
        # Process detected markers
        if ids is not None:
            for i, marker_id in enumerate(ids):
                marker_corners = corners[i].squeeze()  # Get corners for each marker

                # Calculate area of the quadrilateral formed by ArUco tag
                area = self.calculate_quadrilateral_area(marker_corners)

                # Calculate the center coordinates of the ArUco tag
                center_x = int((marker_corners[0][0] + marker_corners[2][0]) / 2)
                center_y = int((marker_corners[0][1] + marker_corners[2][1]) / 2)

                # Append marker information with area and center coordinates to marker_info array
                total_area = int(area)
                marker_info_row = np.array([marker_id[0], marker_corners[0][0], marker_corners[1][0],
                                            marker_corners[2][0], marker_corners[3][0],
                                            marker_corners[0][1], marker_corners[1][1],
                                            marker_corners[2][1], marker_corners[3][1],
                                            center_x, center_y, total_area], dtype=np.int32)
                self.marker_info = np.vstack((self.marker_info, marker_info_row))

                # Annotate the ArUco tag
                self.annotate_aruco(frame, marker_corners, marker_id[0], center_x, center_y, total_area)
        
        return frame

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
        # Convert center coordinates to integers
        center_x = int(center_x)
        center_y = int(center_y)
        
        # Annotate the numerical value of area in the center of the ArUco tag
        cv2.putText(frame, str(area), (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Annotate a box on the ArUco tag (lavender color)
        cv2.polylines(frame, [np.int32(corners)], True, (230, 230, 250), 2)

        # Annotate the marker id on the top edge (center-aligned) in lavender color, outside
        cv2.putText(frame, str(marker_id), (center_x, int(corners[0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 250), 1, cv2.LINE_AA)

        # Annotate the corner coordinates in very small text at their respective coordinates (white color)
        for corner in corners:
            cv2.putText(frame, f"({corner[0]}, {corner[1]})", (int(corner[0]), int(corner[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)

        # Annotate a red circular dot at the center coordinates
        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

    def draw_vertical_line(self, frame):
        line_color = (0, 255, 0)
        line_thickness = 2
        cv2.line(frame, (self.frame_width // 2, 0), (self.frame_width // 2, self.frame_height), line_color, line_thickness)
        return frame

    def run(self):
        try:
            while True:
                frame = self.detect_markers()
                frame_with_line = self.draw_vertical_line(frame)
                cv2.imshow('Webcam with ArUco Detection', frame_with_line)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
                # Print marker information including area
                print("Marker Information:")
                for info in self.marker_info:
                    print(info)
            self.cap.release()
            cv2.destroyAllWindows()
        except Exception as e:
            print("An error occurred:", e)
            self.cap.release()
            cv2.destroyAllWindows()

# Instantiate the ArucoDetector class and run the program
detector = ArucoDetector()
detector.run()
