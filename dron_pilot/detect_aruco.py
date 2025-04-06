import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np


class ArucoDetector(Node):
    """
    Node that detects ArUco markers in images, computes their offset 
    relative to the image center, checks orientation, and publishes results.
    """

    def __init__(self):
        super().__init__('aruco_detector')

        self.bridge = CvBridge()

        # Subscribers
        self.image_subscription = self.create_subscription(Image, '/x500/camera/image_raw', self.image_callback, 10)

        # Publishers
        self.aruco_centered_pub = self.create_publisher(Bool, '/aruco_centered', 10)
        self.aruco_detected_pub = self.create_publisher(Bool, '/aruco_detected', 10)
        self.orientation_ok_pub = self.create_publisher(Bool, '/aruco_orientation_ok', 10)
        self.offset_pub = self.create_publisher(Point, '/aruco_offset', 10)
        self.image_pub = self.create_publisher(Image, '/aruco_detector/image_processed', 10)
        self.offset_orientation_pub = self.create_publisher(Float64, '/aruco_orientation', 10)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector_parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_parameters)

        self.center_threshold = 30

        # State variables
        self.aruco_detected_msg = Bool()
        self.orientation_ok_msg = Bool()
        self.offset_orientation = Float64()
        self.offset_position = Point()

    def image_callback(self, msg: Image) -> None:
   
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        processed_frame = frame.copy()

        height, width, _ = frame.shape
        image_center = (width // 2, height // 2)

        corners, ids, _ = self.detector.detectMarkers(frame)

        if ids is not None and len(corners) > 0:

            self.aruco_detected_msg.data = True
            self.aruco_detected_pub.publish(self.aruco_detected_msg)

            first_marker_corners = corners[0][0]
            aruco_center_x = int(np.mean(first_marker_corners[:, 0]))
            aruco_center_y = int(np.mean(first_marker_corners[:, 1]))

            dx = aruco_center_x - image_center[0]
            dy = aruco_center_y - image_center[1]

            #self.get_logger().info(f"Aruco dx: {dx}, dy: {dy}")

            # Determine the top edge corners of the detected ArUco marker
            top_candidates = sorted(first_marker_corners, key=lambda point: point[1])[:2]
            top_left, top_right = sorted(top_candidates, key=lambda point: point[0])

            y_top_left = top_left[1]
            y_top_right = top_right[1]

            # Calculate orientation error: difference in y-coordinates of top corners
            orientation_error = y_top_left - y_top_right

            if abs(orientation_error) < 5:
                self.orientation_ok_msg.data = True
                self.orientation_ok_pub.publish(self.orientation_ok_msg)
            else:
                self.orientation_ok_msg.data = False
                self.orientation_ok_pub.publish(self.orientation_ok_msg)
                self.offset_orientation.data = float(orientation_error)

            self.offset_orientation_pub.publish(self.offset_orientation)

            centered = abs(dx) < self.center_threshold and abs(dy) < self.center_threshold
            centered_msg = Bool(data=centered)
            self.aruco_centered_pub.publish(centered_msg)

            self.offset_position.x = float(dx)
            self.offset_position.y = float(dy)
            self.offset_position.z = 0.0
            self.offset_pub.publish(self.offset_position)

            cv2.aruco.drawDetectedMarkers(processed_frame, corners, ids)
            cv2.circle(processed_frame, (aruco_center_x, aruco_center_y), 6, (0, 255, 0), -1)
            cv2.drawMarker(
                processed_frame,
                image_center,
                (0, 0, 255),
                markerType=cv2.MARKER_CROSS,
                markerSize=15,
                thickness=2
            )
        else:

            self.aruco_detected_msg.data = False
            self.aruco_detected_pub.publish(self.aruco_detected_msg)

            self.aruco_centered_pub.publish(Bool(data=False))
            self.offset_position.x = 0.0
            self.offset_position.y = 0.0
            self.offset_position.z = 0.0
            self.offset_pub.publish(self.offset_position)

        out_msg = self.bridge.cv2_to_imgmsg(processed_frame, encoding='bgr8')
        self.image_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
