import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(Image,'/x500/camera/image_raw', self.image_callback, 10)

        self.center_pub = self.create_publisher(Bool, '/aruco_centered', 10)
        self.aruco_detected_pub = self.create_publisher(Bool, '/aruco_detected', 10)
        self.orientation_ok_pub = self.create_publisher(Bool, '/aruco_orientation_ok', 10)
        self.offset_pub = self.create_publisher(Point, '/aruco_offset', 10)
        self.image_pub = self.create_publisher(Image, '/aruco_detector/image_processed', 10)
        self.offset_orienation = self.create_publisher(Float64, '/aruco_orientation', 10)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        self.center_threshold = 30  # pixeles

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        processed_frame = frame.copy()
        height, width, _ = frame.shape
        image_center = (width // 2, height // 2)

        corners, ids, _ = self.detector.detectMarkers(frame)

        if ids is not None:

            self.aruco_detected_pub.publish(Bool(data=True))

            c = corners[0][0]
            aruco_center_x = int(np.mean(c[:, 0]))
            aruco_center_y = int(np.mean(c[:, 1]))

            dx = aruco_center_x - image_center[0]
            dy = aruco_center_y - image_center[1]

            self.get_logger().info(f"Aruco dx: {dx}, dy: {dy}")

            top_candidates = sorted(c, key=lambda punto: punto[1])[:2]
        # 2. Ordenamos esos dos puntos por su coordenada x para identificar la esquina izquierda y la derecha
            top_left, top_right = sorted(top_candidates, key=lambda punto: punto[0])
            y_top_left = top_left[1]
            y_top_right = top_right[1]

            # Calcular el error de orientación: si el top_left está más abajo que el top_right, error > 0, de lo contrario, error < 0
            error_orientarion = y_top_left - y_top_right

            if abs(error_orientarion) < 5:
                self.orientation_ok_pub.publish(Bool(data=True))
            else:
                self.orientation_ok_pub.publish(Bool(data=False))

            self.offset_orienation.publish(Float64(data=float(error_orientarion)))
            # Publicar si está centrado
            centered = abs(dx) < self.center_threshold and abs(dy) < self.center_threshold
            centered_msg = Bool()
            centered_msg.data = centered
            self.center_pub.publish(centered_msg)

            # Publicar desplazamiento
            offset_msg = Point()
            offset_msg.x = float(dx)
            offset_msg.y = float(dy)
            offset_msg.z = 0.0
            self.offset_pub.publish(offset_msg)

            # Dibujar ArUco y centro
            cv2.aruco.drawDetectedMarkers(processed_frame, corners, ids)
            cv2.circle(processed_frame, (aruco_center_x, aruco_center_y), 6, (0, 255, 0), -1)
            cv2.drawMarker(processed_frame, image_center, (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=15, thickness=2)

        else:
            self.aruco_detected_pub.publish(Bool(data=False))
            self.center_pub.publish(Bool(data=False))
            self.offset_pub.publish(Point(x=0.0, y=0.0, z=0.0))

        # Publicar imagen procesada
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
