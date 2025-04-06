#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
from cv_bridge import CvBridge, CvBridgeError


class DepthProcessor(Node):
    """
    Node that processes depth images and publishes the distance at the center pixel.
    The distance is calculated by mapping the 8-bit depth value (0-255) to a 
    maximum distance in meters.
    """

    def __init__(self):
        super().__init__('depth_processor_node')
        
        # Subscriptions
        self.subscription = self.create_subscription(Image, '/depth_camera', self.image_callback, 10)

        # Publishers
        self.publisher = self.create_publisher(Float32, '/front_distance', 10)

        self.bridge = CvBridge()

    def image_callback(self, msg: Image) -> None:
      
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        height, width = cv_image.shape[:2]

        center_pixel_value = cv_image[height // 2, width // 2]

        max_distance = 10.0
        distance = (float(center_pixel_value) / 255.0) * max_distance

        distance_msg = Float32()
        distance_msg.data = distance
        self.publisher.publish(distance_msg)

        # self.get_logger().info(f"Published distance: {distance:.2f} m")


def main(args=None):
    rclpy.init(args=args)
    node = DepthProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
