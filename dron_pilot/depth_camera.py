#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
from cv_bridge import CvBridge, CvBridgeError

class DepthProcessor(Node):
    def __init__(self):
        super().__init__('depth_processor_node')
        self.subscription = self.create_subscription(Image, '/depth_camera', self.listener_callback, 10)
        self.publisher = self.create_publisher(Float32, '/front_distance', 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            self.get_logger().error("Error al convertir la imagen: %s" % e)
            return

        height, width = cv_image.shape[:2]

        center_pixel = cv_image[height // 2, width // 2]

        distancia_maxima = 10.0  
        distancia = (float(center_pixel) / 255.0) * distancia_maxima
        msg_distance = Float32()
        msg_distance.data = distancia
        self.publisher.publish(msg_distance)
        #self.get_logger().info("Distancia publicada: %.2f metros" % distancia)

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
