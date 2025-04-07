import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from rclpy.executors import SingleThreadedExecutor
from threading import Thread
from typing import List, Callable
import json  # Para parsear los datos del WebSocket
import math
import numpy as np
import cv2
import base64

class ROS2Bridge(Node):
    def __init__(self):
        super().__init__('fastapi_ros2_bridge')
        
        self.pub_move = self.create_publisher(Pose, '/move/drone', 10)
        self.pub_init = self.create_publisher(Bool, '/init_controller', 10)
        self.sub_move = self.create_subscription(Pose, '/move/drone', self.move_callback, 10)
        self.image_subscription = self.create_subscription(Image, '/aruco_detector/image_processed', self.image_callback, 10)

        self._callbacks: List[Callable[[str], None]] = []

        self._image_callbacks: List[Callable[[str], None]] = []

        self.yaw = 0.0

        self.inited = Bool()

    def move_callback(self, msg: Pose):
        self.yaw = msg.orientation.z

    def publish_message(self, message: str):
        try:
            data = json.loads(message)

            step = 0.03     
            yaw_step = 0.05 
            delta_x = 0.0
            delta_y = 0.0
            delta_z = 0.0

            if int(data["position"]["x_pos"]) == 1:
                delta_x = step * math.cos(self.yaw)
                delta_y = step * math.sin(self.yaw)
            elif int(data["position"]["x_neg"]) == 1:
                delta_x = -step * math.cos(self.yaw)
                delta_y = -step * math.sin(self.yaw)
            elif int(data["position"]["y_pos"]) == 1:
                delta_x = -step * math.sin(self.yaw)
                delta_y = step * math.cos(self.yaw)
            elif int(data["position"]["y_neg"]) == 1:
                delta_x = step * math.sin(self.yaw)
                delta_y = -step * math.cos(self.yaw)
            elif int(data["orientation"]["yaw_pos"]) == 1:
                self.yaw = yaw_step
            elif int(data["orientation"]["yaw_neg"]) == 1:
                self.yaw = -yaw_step

            elif int(data["position"]["z_pos"]) == 1:
                delta_z = -step
            elif int(data["position"]["z_neg"]) == 1:
                delta_z = step

            if int(data['command']['init']) == 1:
                self.inited.data = True
                self.pub_init.publish(self.inited)
            else:
                self.inited.data = False
                self.pub_init.publish(self.inited)


            new_pose = Pose()
            new_pose.position.x = delta_x
            new_pose.position.y = delta_y
            new_pose.position.z = delta_z
            new_pose.orientation.z = self.yaw 

            self.pub_move.publish(new_pose)


        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().error(f'Error processing message: {e}')

    def listener_callback(self, msg: String):
        self.get_logger().info(f'Received: "{msg.data}"')
        for callback in self._callbacks:
            callback(msg.data)


    def image_callback(self, msg: Image):
      
        try:
          
            height = msg.height
            width = msg.width
            channels = 3
            img_array = np.array(msg.data, dtype=np.uint8).reshape((height, width, channels))
            
            _, buffer = cv2.imencode(".jpg", img_array)
         
            frame_base64 = base64.b64encode(buffer).decode("utf-8")
            
            for cb in self._image_callbacks:
                cb(frame_base64)
        
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


    def register_callback(self, callback: Callable[[str], None]):
        self._callbacks.append(callback)

    def register_image_callback(self, callback: Callable[[str], None]):
        self._image_callbacks.append(callback)

ros2_node: ROS2Bridge = None 
executor = None

def start_ros2():
    global ros2_node, executor
    rclpy.init()
    ros2_node = ROS2Bridge()  # Inicializa ros2_node aquí
    executor = SingleThreadedExecutor()
    executor.add_node(ros2_node)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()

def stop_ros2():
    global ros2_node, executor
    executor.shutdown()
    ros2_node.destroy_node()
    rclpy.shutdown()

def get_ros2_node() -> ROS2Bridge:
    return ros2_node 
