import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Bool, Float32, Float64


class NodeController(Node):
    def __init__(self):
        super().__init__("node_controller")

        self.pub_goal_position = self.create_publisher(Pose, "/goal/position", 10)
        self.pub_move_drone = self.create_publisher(Pose, "/move/drone", 10)
        self.status_pub = self.create_publisher(Bool, "/status", 10)
        self.create_subscription(Bool, "/aruco_centered", self.aruco_centered_callback, 10)
        self.create_subscription(Bool, "/aruco_detected", self.aruco_detected_callback, 10)
        self.create_subscription(Point, "/aruco_offset", self.aruco_offset_callback, 10)
        self.create_subscription(Float32, "/front_distance", self.front_distance_callback, 10)
        self.create_subscription(Bool, "/init_controller", self.init_controller_callback, 10)
        self.create_subscription(Bool, '/aruco_orientation_ok', self.aruco_orientation_ok_callback, 10)
        self.create_subscription(Float64, '/aruco_orientation', self.aruco_orientation_callback, 10)

        self.create_timer(0.1, self.controller_callback)

        self.aruco_centered = False
        self.aruco_detected = False
        self.aruco_orientation_ok = False
        self.aruco_orientation = 0.0
        self.aruco_offset_x = 0.0
        self.aruco_offset_y = 0.0
        self.front_distance = 0.0

        self.goal_position = Pose()
        self.move_position = Pose()

        self.init = False

        self.init_search = False

        self.p_gain = 0.000005

    def aruco_centered_callback(self, msg):
        self.aruco_centered = msg.data

    def aruco_detected_callback(self, msg):
        self.aruco_detected = msg.data

    def aruco_offset_callback(self, msg):
        self.aruco_offset_x = msg.x
        self.aruco_offset_y = msg.y

    def front_distance_callback(self, msg):

        self.front_distance = msg.data

    def init_controller_callback(self, msg):

        self.init = msg.data

    def aruco_orientation_ok_callback(self, msg):
        self.aruco_orientation_ok = msg.data

    def aruco_orientation_callback(self, msg):
        self.aruco_orientation = msg.data

    def controller_callback(self):

        if self.init:
            self.goal_position.position.x = 8.0
            self.goal_position.position.y = 5.0
            self.goal_position.position.z = -5.0
            self.goal_position.orientation.z = 0.0
            self.pub_goal_position.publish(self.goal_position)
            self.init_search = True
            self.init = False

        if self.init_search:
            self.move_position.orientation.z = 0.05
            self.pub_move_drone.publish(self.move_position)


        if self.aruco_detected and not self.aruco_centered:
            self.init_search = False
            self.move_position.orientation.z = self.aruco_orientation * 0.0001
            self.move_position.position.y = self.aruco_offset_x * self.p_gain
            self.move_position.position.z = self.aruco_offset_y * self.p_gain
            self.pub_move_drone.publish(self.move_position)
        

        if self.aruco_detected and self.aruco_centered and self.front_distance > 0.04 and self.aruco_orientation_ok:
            self.init_search = False
            self.move_position.orientation.z = 0.0
            self.move_position.position.y = 0.0
            self.move_position.position.x = 0.01
            self.pub_move_drone.publish(self.move_position)
        
        if not self.aruco_detected and self.init:
            self.init_search = True

        else:
            self.move_position.position.x = 0.
            self.move_position.position.y = 0.0
            self.move_position.position.z = 0.0
            self.move_position.orientation.z = 0.0
            self.pub_move_drone.publish(self.move_position)


def main(args=None):
    rclpy.init(args=args)
    node_controller = NodeController()
    rclpy.spin(node_controller)
    rclpy.shutdown()


if __name__ == "__main__":

    main()



    

