import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Bool, Float32, Float64


class NodeController(Node):
    """
    Node that controls drone movement based on ArUco marker detection and centering,
    as well as the measured front distance.
    """

    def __init__(self):
        super().__init__("node_controller")

        # Publishers
        self.pub_goal_position = self.create_publisher(Pose, "/goal/position", 10)
        self.pub_move_drone = self.create_publisher(Pose, "/move/drone", 10)
        self.status_pub = self.create_publisher(Bool, "/status", 10)

        # Subscribers
        self.create_subscription(Bool, "/aruco_centered", self.aruco_centered_callback, 10)
        self.create_subscription(Bool, "/aruco_detected", self.aruco_detected_callback, 10)
        self.create_subscription(Point, "/aruco_offset", self.aruco_offset_callback, 10)
        self.create_subscription(Float32, "/front_distance", self.front_distance_callback, 10)
        self.create_subscription(Bool, "/init_controller", self.init_controller_callback, 10)
        self.create_subscription(Bool, "/aruco_orientation_ok", self.aruco_orientation_ok_callback, 10)
        self.create_subscription(Float64, "/aruco_orientation", self.aruco_orientation_callback, 10)

        self.create_timer(0.1, self.controller_callback)

        # State variables
        self.aruco_centered = False
        self.aruco_detected = False
        self.aruco_orientation_ok = False
        self.aruco_orientation = 0.0
        self.aruco_offset_x = 0.0
        self.aruco_offset_y = 0.0
        self.front_distance = 0.0
        self.initialize = False
        self.init_search = False

        # Position messages
        self.goal_position = Pose()
        self.move_position = Pose()

        # Gains
        self.p_gain = 0.00005
        self.yaw_gain = 0.0001

    def aruco_centered_callback(self, msg: Bool) -> None:
        self.aruco_centered = msg.data

    def aruco_detected_callback(self, msg: Bool) -> None:
        self.aruco_detected = msg.data

    def aruco_offset_callback(self, msg: Point) -> None:
        self.aruco_offset_x = msg.x
        self.aruco_offset_y = msg.y

    def front_distance_callback(self, msg: Float32) -> None:
        self.front_distance = msg.data

    def init_controller_callback(self, msg: Bool) -> None:
      
        self.initialize = msg.data

    def aruco_orientation_ok_callback(self, msg: Bool) -> None:
        self.aruco_orientation_ok = msg.data

    def aruco_orientation_callback(self, msg: Float64) -> None:
        self.aruco_orientation = msg.data

    def set_goal_position(self, x: float, y: float, z: float, orientation_z: float) -> None:
        
        self.goal_position.position.x = x
        self.goal_position.position.y = y
        self.goal_position.position.z = z
        self.goal_position.orientation.z = orientation_z
        self.pub_goal_position.publish(self.goal_position)

    def publish_move_command(self, x: float, y: float, z: float, orientation_z: float) -> None:
       
        self.move_position.position.x = x
        self.move_position.position.y = y
        self.move_position.position.z = z
        self.move_position.orientation.z = orientation_z
        self.pub_move_drone.publish(self.move_position)

    def controller_callback(self) -> None:
       
        # 1. If 'initialize' is True, set a goal position and start searching
        if self.initialize:
            self.set_goal_position(x=8.0, y=5.0, z=-5.0, orientation_z=0.0)
            self.init_search = True
            self.initialize = False

        # 2. If searching, rotate to find the ArUco
        if self.init_search:
            self.publish_move_command(x=0.0, y=0.0, z=0.0, orientation_z=0.05)

        # 3. If ArUco is detected but not centered, adjust orientation and position
        if self.aruco_detected and not self.aruco_centered:
            self.init_search = False
            orientation_z = self.aruco_orientation * self.yaw_gain
            offset_y = self.aruco_offset_x * self.p_gain
            offset_z = self.aruco_offset_y * self.p_gain
            self.publish_move_command(x=0.0, y=offset_y, z=offset_z, orientation_z=orientation_z)

        # 4. If ArUco is detected, centered, orientation is correct, and front distance > 0.04, move forward
        if (
            self.aruco_detected
            and self.aruco_centered
            and self.front_distance > 0.04
            and self.aruco_orientation_ok
        ):
            self.init_search = False
            self.publish_move_command(x=0.01, y=0.0, z=0.0, orientation_z=0.0)

        # 5. If ArUco is not detected but 'initialize' is still True, keep searching
        if not self.aruco_detected and self.initialize:
            self.init_search = True

        else:
            # 6. Otherwise, stop the movement
            self.publish_move_command(x=0.0, y=0.0, z=0.0, orientation_z=0.0)


def main(args=None):
    rclpy.init(args=args)
    node_controller = NodeController()
    rclpy.spin(node_controller)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
