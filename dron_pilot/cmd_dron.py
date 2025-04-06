#!/usr/bin/env python3

import sys
import termios
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


class CmdDrone(Node):
    """
    Node that allows teleoperation of a drone via Offboard control commands and
    trajectory setpoints based on received Pose messages.
    """

    def __init__(self):
        super().__init__('cmd_drone')

        # Define QoS settings for PX4 topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)

        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        self.cmd_pub = self.create_publisher(VehicleCommand,'/fmu/in/vehicle_command',qos_profile)

        # Subscriptions
        self.sub_position = self.create_subscription(Pose, '/goal/position', self.position_callback, 10)
        self.sub_move = self.create_subscription(Pose, '/move/drone', self.move_callback, 10)

        # Position and orientation state
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0

        self.timer = self.create_timer(0.1, self.timer_callback)

    def position_callback(self, msg: Pose):
        
        self.x = msg.position.x
        self.y = msg.position.y
        self.z = msg.position.z
        self.yaw = msg.orientation.z

    def timer_callback(self):
        
        offb_msg = OffboardControlMode()
        offb_msg.position = True
        offb_msg.velocity = False
        offb_msg.acceleration = False
        offb_msg.attitude = False
        offb_msg.body_rate = False
        offb_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(offb_msg)

        traj_msg = TrajectorySetpoint()
        traj_msg.position = [self.x, self.y, self.z]
        traj_msg.yaw = self.yaw
        traj_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.traj_pub.publish(traj_msg)

    def send_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        
        cmd = VehicleCommand()
        cmd.command = command
        cmd.param1 = param1
        cmd.param2 = param2
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(cmd)

    def takeoff(self) -> None:
     
        self.get_logger().info("Takeoff initiated.")
        # Activate Offboard mode and arm the drone
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.z = -2.0

    def land(self) -> None:
       
        self.get_logger().info("Landing initiated.")
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def move_callback(self, msg: Pose) -> None:
       
        dtx = msg.position.x
        dty = msg.position.y
        dtz = msg.position.z
        dtyaw = msg.orientation.z

        # Transform local increments into global coordinates
        global_dx = dtx * math.cos(self.yaw) - dty * math.sin(self.yaw)
        global_dy = dtx * math.sin(self.yaw) + dty * math.cos(self.yaw)

        self.z += dtz
        self.yaw += dtyaw
        self.x += global_dx
        self.y += global_dy


def main(args=None):
    rclpy.init(args=args)
    node = CmdDrone()
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)

    node.takeoff()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
