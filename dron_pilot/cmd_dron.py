#!/usr/bin/env python3
import sys
import termios
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

class TeleopDrone(Node):
    def __init__(self):
        super().__init__('teleop_drone')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.sub_position = self.create_subscription(Pose, '/goal/position', self.position_callback, 10)
        self.sub_move = self.create_subscription(Pose, '/move/drone', self.move_callback, 10)

        
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0   
        self.yaw = 0.0
        
        # Timer que publica mensajes de control cada 0.1 egundos
        self.timer = self.create_timer(0.1, self.timer_callback)
    

    def position_callback(self, msg):
        self.x = msg.position.x
        self.y = msg.position.y
        self.z = msg.position.z
        self.yaw = msg.orientation.z

    def timer_callback(self):
        # Publicar modo offboard (heartbeat)
        offb_msg = OffboardControlMode()
        offb_msg.position = True
        offb_msg.velocity = False
        offb_msg.acceleration = False
        offb_msg.attitude = False
        offb_msg.body_rate = False
        offb_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(offb_msg)
        
        # Publicar el setpoint de trayectoria (posición y yaw deseados)
        traj_msg = TrajectorySetpoint()
        traj_msg.position = [self.x, self.y, self.z]
        traj_msg.yaw = self.yaw
        traj_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.traj_pub.publish(traj_msg)
    
    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Función auxiliar para enviar comandos al vehículo."""
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
    
    def takeoff(self):
        self.get_logger().info("Despegue iniciado")
        # Activa el modo Offboard y arma el dron
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        # Fija la altura de despegue (en NED, valor negativo = altitud sobre el suelo)
        self.z = -2.0
    
    def land(self):
        self.get_logger().info("Aterrizaje iniciado")
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)


    def move_callback(self, msg):
        dtx = msg.position.x
        dty = msg.position.y
        dtz = msg.position.z
        dtyaw = msg.orientation.z

        global_dx = dtx * math.cos(self.yaw) - dty * math.sin(self.yaw)
        global_dy = dtx * math.sin(self.yaw) + dty * math.cos(self.yaw)

        self.z = self.z + dtz
        self.yaw = self.yaw + dtyaw
        self.x += global_dx
        self.y += global_dy
    
    def update_position(self, key):
  
        step = 0.01      # Paso en metros para el movimiento
        yaw_step = 0.05  # Paso en radianes para modificar el yaw

        if key == 'w':  # Avanzar (hacia adelante)
            dx = step * math.cos(self.yaw)
            dy = step * math.sin(self.yaw)
            self.x += dx
            self.y += dy
        elif key == 's':  # Retroceder (hacia atrás)
            dx = -step * math.cos(self.yaw)
            dy = -step * math.sin(self.yaw)
            self.x += dx
            self.y += dy
        elif key == 'a':  # Mover izquierda (lateral)
            dx = -step * math.sin(self.yaw)
            dy = step * math.cos(self.yaw)
            self.x += dx
            self.y += dy
        elif key == 'd':  # Mover derecha (lateral)
            dx = step * math.sin(self.yaw)
            dy = -step * math.cos(self.yaw)
            self.x += dx
            self.y += dy
        # Rotación
        elif key == 'e':  # Rotar a la izquierda (incrementa yaw)
            self.yaw += yaw_step
        elif key == 'q':  # Rotar a la derecha (disminuye yaw)
            self.yaw -= yaw_step
        # Movimiento vertical
        elif key == 'r':  # Ascender (subir): en NED, disminuir z (más negativo)
            self.z -= step
        elif key == 'f':  # Descender (bajar): en NED, aumentar z (más cercano a 0)
            self.z += step
        
def main(args=None):
    rclpy.init(args=args)
    node = TeleopDrone()
    # Espera 2 segundos para establecer la transmisión de setpoints
    for i in range(20):  # 20 ciclos a 0.1 s cada uno = 2 segundos
        rclpy.spin_once(node, timeout_sec=0.1)
    node.takeoff()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
