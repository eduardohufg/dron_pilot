#!/usr/bin/env python3
import sys
import select
import tty
import termios
import math
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node

settings = termios.tcgetattr(sys.stdin)

class TeleopDrone(Node):
    def __init__(self):
        super().__init__('teleop_drone')
        

        self.pub_move = self.create_publisher(Pose, '/move/drone', 10)
        self.sub_move = self.create_subscription(Pose, '/move/drone', self.move_callback, 10)

        self.move = Pose()
        self.yaw = 0.0

    def move_callback(self, msg: Pose):
        self.yaw = msg.orientation.z

                    
    def update_position(self, key):
        step = 0.01     
        yaw_step = 0.05  

        # Variables de desplazamiento para esta pulsación
        delta_x = 0.0
        delta_y = 0.0
        delta_z = 0.0

        if key == 'w':  # Avanzar (hacia adelante)
            delta_x = step * math.cos(self.yaw)
            delta_y = step * math.sin(self.yaw)
        elif key == 's':  # Retroceder (hacia atrás)
            delta_x = -step * math.cos(self.yaw)
            delta_y = -step * math.sin(self.yaw)
        elif key == 'a':  # Mover izquierda (lateral)
            delta_x = -step * math.sin(self.yaw)
            delta_y = step * math.cos(self.yaw)
        elif key == 'd':  # Mover derecha (lateral)
            delta_x = step * math.sin(self.yaw)
            delta_y = -step * math.cos(self.yaw)
        elif key == 'e':  # Rotar a la izquierda 
            self.yaw = yaw_step
        elif key == 'q':  # Rotar a la derecha 
            self.yaw = -yaw_step
        elif key == 'r':  # Ascender (subir)
            delta_z = -step
        elif key == 'f':  # Descender (bajar)
            delta_z = step

        # Creamos un nuevo mensaje Pose para enviar solo el delta de esta pulsación
        new_pose = Pose()
        new_pose.position.x = delta_x
        new_pose.position.y = delta_y
        new_pose.position.z = delta_z
        new_pose.orientation.z = self.yaw  # Se mantiene el valor de yaw

        self.pub_move.publish(new_pose)

    # Si usas una variable interna para el mensaje (como self.move), asegúrate de reiniciarla:
    # self.move.position.x = 0.0
    # self.move.position.y = 0.0
    # self.move.position.z = 0.0


def getKey():
   
    global settings
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = TeleopDrone()
        
    node.get_logger().info(
            "\nTeleoperación:\n"
            "t: Despegar\n"
            "l: Aterrizar\n"
            "w: Avanzar\n"
            "s: Retroceder\n"
            "a: Mover izquierda\n"
            "d: Mover derecha\n"
            "r: Subir\n"
            "f: Bajar\n"
            "e: Rotar izquierda\n"
            "q: Rotar derecha\n"
            "Ctrl+C para salir\n"
    )
        
    try:
        while rclpy.ok():
            key = getKey()
            if key == 't':
                node.takeoff()
            elif key == 'l':
                node.land()
            elif key in ['w', 's', 'a', 'd', 'q', 'e', 'r', 'f']:
                node.update_position(key)
            elif key == '\x03': 
                break
            rclpy.spin_once(node, timeout_sec=0.1)
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    main()
