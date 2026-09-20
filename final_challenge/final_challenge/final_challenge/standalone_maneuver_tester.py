#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import threading
import time

class StandaloneManeuverTester(Node):
    def __init__(self):
        super().__init__('standalone_maneuver_tester')
        
        # Suscriptor a la odometría real
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        # Publicador directo a los motores
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.pose = [0.0, 0.0, 0.0]
        self.start_pose = [0.0, 0.0, 0.0]
        
        self.in_maneuver = False
        self.maneuver_type = None

        self.get_logger().info('✅ Tester de Odometría Independiente Activo.')

    def wrap_to_pi(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def odom_cb(self, msg):
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.pose[2] = np.arctan2(siny_cosp, cosy_cosp)
        
        # Si hay una rutina activa, el callback de odometría la ejecuta en bucle
        if self.in_maneuver:
            self.execute_maneuver()

    def start_maneuver(self, maneuver_type):
        self.in_maneuver = True
        self.maneuver_type = maneuver_type
        self.start_pose = list(self.pose)
        self.get_logger().info(f'🚀 Ejecutando movimiento directo: {maneuver_type.upper()}')

    def execute_maneuver(self):
        dx = self.pose[0] - self.start_pose[0]
        dy = self.pose[1] - self.start_pose[1]
        distance = np.sqrt(dx**2 + dy**2)
        delta_theta = abs(self.wrap_to_pi(self.pose[2] - self.start_pose[2]))
        
        t_msg = Twist()
        
        # --- AQUÍ TUNEAS TUS VELOCIDADES Y DISTANCIAS ---
        if self.maneuver_type == "ahead":
            if distance >= 0.45:  # Distancia objetivo en metros
                self.end_maneuver()
                return
            t_msg.linear.x = 0.08
            t_msg.angular.z = 0.0

        elif self.maneuver_type == "right":
            if delta_theta >= (np.pi / 2.0):  # 90 grados en radianes
                self.end_maneuver()
                return
            t_msg.linear.x = 0.077 * 0.5
            t_msg.angular.z = -0.28 *0.5

        elif self.maneuver_type == "left":
            if delta_theta >= (np.pi / 2.0):  # 90 grados en radianes
                self.end_maneuver()
                return
            t_msg.linear.x = 0.077 *0.5
            t_msg.angular.z = 0.28 *0.5

        self.cmd_pub.publish(t_msg)

    def end_maneuver(self):
        self.in_maneuver = False
        self.maneuver_type = None
        self.stop_robot()
        self.get_logger().info('✅ Movimiento completado. Motores detenidos.')

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

def cli_menu(node):
    while rclpy.ok():
        print("\n" + "="*40)
        print("🤖 TESTER DIRECTO DE MOTORES Y ODOMETRÍA 🤖")
        print("="*40)
        print("1. Girar Izquierda (90 grados)")
        print("2. Girar Derecha (90 grados)")
        print("3. Avanzar Recto (18 cm)")
        print("4. Detener Robot (Emergencia)")
        print("5. Salir")
        
        choice = input("Elige una acción (1-5): ")
        
        if choice == '1':
            node.start_maneuver('left')
        elif choice == '2':
            node.start_maneuver('right')
        elif choice == '3':
            node.start_maneuver('ahead')
        elif choice == '4':
            node.in_maneuver = False
            node.stop_robot()
            print("🛑 Detenido de emergencia.")
        elif choice == '5':
            print("Saliendo...")
            node.stop_robot()
            rclpy.shutdown()
            break
        else:
            print("❌ Opción inválida.")
            
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = StandaloneManeuverTester()
    
    # Hilo separado para que rclpy.spin escuche a /odom mientras el menú sigue abierto
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        cli_menu(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()

if __name__ == '__main__':
    main()