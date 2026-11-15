#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String, Bool
import numpy as np

# Configuración PID de carril
KP, KP_TURN = 0.0007, 0.0034 #0.0015 0.004
KI, KD = 0.0000, 0.0095 #0.0075
EMA_ALPHA = 0.75
ANGULAR_MAX = 1.0
LINEAR_NORMAL, LINEAR_TURNING = 0.11, 0.07
TURNING_THRESHOLD = 95.0

class LineController(Node):
    def __init__(self):
        super().__init__('line_controller')
        
        # Suscriptores
        self.error_sub = self.create_subscription(Float32, '/puzzlebot/line_error', self.control_callback, 10)
        self.traffic_sub = self.create_subscription(String, '/semaforo_estado', self.traffic_cb, 10)
        self.inter_sub = self.create_subscription(Bool, '/puzzlebot/intersection', self.intersection_cb, 10)
        
        # Publicador de comandos de velocidad
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Estado PID
        self.prev_error = 0.0
        self.ema_error = 0.0
        self.integral_error = 0.0
        
        # Estado Semáforos
        self.traffic_state = "N"
        self.waiting_for_green = False 
        self.speed_multiplier = 1.0    

        # Estado Intersección
        self.intersection_stop = False

        self.get_logger().info('Nodo Line Controller con Detección de Intersección Activa.')

    def intersection_cb(self, msg):
        # Si se reporta una intersección y el robot aún no ha reaccionado a ella
        if msg.data and not self.intersection_stop:
            self.intersection_stop = True
            self.get_logger().warn('INTERSECCIÓN DETECTADA')

    def traffic_cb(self, msg):
        current_light = msg.data
        if current_light == "R":
            self.traffic_state = "R"
            self.waiting_for_green = True
            self.speed_multiplier = 0.0
            self.get_logger().info('Semáforo ROJO')
        elif current_light == "A":
            if not self.waiting_for_green:
                self.traffic_state = "A"
                self.speed_multiplier = 0.7
                self.get_logger().info('Semáforo AMARILLO')
        elif current_light == "V":
            self.traffic_state = "V"
            self.waiting_for_green = False
            self.speed_multiplier = 1.0
            self.get_logger().info('Semáforo VERDE')
        elif current_light == "N":
            if not self.waiting_for_green:
                self.traffic_state = "N"
                self.speed_multiplier = 1.0

    def control_callback(self, msg):
        # Condición de paro prioritaria 1: Intersección alcanzada
        if self.intersection_stop:
            self.stop_robot()
            return

        # Condición de paro prioritaria 2: Semáforo en rojo / en espera
        if self.waiting_for_green:
            self.stop_robot()
            return

        current_error = msg.data

        # 1. Suavizado del error (Filtro EMA)
        self.ema_error = (EMA_ALPHA * current_error) + ((1.0 - EMA_ALPHA) * self.ema_error)
        
        # 2. Término Integral
        self.integral_error = float(np.clip(self.integral_error + self.ema_error, -2000, 2000))
        
        # 3. Derivada
        derivative = self.ema_error - self.prev_error
        self.prev_error = self.ema_error

        # 4. Asignación dinámica de KP
        active_kp = KP_TURN if abs(self.ema_error) >= TURNING_THRESHOLD else KP

        # 5. Control PID
        steering = -(active_kp * self.ema_error + KI * self.integral_error + KD * derivative)
        
        t_msg = Twist()
        t_msg.angular.z = float(np.clip(steering, -ANGULAR_MAX, ANGULAR_MAX) * self.speed_multiplier)
        
        base_linear = LINEAR_TURNING if abs(self.ema_error) > TURNING_THRESHOLD else LINEAR_NORMAL
        t_msg.linear.x = float(base_linear * self.speed_multiplier)
        
        self.cmd_pub.publish(t_msg)

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LineController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupción por teclado detectada.')
    finally:
        node.stop_robot()
        node.get_logger().info('Cerrando de forma segura')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
