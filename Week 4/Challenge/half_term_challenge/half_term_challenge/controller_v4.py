import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from half_term_challenge_interfaces.msg import GoalList
import numpy as np

# Clase para el controlador PID (Proporcional, Integral, Derivativo)
class PID:
    def __init__(self, kp, ki, kd, max_out, min_out):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.min_out = min_out
        self.error_sum = 0.0  # Acumulador para el término integral
        self.last_error = 0.0 # Error previo para el término derivativo

    def compute(self, error, dt):
        if dt <= 0.0: return 0.0
        
        # Cálculo integral con anti-windup (limitando la suma)
        self.error_sum = np.clip(self.error_sum + error * dt, -1.0, 1.0)
        
        # Cálculo derivativo (tasa de cambio del error)
        d_error = (error - self.last_error) / dt
        self.last_error = error
        
        # Fórmula general del PID
        output = (self.kp * error) + (self.ki * self.error_sum) + (self.kd * d_error)
        
        # Umbral de tolerancia para evitar pequeñas oscilaciones al final
        if abs(error) < 0.01: return 0.0
        
        # Limitar la salida entre el rango mínimo y máximo permitido
        abs_output = np.clip(abs(output), self.min_out, self.max_out)
        return np.sign(output) * abs_output


class ControllerNodeFinal(Node):
    def __init__(self):
        super().__init__('controller_node_final')
        # Publicador de velocidad
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Suscriptores: Odometría, Metas y Estado del Semáforo
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(GoalList, '/goals', self.goal_cb, 10)
        self.create_subscription(String, '/semaforo_estado', self.traffic_cb, 10)

        # PID de lazo externo (Posición -> Determina la velocidad deseada)
        self.linear_pid = PID(kp=0.8, ki=0.01, kd=0.05, max_out=0.15, min_out=0.01)

        # PID de lazo interno (Seguimiento de velocidad: Corrige la velocidad actual)
        self.velocity_pid = PID(kp=0.5, ki=0.0, kd=0.01, max_out=0.18, min_out=0.0)
        self.current_vel_x = 0.0

        # Parámetros para el control de rotación (Angular)
        self.kp_a = 0.8
        self.max_w, self.min_w = 1.75, 0.1

        self.current_goal = None
        self.pose = [0.0, 0.0, 0.0] # [x, y, yaw]
        self.stop_forever = False   # Indica si es el objetivo final de la ruta
        self.last_time = self.get_clock().now()

        # Lógica de semáforos
        self.traffic_state = "N"
        self.waiting_for_green = False # Candado para esperar luz verde tras una roja
        self.speed_multiplier = 1.0    # Modificador de velocidad (0.0 para detenerse, 0.7 lento)

    # Función para normalizar ángulos entre -pi y pi
    def wrap_to_pi(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    # Callback del semáforo
    def traffic_cb(self, msg):
        current_light = msg.data

        if current_light == "R": # ROJO
            self.traffic_state = "R"
            self.waiting_for_green = True   # Bloqueo: debe esperar a que cambie a verde
            self.speed_multiplier = 0.0

        elif current_light == "A": # AMARILLO
            # Solo reduce velocidad si no está ya bloqueado por una luz roja previa
            if not self.waiting_for_green:
                self.traffic_state = "A"
                self.speed_multiplier = 0.7

        elif current_light == "V": # VERDE
            self.traffic_state = "V"
            self.waiting_for_green = False  # El verde libera el bloqueo del rojo
            self.speed_multiplier = 1.0

        elif current_light == "N": # NINGUNO / OFF
            if not self.waiting_for_green:
                self.traffic_state = "N"
                # Si venía de amarillo, se mantiene lento hasta que vea un cambio claro

    # Callback de odometría: actualiza posición y activa el control
    def odom_cb(self, msg):
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        
        # Conversión de Cuaternión a ángulo de Euler (Yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.pose[2] = np.arctan2(siny_cosp, cosy_cosp)
        
        self.current_vel_x = msg.twist.twist.linear.x
        self.control() # Ejecuta el algoritmo de control en cada actualización de pose

    # Callback de metas: recibe el objetivo actual
    def goal_cb(self, msg):
        new_goal = msg.current_goal.position
        # Si la meta cambia, reiniciamos el término integral del PID
        if self.current_goal is None or (
            self.current_goal.x != new_goal.x or
            self.current_goal.y != new_goal.y
        ):
            self.current_goal = new_goal
            self.linear_pid.error_sum = 0.0
        self.stop_forever = msg.final_goal

    # Algoritmo principal de control
    def control(self):
        if self.current_goal is None:
            return

        # Parada inmediata si el semáforo está en rojo (esperando verde)
        if self.waiting_for_green:
            self.stop_robot()
            return

        # Calcular delta de tiempo (dt)
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Calcular error de distancia (rho) y error de ángulo (alpha)
        dx = self.current_goal.x - self.pose[0]
        dy = self.current_goal.y - self.pose[1]
        rho = np.sqrt(dx**2 + dy**2)
        alpha = self.wrap_to_pi(np.arctan2(dy, dx) - self.pose[2])

        # Definir umbral de llegada (más estricto si es la meta final)
        limit = 0.02 if self.stop_forever else 0.05
        if rho < limit:
            self.stop_robot()
            if self.stop_forever:
                self.current_goal = None # Se detiene definitivamente
            return

        cmd = Twist()
        # Fase 1: Si el ángulo es muy grande, gira sobre su propio eje
        if abs(alpha) > 0.15:
            cmd.linear.x = 0.0
            cmd.angular.z = np.sign(alpha) * np.clip(
                abs(self.kp_a * alpha), self.min_w, self.max_w
            )
        # Fase 2: Si está alineado, avanza usando PID en cascada
        else:
            # PID Exterior: Calcula qué velocidad lineal deberíamos tener según la distancia
            v_desired = self.linear_pid.compute(rho, dt) * self.speed_multiplier
            
            # PID Interior: Ajusta la potencia del motor comparando velocidad deseada vs actual
            vel_error = v_desired - self.current_vel_x
            cmd.linear.x = float(v_desired + self.velocity_pid.compute(vel_error, dt))
            
            # Ajuste fino de dirección mientras avanza
            cmd.angular.z = np.clip(self.kp_a * alpha, -self.max_w, self.max_w)

        self.vel_pub.publish(cmd)

    # Función auxiliar para enviar velocidad cero
    def stop_robot(self):
        self.vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNodeFinal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
