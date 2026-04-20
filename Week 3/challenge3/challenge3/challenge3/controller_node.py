import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from challenge3_interfaces.msg import GoalList
import numpy as np

class PID:
    """Implementación estándar de controlador PID con mejoras."""
    def __init__(self, kp, ki, kd, max_out, min_out):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.min_out = min_out
        
        self.error_sum = 0.0
        self.last_error = 0.0

    def compute(self, error, dt):
        if dt <= 0.0:
            return 0.0
        
        # Integral con Anti-windup (limitando la suma)
        self.error_sum = np.clip(self.error_sum + error * dt, -1.0, 1.0)
        
        # Derivada
        d_error = (error - self.last_error) / dt
        self.last_error = error
        
        output = (self.kp * error) + (self.ki * self.error_sum) + (self.kd * d_error)
        
        # Si el error es muy pequeño, permitimos que baje de min_out para frenar
        if abs(error) < 0.01:
            return 0.0
            
        # Limitar salida
        abs_output = np.clip(abs(output), self.min_out, self.max_out)
        return np.sign(output) * abs_output

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(GoalList, '/goals', self.goal_cb, 10)

        # Parámetros PID - Lineal
        self.linear_pid = PID(kp=0.8, ki=0.01, kd=0.05, max_out=0.15, min_out=0.01)
        
        # Ganancia Angular
        self.kp_a = 0.8
        self.max_w, self.min_w = 1.75, 0.1
        
        self.current_goal = None
        self.pose = [0.0, 0.0, 0.0] 
        self.stop_forever = False
        self.last_time = self.get_clock().now()

    def wrap_to_pi(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def odom_cb(self, msg):
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.pose[2] = np.arctan2(siny_cosp, cosy_cosp)
        
        self.control()

    def goal_cb(self, msg):
        new_goal = msg.current_goal.position
        if self.current_goal is None or (self.current_goal.x != new_goal.x or self.current_goal.y != new_goal.y):
            self.current_goal = new_goal
            self.linear_pid.error_sum = 0.0 # Reset integral
        self.stop_forever = msg.final_goal

    def control(self):
        if self.current_goal is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        dx = self.current_goal.x - self.pose[0]
        dy = self.current_goal.y - self.pose[1]
        
        rho = np.sqrt(dx**2 + dy**2) 
        angle_to_goal = np.arctan2(dy, dx)
        alpha = self.wrap_to_pi(angle_to_goal - self.pose[2])

        # Definir límites de llegada
        limit = 0.02 if self.stop_forever else 0.05

        if rho < limit:
            self.stop_robot()
            if self.stop_forever:
                self.get_logger().info("¡Misión finalizada!")
                self.current_goal = None
            return

        cmd = Twist()
        
        # LÓGICA DE CONTROL
        if abs(alpha) > 0.15:
            # Fase A: Rotación pura (más tolerancia para evitar oscilaciones)
            cmd.linear.x = 0.0
            cmd.angular.z = np.sign(alpha) * np.clip(abs(self.kp_a * alpha), self.min_w, self.max_w)
        else:
            # Fase B: Avance y corrección suave
            cmd.linear.x = float(self.linear_pid.compute(rho, dt))
            # Corrección angular proporcional sin saltos bruscos
            cmd.angular.z = np.clip(self.kp_a * alpha, -self.max_w, self.max_w)

        self.vel_pub.publish(cmd)

    def stop_robot(self):
        self.vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
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
