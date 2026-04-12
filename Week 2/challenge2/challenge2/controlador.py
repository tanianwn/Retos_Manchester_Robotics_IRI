import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from challenge2_interfaces.msg import Trajectories

class ControladorFSM(Node):
    def __init__(self):
        super().__init__('controlador_node')

        self.subscription = self.create_subscription(Trajectories, '/pose', self.path_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Estados
        self.STATE_IDLE = 0
        self.STATE_ROTATE = 1
        self.STATE_MOVE = 2
        self.state = self.STATE_IDLE

        # Calibración física
        self.LINEAR_CALIB = 0.9569378 #0.9569378
        self.ANGULAR_CALIB = 0.97 #0.97

        # Pose interna (asume inicio en 0,0,0)
        self.cur_x, self.cur_y, self.cur_theta = 0.0, 0.0, 0.0
        self.points = []
        self.current_idx = 0
        
        self.v_lin = 0.0
        self.v_ang = 0.0
        
        self.state_start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Controlador listo y esperando trayectoria...")

    def path_callback(self, msg):
        self.get_logger().info("Nueva trayectoria recibida. Iniciando...")
        self.cur_x, self.cur_y, self.cur_theta = 0.0, 0.0, 0.0
        self.points = msg.points
        self.v_lin = msg.linear_velocity
        self.v_ang = msg.angular_velocity
        self.current_idx = 0
        self.state = self.STATE_ROTATE
        self.state_start_time = self.get_clock().now()

    def control_loop(self):
        if self.state == self.STATE_IDLE:
            return

        now = self.get_clock().now()
        elapsed = (now - self.state_start_time).nanoseconds * 1e-9
        cmd = Twist()

        target = self.points[self.current_idx]
        dx = target.x - self.cur_x
        dy = target.y - self.cur_y
        dist = np.sqrt(dx**2 + dy**2)
        target_angle = np.arctan2(dy, dx)
        angle_to_turn = (target_angle - self.cur_theta + np.pi) % (2 * np.pi) - np.pi

        if self.state == self.STATE_ROTATE:
            duration = abs(angle_to_turn / self.v_ang) * self.ANGULAR_CALIB
            if elapsed < duration:
                cmd.angular.z = self.v_ang if angle_to_turn > 0 else -self.v_ang
            else:
                self.cur_theta = target_angle
                self.state = self.STATE_MOVE
                self.state_start_time = now
                self.get_logger().info(f"Orientación hacia Punto {self.current_idx + 1} lista. Avanzando...")

        elif self.state == self.STATE_MOVE:
            duration = (dist / self.v_lin) * self.LINEAR_CALIB if self.v_lin > 0 else 0.0
            if elapsed < duration:
                cmd.linear.x = self.v_lin
            else:
                self.cur_x = target.x
                self.cur_y = target.y
                self.get_logger().info(f"¡Punto {self.current_idx + 1} alcanzado! ({target.x}, {target.y})")
                # --------------------
                self.current_idx += 1
                
                if self.current_idx < len(self.points):
                    self.state = self.STATE_ROTATE
                else:
                    self.state = self.STATE_IDLE
                    self.get_logger().info("¡Destino alcanzado!")
                self.state_start_time = now

        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ControladorFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
