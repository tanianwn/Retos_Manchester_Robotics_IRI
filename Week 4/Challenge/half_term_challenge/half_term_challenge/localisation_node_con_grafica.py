import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import transforms3d
import numpy as np
import matplotlib.pyplot as plt

class LocalisationNode(Node):
    def __init__(self):
        super().__init__('localisation_node')
        
        # --- Constantes Físicas ---
        self._l = 0.18  # Distancia entre ruedas (Wheelbase)
        self._r = 0.05  # Radio de las ruedas
        
        # --- Variables de Estado ---
        self.X, self.Y, self.Th = 0.0, 0.0, 0.0
        self.wr, self.wl = 0.0, 0.0
        self.last_time = self.get_clock().now()

        # --- Buffers de datos para la gráfica ---
        self.history_x = []
        self.history_y = []

        # --- Perfil QoS para compatibilidad con sensores ---
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        # --- Suscripciones y Publicadores ---
        self.create_subscription(Float32, 'VelocityEncR', self.wr_cb, qos)
        self.create_subscription(Float32, 'VelocityEncL', self.wl_cb, qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # --- Configuración de Matplotlib ---
        plt.ion() # Modo interactivo activado
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'b-', label='Trayectoria') # Línea de camino
        self.robot_dot, = self.ax.plot([], [], 'ro', label='Robot')    # Posición actual
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_title('Trayectoria del Robot en Tiempo Real')
        self.ax.legend()
        self.ax.grid(True)
        self.ax.axis('equal')

        # Temporizador a 50Hz (0.02s)
        self.create_timer(0.02, self.update)
        self.get_logger().info("Nodo de Localización Iniciado con Gráfica.")

    def wr_cb(self, msg): 
        self.wr = msg.data

    def wl_cb(self, msg): 
        self.wl = msg.data

    def update(self):
        # Calcular el diferencial de tiempo (dt)
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0: return
        self.last_time = now

        # --- Cálculo de Velocidades Lineal y Angular ---
        v = self._r * (self.wr + self.wl) / 2.0
        w = self._r * (self.wr - self.wl) / self._l

        # --- Integración de Euler para la Pose ---
        self.Th += w * dt
        self.X += v * np.cos(self.Th) * dt
        self.Y += v * np.sin(self.Th) * dt

        # Guardar posición en el historial
        self.history_x.append(self.X)
        self.history_y.append(self.Y)

        # Publicar mensaje de Odometría
        self.publish_odom(now, v, w)
        
        # Actualizar la gráfica visual
        self.update_plot()

    def publish_odom(self, now, v, w):
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Posición
        odom.pose.pose.position.x = self.X
        odom.pose.pose.position.y = self.Y
        
        # Convertir ángulo Th (Euler) a Cuaternión para ROS2
        q = transforms3d.euler.euler2quat(0, 0, self.Th)
        odom.pose.pose.orientation.w = q[0]
        odom.pose.pose.orientation.x = q[1]
        odom.pose.pose.orientation.y = q[2]
        odom.pose.pose.orientation.z = q[3]

        # Velocidades (Twist)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        
        self.odom_pub.publish(odom)

    def update_plot(self):
        # Actualizar datos de los objetos de la gráfica
        self.line.set_data(self.history_x, self.history_y)
        self.robot_dot.set_data([self.X], [self.Y])
        
        # Ajuste dinámico de los ejes para seguir al robot
        if len(self.history_x) > 0:
            self.ax.set_xlim(min(self.history_x) - 0.5, max(self.history_x) + 0.5)
            self.ax.set_ylim(min(self.history_y) - 0.5, max(self.history_y) + 0.5)
        
        # Refrescar lienzo de dibujo
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = LocalisationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cerrar Matplotlib correctamente al terminar
        plt.ioff()
        plt.show()
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
   main()
