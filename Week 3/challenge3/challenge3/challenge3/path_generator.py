import rclpy
from rclpy.node import Node
from challenge3_interfaces.msg import GoalList
from nav_msgs.msg import Odometry
import numpy as np

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        self.goal_pub = self.create_publisher(GoalList, '/goals', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        # --- Configuración de Trayectoria Dinámica ---
        self.targets = self.solicitar_puntos()
        self.current_idx = 0
        self.mission_finished = False
        
        self.get_logger().info(f"Trayectoria cargada con {len(self.targets)} puntos.")

    def solicitar_puntos(self):
        """Permite al usuario ingresar puntos por la terminal."""
        puntos = []
        print("-" * 30)
        print("CONFIGURACIÓN DE TRAYECTORIA")
        try:
            n = int(input("¿Cuántos puntos tendrá la trayectoria?: "))
            for i in range(n):
                print(f"\nPunto {i+1}:")
                x = float(input("  Coordenada X: "))
                y = float(input("  Coordenada Y: "))
                puntos.append([x, y])
        except ValueError:
            self.get_logger().error("Entrada inválida. Usando trayectoria por defecto [0,0].")
            puntos = [[0.0, 0.0]]
        
        print("-" * 30)
        return puntos

    def odom_cb(self, odom):
        # Obtener posición actual del robot
        curr_x = odom.pose.pose.position.x
        curr_y = odom.pose.pose.position.y
        target = self.targets[self.current_idx]
        
        # Calcular distancia euclidiana al objetivo actual
        dist = np.sqrt((target[0] - curr_x)**2 + (target[1] - curr_y)**2)
        
        # UMBRAL: 5cm para considerar que se llegó al punto
        if dist < 0.05 and not self.mission_finished: 
            if self.current_idx < len(self.targets) - 1:
                self.current_idx += 1
                self.get_logger().info(f"Objetivo intermedio alcanzado. Moviendo al Punto {self.current_idx + 1}")
            else:
                if not self.mission_finished:
                    self.mission_finished = True
                    self.get_logger().info("¡Todos los objetivos enviados! Esperando llegada al punto final.")

        # Publicar el mensaje personalizado GoalList
        msg = GoalList()
        msg.current_goal.position.x = float(self.targets[self.current_idx][0])
        msg.current_goal.position.y = float(self.targets[self.current_idx][1])
        msg.is_reachable = True 
        msg.final_goal = self.mission_finished
        self.goal_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    # El nodo se inicializa y pedirá los datos antes de hacer el spin
    node = PathGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
