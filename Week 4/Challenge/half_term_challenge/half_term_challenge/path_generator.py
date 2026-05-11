import rclpy
from rclpy.node import Node
from half_term_challenge_interfaces.msg import GoalList
from nav_msgs.msg import Odometry
import numpy as np
import sys

class PathGenerator(Node):
    def __init__(self, targets):
        super().__init__('path_generator')
        
        # Publicador para enviar los objetivos al controlador
        self.goal_pub = self.create_publisher(GoalList, '/goals', 10)
        
        # Suscriptor a odometría para saber cuándo pasar al siguiente punto
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        # --- Configuración de Trayectoria ---
        self.targets = targets
        self.current_idx = 0
        self.mission_finished = False
        
        self.get_logger().info(f"Nodo iniciado. Trayectoria cargada con {len(self.targets)} puntos.")

    def odom_cb(self, odom):
        # Obtener posición actual del robot desde la odometría
        curr_x = odom.pose.pose.position.x
        curr_y = odom.pose.pose.position.y
        
        target = self.targets[self.current_idx]
        
        # Calcular distancia euclidiana al objetivo actual
        dist = np.sqrt((target[0] - curr_x)**2 + (target[1] - curr_y)**2)
        
        # UMBRAL: 5cm para considerar que se llegó al punto y cambiar al siguiente
        if dist < 0.05 and not self.mission_finished: 
            if self.current_idx < len(self.targets) - 1:
                self.current_idx += 1
                self.get_logger().info(f"Punto {self.current_idx} alcanzado. Moviendo al Punto {self.current_idx + 1}: {self.targets[self.current_idx]}")
            else:
                if not self.mission_finished:
                    self.mission_finished = True
                    self.get_logger().info("¡Misión completada! Se alcanzó el último punto de la trayectoria.")

        # Publicar el mensaje personalizado GoalList
        msg = GoalList()
        msg.current_goal.position.x = float(self.targets[self.current_idx][0])
        msg.current_goal.position.y = float(self.targets[self.current_idx][1])
        msg.is_reachable = True 
        msg.final_goal = self.mission_finished
        
        self.goal_pub.publish(msg)

def solicitar_puntos():
    """Función para capturar la entrada del usuario por terminal antes de arrancar ROS"""
    puntos = []
    print("\n" + "="*40)
    print("      CONFIGURADOR DE TRAYECTORIA")
    print("="*40)
    
    try:
        entrada_n = input("¿Cuántos puntos tendrá la trayectoria?: ")
        n = int(entrada_n)
        
        for i in range(n):
            print(f"\n--- Configurando Punto {i+1} ---")
            x = float(input(f"  Coordenada X para Punto {i+1}: "))
            y = float(input(f"  Coordenada Y para Punto {i+1}: "))
            puntos.append([x, y])
            
    except (ValueError, EOFError):
        print("\n[ERROR] Entrada inválida detectada.")
        print("Cargando trayectoria de seguridad: [0.0, 0.0]")
        puntos = [[0.0, 0.0]]
        
    print("\n" + "="*40)
    print("Iniciando nodo... Por favor regrese a la terminal principal.")
    print("="*40 + "\n")
    return puntos

def main(args=None):
    rclpy.init(args=args)
    
    # 1. Pedimos los puntos
    puntos_mision = solicitar_puntos()
    
    # 2. Iniciamos el nodo pasándole los puntos capturados
    node = PathGenerator(puntos_mision)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo detenido por el usuario.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
