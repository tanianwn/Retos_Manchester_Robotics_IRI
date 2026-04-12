import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Point
from challenge2_interfaces.msg import Trajectories
import matplotlib.pyplot as plt

# Función auxiliar para normalizar ángulos entre -PI y PI
def wrap_to_pi(theta):
    return (theta + math.pi) % (2 * math.pi) - math.pi

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        self.publisher_ = self.create_publisher(Trajectories, '/pose', 10)
        self.get_logger().info("Nodo Generador (Cerebro) iniciado.")
        self.create_timer(1.0, self.capturar_trayectoria)

    def capturar_trayectoria(self):
        msg = Trajectories()
        MAX_LINEAR_V = 0.7   # m/s
        MAX_ANGULAR_V = 0.5  # rad/s (Velocidad de giro fija)

        try:
            # 1. Entrada de puntos
            n = int(input("\n¿Cuántos puntos tiene la trayectoria?: "))
            points_list = []
            for i in range(n):
                p = Point()
                print(f"--- Punto {i+1} ---")
                p.x = float(input("X: "))
                p.y = float(input("Y: "))
                p.z = 0.0
                points_list.append(p)
                msg.points.append(p)

            # 2. Simulación de la ruta (Cálculo de Distancia y Ángulos Totales)
            cur_x, cur_y, cur_theta = 0.0, 0.0, 0.0
            total_dist = 0.0
            total_angle = 0.0

            for p in points_list:
                dx = p.x - cur_x
                dy = p.y - cur_y
                dist_tramo = math.hypot(dx, dy)
                target_angle = math.atan2(dy, dx)
                # Sumamos el valor absoluto del giro necesario
                giro_tramo = abs(wrap_to_pi(target_angle - cur_theta))
                
                total_dist += dist_tramo
                total_angle += giro_tramo
                
                # Actualizamos pose simulada
                cur_x, cur_y, cur_theta = p.x, p.y, target_angle

            # 3. Elección de modo
            print(f"\nDistancia total: {total_dist:.2f}m | Giro total: {total_angle:.2f} rad")
            while True:
                opcion = input("¿Enviar por Tiempo (t) o Velocidad (v)?: ").lower()
                
                if opcion == 't':
                    t_user = float(input("Introduce el tiempo TOTAL deseado (segundos): "))
                    
                    # PASO 1: Tiempo de rotación (usando velocidad fija)
                    t_rotacion = total_angle / MAX_ANGULAR_V
                    
                    if t_rotacion >= t_user:
                        print(f"ERROR: Solo girar toma {t_rotacion:.2f}s. El tiempo es muy corto.")
                        continue
                    
                    # PASO 2: Calcular velocidad lineal necesaria
                    t_lineal_restante = t_user - t_rotacion
                    v_lineal_calc = total_dist / t_lineal_restante
                    
                    if v_lineal_calc > MAX_LINEAR_V:
                        print(f"ERROR: Se requiere {v_lineal_calc:.2f} m/s para cumplir. Excede el límite de {MAX_LINEAR_V}.")
                        continue
                    
                    msg.linear_velocity = v_lineal_calc
                    msg.angular_velocity = MAX_ANGULAR_V
                    break

                elif opcion == 'v':
                    v_lin = float(input(f"Introduce velocidad lineal (máx {MAX_LINEAR_V}): "))
                    v_ang = float(input(f"Introduce velocidad angular (máx {MAX_ANGULAR_V}): "))
                    
                    if v_lin > MAX_LINEAR_V or v_ang > MAX_ANGULAR_V:
                        print("Alguna velocidad excede los límites permitidos.")
                        continue
                    
                    # Cálculo de tiempo total para información del usuario
                    t_estimado = (total_angle / v_ang) + (total_dist / v_lin)
                    print(f"El robot tardará aproximadamente {t_estimado:.2f}s en total.")
                    
                    msg.linear_velocity = v_lin
                    msg.angular_velocity = v_ang
                    break

            # 4. Publicar
            self.get_logger().info(f"Puntos seleccionados: {[(p.x, p.y) for p in points_list]}")
            # --------------------
            # --- GENERAR GRÁFICA DE LA TRAYECTORIA ---
            x_vals = [0.0] + [p.x for p in points_list]
            y_vals = [0.0] + [p.y for p in points_list]

            plt.figure(figsize=(8, 6))
            # Dibujar la línea de la trayectoria
            plt.plot(x_vals, y_vals, 'b--', label='Trayectoria planeada')
            # Dibujar los puntos (Waypoints)
            plt.scatter(x_vals, y_vals, c='red', marker='o', label='Waypoints')
            # Marcar el inicio
            plt.scatter(0, 0, c='green', marker='s', s=100, label='Inicio (0,0)')
            
            # Etiquetas y formato
            plt.title(f"Visualización de Trayectoria - Modo: {opcion.upper()}")
            plt.xlabel("X (metros)")
            plt.ylabel("Y (metros)")
            plt.grid(True)
            plt.legend()
            plt.axis('equal') # Para que el cuadrado se vea como cuadrado
            
            print("\nCierra la ventana de la gráfica para enviar la trayectoria al robot...")
            plt.show() 
            # ------------------------------------------
            #---------------------
            self.publisher_.publish(msg)
            self.get_logger().info("Trayectoria validada y enviada al controlador.")
            
            
        except ValueError:
            self.get_logger().error("Entrada no numérica.")
        
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
