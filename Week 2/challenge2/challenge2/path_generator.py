import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Point
from challenge2_interfaces.msg import Trajectories
import matplotlib.pyplot as plt

def wrap_to_pi(theta):
    return (theta + math.pi) % (2 * math.pi) - math.pi

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        self.publisher_ = self.create_publisher(Trajectories, '/pose', 10)
        
        #Declaramos los parámetros por si el interruptor está en True
        self.declare_parameter('path_x', [1.0, 1.0, 0.0, 0.0])
        self.declare_parameter('path_y', [0.0, 1.0, 1.0, 0.0])
        self.declare_parameter('mode', 'v')           
        self.declare_parameter('target_time', 20.0)   
        self.declare_parameter('linear_vel', 0.3)     
        self.declare_parameter('angular_vel', 0.5)    

        self.get_logger().info("Path generator node")
        self.create_timer(1.0, self.capturar_trayectoria)

    def capturar_trayectoria(self):
        msg = Trajectories()
        MAX_LINEAR_V = 0.7   
        MAX_ANGULAR_V = 0.5  

        USAR_ARCHIVO_YAML = True

        if not USAR_ARCHIVO_YAML:
            #Generar trayectoria a partir de inputs en terminal
            try:
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

                cur_x, cur_y, cur_theta = 0.0, 0.0, 0.0
                total_dist = 0.0
                total_angle = 0.0

                for p in points_list:
                    dx = p.x - cur_x
                    dy = p.y - cur_y
                    dist_tramo = math.hypot(dx, dy)
                    target_angle = math.atan2(dy, dx)
                    giro_tramo = abs(wrap_to_pi(target_angle - cur_theta))
                    
                    total_dist += dist_tramo
                    total_angle += giro_tramo
                    cur_x, cur_y, cur_theta = p.x, p.y, target_angle

                print(f"\nDistancia total: {total_dist:.2f}m | Giro total: {total_angle:.2f} rad")
                while True:
                    opcion = input("¿Enviar por Tiempo (t) o Velocidad (v)?: ").lower()
                    
                    if opcion == 't':
                        t_user = float(input("Introduce el tiempo TOTAL deseado (segundos): "))
                        t_rotacion = total_angle / MAX_ANGULAR_V
                        if t_rotacion >= t_user:
                            print(f"ERROR: Solo girar toma {t_rotacion:.2f}s. El tiempo es muy corto.")
                            continue
                        t_lineal_restante = t_user - t_rotacion
                        v_lineal_calc = total_dist / t_lineal_restante
                        if v_lineal_calc > MAX_LINEAR_V:
                            print(f"ERROR: Se requiere {v_lineal_calc:.2f} m/s. Excede límite.")
                            continue
                        msg.linear_velocity = v_lineal_calc
                        msg.angular_velocity = MAX_ANGULAR_V
                        break

                    elif opcion == 'v':
                        v_lin = float(input(f"Introduce velocidad lineal (máx {MAX_LINEAR_V}): "))
                        v_ang = float(input(f"Introduce velocidad angular (máx {MAX_ANGULAR_V}): "))
                        if v_lin > MAX_LINEAR_V or v_ang > MAX_ANGULAR_V:
                            print("Velocidades exceden límites permitidos.")
                            continue
                        t_estimado = (total_angle / v_ang) + (total_dist / v_lin)
                        print(f"El robot tardará aprox {t_estimado:.2f}s en total.")
                        msg.linear_velocity = v_lin
                        msg.angular_velocity = v_ang
                        break

            except ValueError:
                self.get_logger().error("Entrada no numérica.")
                raise SystemExit

        else:
            #Generar trayectoria a partir de archivo 
            self.get_logger().info("Leyendo trayectoria desde archivo de parámetros YAML")
            path_x = self.get_parameter('path_x').value
            path_y = self.get_parameter('path_y').value
            opcion = self.get_parameter('mode').value.lower()
            
            if len(path_x) != len(path_y) or len(path_x) == 0:
                self.get_logger().error("Error en las listas 'path_x' y 'path_y'.")
                raise SystemExit

            points_list = []
            for x, y in zip(path_x, path_y):
                p = Point()
                p.x, p.y, p.z = float(x), float(y), 0.0
                points_list.append(p)
                msg.points.append(p)

            cur_x, cur_y, cur_theta = 0.0, 0.0, 0.0
            total_dist = 0.0
            total_angle = 0.0

            for p in points_list:
                dx = p.x - cur_x
                dy = p.y - cur_y
                dist_tramo = math.hypot(dx, dy)
                target_angle = math.atan2(dy, dx)
                total_dist += dist_tramo
                total_angle += abs(wrap_to_pi(target_angle - cur_theta))
                cur_x, cur_y, cur_theta = p.x, p.y, target_angle

            self.get_logger().info(f"Distancia total: {total_dist:.2f}m | Giro total: {total_angle:.2f} rad")

            if opcion == 't':
                t_user = self.get_parameter('target_time').value
                t_rotacion = total_angle / MAX_ANGULAR_V
                if t_rotacion >= t_user:
                    self.get_logger().error("Tiempo insuficiente para girar.")
                    raise SystemExit
                v_lineal_calc = total_dist / (t_user - t_rotacion)
                if v_lineal_calc > MAX_LINEAR_V:
                    self.get_logger().error(f"Se excede límite de velocidad lineal ({v_lineal_calc:.2f} m/s).")
                    raise SystemExit
                msg.linear_velocity = v_lineal_calc
                msg.angular_velocity = MAX_ANGULAR_V

            elif opcion == 'v':
                v_lin = self.get_parameter('linear_vel').value
                v_ang = self.get_parameter('angular_vel').value
                if v_lin > MAX_LINEAR_V or v_ang > MAX_ANGULAR_V:
                    self.get_logger().error("Velocidades exceden límites permitidos.")
                    raise SystemExit
                msg.linear_velocity = v_lin
                msg.angular_velocity = v_ang

            else:
                self.get_logger().error("Modo inválido en YAML. Usa 't' o 'v'.")
                raise SystemExit

        #Gráfica
        self.get_logger().info(f"Puntos a procesar: {[(p.x, p.y) for p in points_list]}")
        
        x_vals = [0.0] + [p.x for p in points_list]
        y_vals = [0.0] + [p.y for p in points_list]

        plt.figure(figsize=(8, 6))
        plt.plot(x_vals, y_vals, 'b--', label='Trayectoria planeada')
        plt.scatter(x_vals, y_vals, c='red', marker='o', label='Waypoints')
        plt.scatter(0, 0, c='green', marker='s', s=100, label='Inicio (0,0)')
        
        plt.title(f"Visualización de Trayectoria - Modo: {opcion.upper()}")
        plt.xlabel("X (metros)")
        plt.ylabel("Y (metros)")
        plt.grid(True)
        plt.legend()
        plt.axis('equal')
        
        print("\nCierra la ventana de la gráfica para enviar la trayectoria al robot")
        plt.show() 
        
        self.publisher_.publish(msg)
        self.get_logger().info("Trayectoria validada y enviada al controlador.")
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

if __name__ == '__main__':
    main()
