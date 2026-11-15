#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String, Bool
import numpy as np

# Configuración PID 
KP, KP_TURN = 0.0007, 0.0034 
KI, KD = 0.0000, 0.0095 
EMA_ALPHA = 0.75
ANGULAR_MAX = 1.0
LINEAR_NORMAL, LINEAR_TURNING = 0.11, 0.07
TURNING_THRESHOLD = 95.0

class LineController(Node):
    def __init__(self):
        super().__init__('line_controller')
        
        # Máquina de estados
        self.state = "SIGUIENDO"
        
        # Seguro de cámara 
        self.camara_lista = False  # El robot no se moverá hasta que esto sea True
        
        # Suscriptores
        self.error_sub = self.create_subscription(Float32, '/puzzlebot/line_error', self.error_cb, 10)
        self.traffic_sub = self.create_subscription(String, '/semaforo_estado', self.traffic_cb, 10)
        self.inter_sub = self.create_subscription(Bool, '/puzzlebot/intersection', self.intersection_cb, 10)
        self.yolo_sub = self.create_subscription(String, '/yolo/detecciones', self.yolo_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        # Publicador
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Bucle a 20Hz
        self.master_timer = self.create_timer(0.05, self.master_control_loop)
        
        # Variables de Memoria
        self.pose = [0.0, 0.0, 0.0]        
        self.start_pose = [0.0, 0.0, 0.0]  
        self.current_line_error = 0.0
        self.prev_error = 0.0
        self.ema_error = 0.0
        self.integral_error = 0.0
        
        self.traffic_state = "N"
        self.waiting_for_green = False 
        self.traffic_speed_mult = 1.0    

        self.wait_start_time = None
        self.maneuver_start_time = None 
        self.last_inter_msg = False  
        self.maneuver_type = None
        
        # Bandera para saber si la maniobra se hace en amarillo
        self.maneuver_is_yellow = False
        
        # Contador de validación del semáforo en intersección
        self.traffic_eval_count = 0

        self.cooldown_active = False
        self.cooldown_timer = None

        self.yolo_signal = "Ninguno"
        self.yolo_speed_mult = 1.0
        self.stop_signal_active = False

        self.get_logger().info('Nodo Activo')
        self.get_logger().info('Esperando datos de la cámara...')

    def wrap_to_pi(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def error_cb(self, msg):
        #Primero se verifica si la cámara está enviando imagen
        if not self.camara_lista:
            self.camara_lista = True
            self.get_logger().info('Cámara conectada y enviando datos')
            
        self.current_line_error = msg.data

    def odom_cb(self, msg):
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.pose[2] = np.arctan2(siny_cosp, cosy_cosp)

    def yolo_cb(self, msg):
        #Ignorar todo si estamos maniobrando
        if self.state == "MANIOBRANDO":
            return

        raw_data = msg.data
        if raw_data == "Ninguno":
            nueva_senal = "Ninguno"
        else:
            primera_deteccion = raw_data.split(" | ")[0]
            nueva_senal = primera_deteccion.split(" ")[0].strip()

        if nueva_senal != "Ninguno" and nueva_senal != self.yolo_signal:
            self.yolo_signal = nueva_senal
            flecha = "➡️" if nueva_senal == "turn_right_ahead" else "⬅️" if nueva_senal == "turn_left_ahead" else "⬆️"
            self.get_logger().info(f'{flecha} [Visión YOLO] Señal: {nueva_senal}')

        if nueva_senal == "stop":
            if not self.stop_signal_active:
                self.stop_signal_active = True
                
                # REQUISITO 2: Prioridad de STOP sobre Rojo Falso
                if self.traffic_state == "R" or self.waiting_for_green:
                    self.traffic_state = "N"
                    self.waiting_for_green = False
                    self.traffic_speed_mult = 1.0
                    
                self.get_logger().warn('Señal STOP detectada')
        else:
            if self.stop_signal_active and nueva_senal != "stop":
                self.stop_signal_active = False

        if nueva_senal in ["roadwork_ahead", "give_way"]:
            self.yolo_speed_mult = 0.4
        else:
            self.yolo_speed_mult = 1.0

    def traffic_cb(self, msg):
        #Ignorar el semáforo si estamos maniobrando
        if self.state == "MANIOBRANDO":
            return

        current_light = msg.data
        
        #Contar publicaciones mientras está en intersección para asegurar validez
        if self.state == "EVALUANDO":
            self.traffic_eval_count += 1
        
        #Ignorar completamente el semáforo rojo si estamos en un STOP
        if current_light == "R" and self.stop_signal_active:
            return

        if current_light == "R":
            if self.traffic_state != "R":
                self.get_logger().warn('Semáforo rojo')
            self.traffic_state = "R"
            self.waiting_for_green = True
            self.traffic_speed_mult = 0.0
        elif current_light == "A":
            if not self.waiting_for_green:
                if self.traffic_state != "A":
                    self.get_logger().warn('Semáforo amarillo')
                self.traffic_state = "A"
                self.traffic_speed_mult = 0.4
        elif current_light == "V":
            if self.traffic_state != "V":
                self.get_logger().warn('Semáforo verde')
            self.traffic_state = "V"
            self.waiting_for_green = False
            self.traffic_speed_mult = 1.0
        elif current_light == "N":
            if not self.waiting_for_green:
                self.traffic_state = "N"
                self.traffic_speed_mult = 1.0

    def intersection_cb(self, msg):
        is_new = msg.data and not self.last_inter_msg
        self.last_inter_msg = msg.data

        if is_new:
            if self.state == "SIGUIENDO" and not self.cooldown_active:
                self.state = "EVALUANDO"
                self.wait_start_time = self.get_clock().now()
                self.traffic_eval_count = 0  # NUEVO: Reiniciamos el contador de seguridad del semáforo
                self.get_logger().warn('1. Intersección detectada, esperando a detección de YOLO')

    def master_control_loop(self):
        t_msg = Twist() 

        #Cortador de corriente por software
        if not self.camara_lista:
            t_msg.linear.x = 0.0
            t_msg.angular.z = 0.0
            self.cmd_pub.publish(t_msg)
            return  # Evita que se ejecute el resto de la lógica hasta estar listos

        if self.state == "EVALUANDO":
            # Asegura que el robot se quede completamente quieto mientras espera
            t_msg.linear.x = 0.0
            t_msg.angular.z = 0.0
            
            elapsed = (self.get_clock().now() - self.wait_start_time).nanoseconds * 1e-9
            
            # Dejamos que asiente 0.5s para perder inercia
            if elapsed >= 0.9:
                # El robot se quedará atorado en este bloque if hasta que detecte algo de la lista
                valid_signal = self.yolo_signal in ["ahead_only", "turn_right_ahead", "turn_left_ahead"]
                
                #Exigimos al menos 6 publicaciones del semáforo para darle la oportunidad de leerlo bien
                if valid_signal and self.traffic_eval_count >= 6:
                    #Detenido en rojo hasta verde
                    if self.traffic_state == "R":
                        pass  # No avanza a MANIOBRANDO mientras siga en Rojo
                    else:
                        senal_final = self.yolo_signal
                        self.get_logger().info(f'2. Señal: {senal_final} | Semáforo: {self.traffic_state}')
                        
                        self.maneuver_type = senal_final
                        self.start_pose = list(self.pose)
                        self.state = "MANIOBRANDO"
                        self.maneuver_start_time = self.get_clock().now()
                        self.yolo_signal = "Ninguno" 
                        
                        #Guardar si se ejecutará en amarillo para reducir velocidad
                        self.maneuver_is_yellow = (self.traffic_state == "A")
                        
                        self.get_logger().info(f'3. Ejecutando Rutina: {self.maneuver_type.upper()}')

        elif self.state == "MANIOBRANDO":
            dx = self.pose[0] - self.start_pose[0]
            dy = self.pose[1] - self.start_pose[1]
            distance = np.sqrt(dx**2 + dy**2)
            delta_theta = abs(self.wrap_to_pi(self.pose[2] - self.start_pose[2]))
            
            elapsed_maneuver = (self.get_clock().now() - self.maneuver_start_time).nanoseconds * 1e-9

            # Condición de escape 1: Intersección detectada
            if elapsed_maneuver > 1.5 and self.last_inter_msg:
                self.get_logger().warn('Siguiente línea detectada, cediendo control al PID.')
                self.terminar_maniobra()
                return
            
            # Condición de escape 2: Timeout de seguridad
            if elapsed_maneuver > 5.0:
                self.get_logger().warn('Timeout de maniobra. Regresando al seguidor.')
                self.terminar_maniobra()
                return

            #Reducir velocidad lineal y angular proporcionalmente si es amarillo
            vel_mult = 0.5 if self.maneuver_is_yellow else 1.0

            if self.maneuver_type in ["ahead_only", "ahead"]:
                if distance >= 0.45:
                    self.terminar_maniobra()
                    return
                t_msg.linear.x = 0.11 * vel_mult 
                t_msg.angular.z = 0.0

            elif self.maneuver_type in ["turn_right_ahead", "right"]:
                if delta_theta >= (np.pi / 2.0):
                    self.terminar_maniobra()
                    return
                t_msg.linear.x = 0.077 * vel_mult
                t_msg.angular.z = -0.28 * vel_mult

            elif self.maneuver_type in ["turn_left_ahead", "left"]:
                if delta_theta >= (np.pi / 2.0):
                    self.terminar_maniobra()
                    return
                t_msg.linear.x = 0.077 * vel_mult
                t_msg.angular.z = 0.28 * vel_mult
            else:
                self.terminar_maniobra()
                return

        elif self.state == "SIGUIENDO":
            if self.stop_signal_active or self.waiting_for_green:
                t_msg.linear.x = 0.0
                t_msg.angular.z = 0.0
            else:
                self.ema_error = (EMA_ALPHA * self.current_line_error) + ((1.0 - EMA_ALPHA) * self.ema_error)
                self.integral_error = float(np.clip(self.integral_error + self.ema_error, -2000, 2000))
                derivative = self.ema_error - self.prev_error
                self.prev_error = self.ema_error

                active_kp = KP_TURN if abs(self.ema_error) >= TURNING_THRESHOLD else KP
                steering = -(active_kp * self.ema_error + KI * self.integral_error + KD * derivative)
                
                combined_speed_mult = self.traffic_speed_mult * self.yolo_speed_mult
                
                t_msg.angular.z = float(np.clip(steering, -ANGULAR_MAX, ANGULAR_MAX) * combined_speed_mult)
                base_linear = LINEAR_TURNING if abs(self.ema_error) > TURNING_THRESHOLD else LINEAR_NORMAL
                t_msg.linear.x = float(base_linear * combined_speed_mult)

        self.cmd_pub.publish(t_msg)

    def terminar_maniobra(self):
        self.state = "SIGUIENDO"
        self.cooldown_active = True
        self.maneuver_is_yellow = False # Reiniciamos estado
        self.get_logger().info('4. Rutina completada. Regresando a Seguidor de Línea.')
        self.cooldown_timer = self.create_timer(2.5, self.end_cooldown)

    def end_cooldown(self):
        if self.cooldown_timer:
            self.cooldown_timer.destroy()
            self.cooldown_timer = None
        self.cooldown_active = False
        self.get_logger().info('Cooldown finalizado. Buscando nuevas intersecciones.')

def main(args=None):
    rclpy.init(args=args)
    node = LineController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupción por teclado.')
    finally:
        msg = Twist()
        node.cmd_pub.publish(msg) 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
