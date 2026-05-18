#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import time


IMAGE_WIDTH     = 640
IMAGE_HEIGHT    = 480

#Scanlines
SCANLINES_Y = [390, 410, 430, 450, 475]

#ROI
ROI_X_MIN = 190
ROI_X_MAX = 450

# Ajuste de iluminación
ALPHA_CONTRAST  = 0.5   #Contraste 
BETA_BRIGHTNESS = -40   #Brillo

KP = 0.0015
KP_TURN = 0.004 # kp turn
KI = 0.0000 
KD = 0.0075
ANGULAR_MAX  = 1.0
LINEAR_SPEED         = 0.12
LINEAR_SPEED_TURNING = 0.07
TURNING_THRESHOLD_PX = 60
EMA_ALPHA = 0.75

class LaneFollowerScanline(Node):

    def __init__(self):
        super().__init__('lane_follower_scanline')
        self.bridge = CvBridge()

        # Suscriptor
        self.image_sub = self.create_subscription(
            Image,
            '/video_source/raw',
            self.image_callback,
            10
        )

        # Publicadores
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.err_pub = self.create_publisher(Float32, '/puzzlebot/lane_error', 10)
        self.thresh_pub = self.create_publisher(Image, '/puzzlebot/camera/thresh', 10)
        self.debug_pub = self.create_publisher(Image, '/puzzlebot/camera/debug', 10)

        # Estado
        self.prev_error = 0.0
        self.ema_error  = 0.0
        self.integral_error = 0.0  
        self.last_valid_error = 0.0
        
        self.get_logger().info('Nodo LaneFollower (Scanlines + ROI Visual + PID + Gafas de Sol) iniciado.')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if frame.shape[1] != IMAGE_WIDTH or frame.shape[0] != IMAGE_HEIGHT:
                frame = cv2.resize(frame, (IMAGE_WIDTH, IMAGE_HEIGHT))
            self.process_logic(frame)
        except Exception as e:
            self.get_logger().error(f'Error Image Callback: {e}')

    def process_logic(self, frame):
        try:
            
            frame = cv2.convertScaleAbs(frame, alpha=ALPHA_CONTRAST, beta=BETA_BRIGHTNESS)
            
            image_center = IMAGE_WIDTH // 2
            full_debug = frame.copy()

            #Oscurecer bordes
            overlay = full_debug.copy()
            cv2.rectangle(overlay, (0, 0), (ROI_X_MIN, IMAGE_HEIGHT), (0, 0, 0), -1)
            cv2.rectangle(overlay, (ROI_X_MAX, 0), (IMAGE_WIDTH, IMAGE_HEIGHT), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.6, full_debug, 0.4, 0, full_debug)

            #Area de scanlines
            scan_y_min = SCANLINES_Y[0] - 15
            scan_y_max = SCANLINES_Y[-1] + 15
            cv2.rectangle(full_debug, (ROI_X_MIN, scan_y_min), (ROI_X_MAX, scan_y_max), (255, 0, 255), 2)

            #Preprocesamiento (Gray, Blur y Otsu)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (7, 7), 0)
            _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

            #ROI
            thresh[:, :ROI_X_MIN] = 0
            thresh[:, ROI_X_MAX:] = 0

            errors = []

            #Lógica de scanlines
            for y in SCANLINES_Y:
                row_pixels = thresh[y, :]
                white_indices = np.where(row_pixels == 255)[0]

                if len(white_indices) > 0:
                    pLeft_x = white_indices[0]
                    
                    pRight_x = pLeft_x
                    for i in range(pLeft_x, IMAGE_WIDTH):
                        if row_pixels[i] == 0:
                            pRight_x = i - 1
                            break
                    else:
                        pRight_x = IMAGE_WIDTH - 1

                    line_center_x = (pLeft_x + pRight_x) / 2.0
                    error = line_center_x - image_center
                    errors.append(error)

                    cv2.circle(full_debug, (int(pLeft_x), y), 5, (0, 255, 0), -1) 
                    cv2.circle(full_debug, (int(pRight_x), y), 5, (255, 0, 0), -1) 
                    cv2.circle(full_debug, (int(line_center_x), y), 5, (0, 0, 255), -1) 

                #Línea amarilla de escaneo
                cv2.line(full_debug, (ROI_X_MIN, y), (ROI_X_MAX, y), (0, 255, 255), 1)

            #Control
            if errors:
                current_error = sum(errors) / len(errors)
                mode = 'TRACK'
                self.last_valid_error = current_error
            else:
                current_error = self.last_valid_error
                mode = 'LOST'

            cv2.line(full_debug, (image_center, 0), (image_center, IMAGE_HEIGHT), (255, 255, 255), 2)

            # CONTROL PID
            self.ema_error = (EMA_ALPHA * current_error) + ((1.0 - EMA_ALPHA) * self.ema_error)
            
            # Cálculo del término Integral con límite anti-windup
            self.integral_error += self.ema_error
            self.integral_error = float(np.clip(self.integral_error, -2000.0, 2000.0))

            derivative = self.ema_error - self.prev_error
            self.prev_error = self.ema_error

            active_kp = KP_TURN if abs(self.ema_error) >= 80 else KP # kp turn

            ang_z = float(np.clip(-(active_kp * self.ema_error + KI * self.integral_error + KD * derivative), -ANGULAR_MAX, ANGULAR_MAX)) # kp turn
            lin_x = float(LINEAR_SPEED_TURNING if abs(self.ema_error) > TURNING_THRESHOLD_PX else LINEAR_SPEED)

            # Publicar velocidad
            t_msg = Twist()
            t_msg.linear.x = lin_x
            t_msg.angular.z = ang_z
            self.cmd_pub.publish(t_msg)

            self.err_pub.publish(Float32(data=current_error))

            # PUBLICAR TÓPICOS DE VIDEO
            cv2.putText(full_debug, f"M: {mode} E: {current_error:.0f}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            timestamp = self.get_clock().now().to_msg()
            
            thresh_msg = self.bridge.cv2_to_imgmsg(thresh, encoding='mono8')
            thresh_msg.header.stamp = timestamp
            self.thresh_pub.publish(thresh_msg)
            
            debug_msg = self.bridge.cv2_to_imgmsg(full_debug, encoding='bgr8')
            debug_msg.header.stamp = timestamp
            self.debug_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Error Logic: {e}')

    def destroy_node(self):
        self.get_logger().info('Apagando motores antes de salir...')
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_pub.publish(stop_msg)
        time.sleep(0.2) 
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowerScanline()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
