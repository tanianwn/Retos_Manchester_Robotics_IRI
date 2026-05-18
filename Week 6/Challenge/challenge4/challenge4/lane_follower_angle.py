#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import math

IMAGE_WIDTH     = 640
IMAGE_HEIGHT    = 480

# TRAPECIO
ROI_TOP_Y      = 0.70  # Altura inicio (0.0 arriba, 1.0 abajo)
ROI_BOTTOM_Y   = 1.0  # Altura fin
TRAP_TOP_WIDTH = 0.85  # Ancho superior (fracción)
TRAP_BOT_WIDTH = 1.25  # Ancho base (fracción)

LOOKAHEAD_FRAC  = 0.5
LEFT_THRESHOLD   = 0.38
RIGHT_THRESHOLD  = 0.62
HALF_LANE_WIDTH_PX = 155

KP = 0.015
KD = 0.0065
ANGULAR_MAX  = 2.0
LINEAR_SPEED         = 0.13
LINEAR_SPEED_TURNING = 0.10
TURNING_THRESHOLD_PX = 60
EMA_ALPHA = 0.55

CANNY_LOW       = 50
CANNY_HIGH      = 200
HOUGH_THRESHOLD      = 35
HOUGH_MIN_LINE_LEN   = 25
HOUGH_MAX_LINE_GAP   = 80
ANGLE_FILTER_MIN = 20
ANGLE_FILTER_MAX = 160

class LaneFollowerAngle(Node):

    def __init__(self):
        super().__init__('lane_follower_angle')
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
        self.roi_pub   = self.create_publisher(Image, '/puzzlebot/camera/roi', 10)
        self.canny_pub = self.create_publisher(Image, '/puzzlebot/camera/canny', 10)
        self.debug_pub = self.create_publisher(Image, '/puzzlebot/camera/debug', 10)

        # Estado
        self.prev_error = 0.0
        self.ema_error  = 0.0
        self.last_valid_error = 0.0
        
        self.get_logger().info('Nuevo nodo LaneFollower iniciado.')

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
            h, w = frame.shape[:2]
            
            # MÁSCARA TRAPEZOIDE
            mask = np.zeros((h, w), dtype=np.uint8)
            y_top = int(h * ROI_TOP_Y)
            y_bot = int(h * ROI_BOTTOM_Y)
            x_top_offset = int(w * (1 - TRAP_TOP_WIDTH) / 2)
            x_bot_offset = int(w * (1 - TRAP_BOT_WIDTH) / 2)
            
            pts = np.array([
                [x_top_offset, y_top],
                [w - x_top_offset, y_top],
                [w - x_bot_offset, y_bot],
                [x_bot_offset, y_bot]
            ], np.int32)
            
            cv2.fillPoly(mask, [pts], 255)
            masked_img = cv2.bitwise_and(frame, frame, mask=mask)
            
            # Recorte para procesar 
            roi = masked_img[y_top:y_bot, :]
            
            # Preparar imagen de debug
            full_debug = frame.copy()
            cv2.polylines(full_debug, [pts], True, (0, 0, 255), 2)

            if roi.size == 0: return

            # PROCESAMIENTO
            gray  = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blur  = cv2.GaussianBlur(gray, (3, 3), 0)
            edges = cv2.Canny(blur, CANNY_LOW, CANNY_HIGH)

            # Deteccion basado en angulos
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, HOUGH_THRESHOLD, 
                                    minLineLength=HOUGH_MIN_LINE_LEN, maxLineGap=HOUGH_MAX_LINE_GAP)

            angulos = []

            if lines is not None:
                for seg in lines:
                    x1, y1, x2, y2 = seg[0]
                    
                    if y2 > y1:
                        x1, y1, x2, y2 = x2, y2, x1, y1
                    
                    angulo_rad = math.atan2(y2 - y1, x2 - x1)
                    angulo_deg = math.degrees(angulo_rad)
                    
                    if ANGLE_FILTER_MIN < abs(angulo_deg) < ANGLE_FILTER_MAX:
                        cv2.line(full_debug, (x1, y1 + y_top), (x2, y2 + y_top), (0, 255, 0), 2)
                        angulos.append(angulo_deg)

            # Calculando error en base a -90 grados como recta
            if angulos:
                angulo_promedio = sum(angulos) / len(angulos)
                # Si el ángulo es -90, el error es 0. 
                current_error = float(angulo_promedio - (-90.0))
                mode = 'TRACK'
                self.last_valid_error = current_error
            else:
                current_error = self.last_valid_error
                mode = 'LOST'

            # PD CONTROL 
            self.ema_error = (EMA_ALPHA * current_error) + ((1.0 - EMA_ALPHA) * self.ema_error)
            derivative = self.ema_error - self.prev_error
            self.prev_error = self.ema_error

            ang_z = float(np.clip(-(KP * self.ema_error + KD * derivative), -ANGULAR_MAX, ANGULAR_MAX))
            lin_x = float(LINEAR_SPEED_TURNING if abs(self.ema_error) > TURNING_THRESHOLD_PX else LINEAR_SPEED)

            # Publicar velocidad
            t_msg = Twist()
            t_msg.linear.x = lin_x
            t_msg.linear.y = 0.0
            t_msg.linear.z = 0.0
            t_msg.angular.x = 0.0
            t_msg.angular.y = 0.0
            t_msg.angular.z = ang_z
            self.cmd_pub.publish(t_msg)

            self.err_pub.publish(Float32(data=current_error))

            # PUBLICAR TÓPICOS DE VIDEO
            cv2.putText(full_debug, f"M: {mode} E: {current_error:.0f}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            timestamp = self.get_clock().now().to_msg()
            
            # ROI
            roi_msg = self.bridge.cv2_to_imgmsg(roi, encoding='bgr8')
            roi_msg.header.stamp = timestamp
            self.roi_pub.publish(roi_msg)
            
            # Canny
            canny_msg = self.bridge.cv2_to_imgmsg(edges, encoding='mono8')
            canny_msg.header.stamp = timestamp
            self.canny_pub.publish(canny_msg)
            
            # Debug
            debug_msg = self.bridge.cv2_to_imgmsg(full_debug, encoding='bgr8')
            debug_msg.header.stamp = timestamp
            self.debug_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Error Logic: {e}')

    def destroy_node(self):
        # Detener el robot al apagar
        stop_msg = Twist()
        self.cmd_pub.publish(stop_msg)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowerAngle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()