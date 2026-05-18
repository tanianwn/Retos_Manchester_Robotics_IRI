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

# Motores
DISABLE_MOTORS  = True  

IMAGE_WIDTH     = 640
IMAGE_HEIGHT    = 480

# Ajustes del trapecio
ROI_TOP_Y      = 0.75  
ROI_BOTTOM_Y   = 1.0   
TRAP_TOP_WIDTH = 0.85  
TRAP_BOT_WIDTH = 1.25  

LOOKAHEAD_FRAC  = 0.4
LEFT_THRESHOLD   = 0.38
RIGHT_THRESHOLD  = 0.62
HALF_LANE_WIDTH_PX = 155

KP = 0.014
KD = 0.0067
ANGULAR_MAX  = 2.0
LINEAR_SPEED         = 0.13
LINEAR_SPEED_TURNING = 0.10
TURNING_THRESHOLD_PX = 60
EMA_ALPHA = 0.55

# Ajustes de Hough
HOUGH_THRESHOLD      = 50  
HOUGH_MIN_LINE_LEN   = 45  
HOUGH_MAX_LINE_GAP   = 30  
ANGLE_FILTER_MIN = 20
ANGLE_FILTER_MAX = 160


class LaneFollowerPuzzlebot(Node):

    def __init__(self):
        super().__init__('lane_follower_puzzlebot')
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
        self.otsu_pub  = self.create_publisher(Image, '/puzzlebot/camera/canny', 10) 
        self.debug_pub = self.create_publisher(Image, '/puzzlebot/camera/debug', 10)

        # Estado
        self.prev_error = 0.0
        self.ema_error  = 0.0
        self.last_valid_error = 0.0
        
        estado_motor = "APAGADOS" if DISABLE_MOTORS else "ENCENDIDOS"
        self.get_logger().info(f'Nodo LaneFollower (Trapecio + OTSU Mejorado) iniciado. Motores: {estado_motor}')

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
            
            # Máscara trapezoide
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
            
            # Imagen de debug
            full_debug = frame.copy()
            cv2.polylines(full_debug, [pts], True, (0, 0, 255), 2)

            if roi.size == 0: return

            roi_h = roi.shape[0]
            lookahead_row = int(roi_h * LOOKAHEAD_FRAC)

            # Preprocesamiento
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (9, 9), 0) 
            otsu_val, binary_otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            
            # Limpieza morfológica
            kernel = np.ones((5,5), np.uint8)
            binary_otsu = cv2.morphologyEx(binary_otsu, cv2.MORPH_OPEN, kernel)
            binary_otsu = cv2.morphologyEx(binary_otsu, cv2.MORPH_CLOSE, kernel)

            edges = cv2.Canny(binary_otsu, 50, 150)

            # Detección de líneas
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, HOUGH_THRESHOLD, 
                                    minLineLength=HOUGH_MIN_LINE_LEN, maxLineGap=HOUGH_MAX_LINE_GAP)

            left_xs, right_xs = [], []

            if lines is not None:
                for seg in lines:
                    x1, y1, x2, y2 = seg[0]
                    angle = math.degrees(math.atan2(abs(y2 - y1), abs(x2 - x1) + 1e-6))
                    
                    if ANGLE_FILTER_MIN < angle < ANGLE_FILTER_MAX:
                        cv2.line(full_debug, (x1, y1 + y_top), (x2, y2 + y_top), (0, 255, 0), 2)
                        x_proj = self._project_to_row(x1, y1, x2, y2, lookahead_row)
                        if x_proj is not None:
                            if (x_proj / w) < LEFT_THRESHOLD: left_xs.append(x_proj)
                            elif (x_proj / w) > RIGHT_THRESHOLD: right_xs.append(x_proj)

            # Lógica de carril
            cx_image = w / 2.0
            x_left = float(np.mean(left_xs)) if left_xs else None
            x_right = float(np.mean(right_xs)) if right_xs else None
            
            if x_left is not None and x_right is not None:
                lane_x, mode = (x_left + x_right) / 2.0, 'BOTH'
            elif x_left is not None:
                lane_x, mode = x_left + HALF_LANE_WIDTH_PX, 'LEFT'
            elif x_right is not None:
                lane_x, mode = x_right - HALF_LANE_WIDTH_PX, 'RIGHT'
            else:
                lane_x, mode = cx_image + self.last_valid_error, 'LOST'

            pixel_error = float(lane_x - cx_image)
            if mode != 'LOST': self.last_valid_error = pixel_error

            # CONTROL PD
            self.ema_error = (EMA_ALPHA * pixel_error) + ((1.0 - EMA_ALPHA) * self.ema_error)
            derivative = self.ema_error - self.prev_error
            self.prev_error = self.ema_error

            ang_z = float(np.clip(-(KP * self.ema_error + KD * derivative), -ANGULAR_MAX, ANGULAR_MAX))
            lin_x = float(LINEAR_SPEED_TURNING if abs(self.ema_error) > TURNING_THRESHOLD_PX else LINEAR_SPEED)

            # Publicar velocidad
            t_msg = Twist()
            
            if DISABLE_MOTORS:
                t_msg.linear.x = 0.0
                t_msg.angular.z = 0.0
                cv2.putText(full_debug, "MOTORS DISABLED", (180, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            else:
                t_msg.linear.x = lin_x
                t_msg.angular.z = ang_z

            t_msg.linear.y = 0.0
            t_msg.linear.z = 0.0
            t_msg.angular.x = 0.0
            t_msg.angular.y = 0.0
            
            self.cmd_pub.publish(t_msg)
            self.err_pub.publish(Float32(data=pixel_error))

            # PUBLICAR TÓPICOS DE VIDEO
            cv2.circle(full_debug, (int(lane_x), y_top + lookahead_row), 12, (0, 255, 255), -1)
            cv2.putText(full_debug, f"M: {mode} E: {pixel_error:.0f} OTSU: {otsu_val:.0f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            timestamp = self.get_clock().now().to_msg()
            
            thresh_msg = self.bridge.cv2_to_imgmsg(binary_otsu, encoding='mono8')
            thresh_msg.header.stamp = timestamp
            self.otsu_pub.publish(thresh_msg)
            
            debug_msg = self.bridge.cv2_to_imgmsg(full_debug, encoding='bgr8')
            debug_msg.header.stamp = timestamp
            self.debug_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Error Logic: {e}')

    def _project_to_row(self, x1, y1, x2, y2, target_row):
        if y1 == y2: return None
        t = (target_row - y1) / (y2 - y1)
        x_proj = x1 + t * (x2 - x1)
        return x_proj if 0 <= x_proj <= IMAGE_WIDTH else None

    def destroy_node(self):
        stop_msg = Twist()
        self.cmd_pub.publish(stop_msg)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowerPuzzlebot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()