#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

# Configuración de imagen
IMAGE_WIDTH  = 640
IMAGE_HEIGHT = 480

# Límites de la ROI adaptados
INTERSECTION_ROI_X_MIN = 80   
INTERSECTION_ROI_X_MAX = 560  
SCANLINES_INTERSECTION_Y = [380, 410, 440, 470] #se bajaron 20 pixeles, regresar si no funciona

# Filtrado de ruido
ALPHA_CONTRAST   = 0.4
BETA_BRIGHTNESS  = -30
MIN_LINE_WIDTH   = 12    
MAX_LINE_WIDTH   = 110   

class IntersectionDetector(Node):
    def __init__(self):
        super().__init__('intersection_detector')
        self.bridge = CvBridge()
        
        # Suscriptor de la cámara
        self.image_sub = self.create_subscription(
            Image, 
            '/video_source/raw', 
            self.image_callback, 
            10
        )
        
        # Publicadores
        self.intersection_pub = self.create_publisher(Bool, '/puzzlebot/intersection', 10)
        self.inter_debug_pub = self.create_publisher(Image, '/puzzlebot/camera/intersection_debug', 10)
        
        self.get_logger().info('Nodo Intersection Detector inicializado')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if frame.shape[1] != IMAGE_WIDTH or frame.shape[0] != IMAGE_HEIGHT:
                frame = cv2.resize(frame, (IMAGE_WIDTH, IMAGE_HEIGHT))
            
            # 1. Preprocesamiento de alto contraste y binarización
            frame_enhanced = cv2.convertScaleAbs(frame, alpha=ALPHA_CONTRAST, beta=BETA_BRIGHTNESS)
            gray = cv2.cvtColor(frame_enhanced, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (9,9), 0)
            _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            
            kernel = np.ones((5,5), np.uint8)
            thresh = cv2.erode(thresh, kernel, iterations=6)

            # 2. Aplicar Máscara ROI Lateral
            thresh_inter = thresh.copy()
            thresh_inter[:, :INTERSECTION_ROI_X_MIN] = 0
            thresh_inter[:, INTERSECTION_ROI_X_MAX:] = 0

            # Convertir a BGR para pintar las ayudas visuales a color
            binary_debug = cv2.cvtColor(thresh_inter, cv2.COLOR_GRAY2BGR)

            scanlines_confirming_intersection = 0

            # 3. Escaneo por Scanlines
            for y_int in SCANLINES_INTERSECTION_Y:
                row_pixels = thresh_inter[y_int, :]
                inter_blocks = []
                
                x = INTERSECTION_ROI_X_MIN
                while x < INTERSECTION_ROI_X_MAX:
                    if row_pixels[x] == 255:  
                        start_x = x
                        end_x = start_x
                        while end_x < INTERSECTION_ROI_X_MAX and row_pixels[end_x] == 255:
                            end_x += 1
                        end_x -= 1
                        
                        width = end_x - start_x + 1
                        if MIN_LINE_WIDTH <= width <= MAX_LINE_WIDTH:
                            inter_blocks.append((start_x, end_x))
                        x = end_x + 1
                    else:
                        x += 1

                    if len(inter_blocks) >= 4:
                        break

                # Evaluación estricta de 4 blobs por fila
                if len(inter_blocks) >= 4:
                    scanlines_confirming_intersection += 1
                    cv2.line(binary_debug, (INTERSECTION_ROI_X_MIN, y_int), (INTERSECTION_ROI_X_MAX, y_int), (255, 0, 255), 2)
                    for b in inter_blocks:
                        cv2.circle(binary_debug, (int(b[0]), y_int), 5, (0, 255, 0), -1)
                        cv2.circle(binary_debug, (int(b[1]), y_int), 5, (0, 0, 255), -1)
                else:
                    cv2.line(binary_debug, (INTERSECTION_ROI_X_MIN, y_int), (INTERSECTION_ROI_X_MAX, y_int), (255, 255, 0), 1)

            # 4. Criterio de Activación por Fila Única (Estrategia de Diversidad Espacial)
            intersection_detected = (scanlines_confirming_intersection >= 1)

            # Publicar resultado booleano
            self.intersection_pub.publish(Bool(data=intersection_detected))

            # Logger en terminal
            if intersection_detected:
                self.get_logger().warn(f"[INTERSECCIÓN DETECTADA] Enviando True ({scanlines_confirming_intersection} líneas válidas).")

            # 5. Capa visual HUD para el Debug
            mode = 'INTERSECTION' if intersection_detected else 'NORMAL'
            cv2.putText(binary_debug, f"MODE: {mode} | Match Rows: {scanlines_confirming_intersection}", (15, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            image_center = IMAGE_WIDTH // 2
            cv2.line(binary_debug, (image_center, 0), (image_center, IMAGE_HEIGHT), (100, 100, 100), 1)
            
            # Publicar la imagen binarizada modificada al tópico de debug
            timestamp = self.get_clock().now().to_msg()
            debug_msg = self.bridge.cv2_to_imgmsg(binary_debug, encoding='bgr8')
            debug_msg.header.stamp = timestamp
            self.inter_debug_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Error crítico en Callback de Intersección: {e}')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(IntersectionDetector())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
