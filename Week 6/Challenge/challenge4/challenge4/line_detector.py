#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

#Configuración de Imagen y ROI
IMAGE_WIDTH, IMAGE_HEIGHT = 640, 480
SCANLINES_Y = [390, 410, 430, 450, 475]
ROI_X_MIN, ROI_X_MAX = 190, 450
ALPHA_CONTRAST, BETA_BRIGHTNESS = 0.5, -40

class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector')
        self.bridge = CvBridge()
        
        # Suscriptor de cámara
        self.image_sub = self.create_subscription(Image, '/video_source/raw', self.image_callback, 10)
        
        # Publicadores
        self.err_pub = self.create_publisher(Float32, '/puzzlebot/line_error', 10)
        self.thresh_pub = self.create_publisher(Image, '/puzzlebot/camera/thresh', 10)
        self.debug_pub = self.create_publisher(Image, '/puzzlebot/camera/debug', 10)
        
        self.get_logger().info('Nodo Line Detector (Percepción) iniciado.')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if frame.shape[1] != IMAGE_WIDTH:
                frame = cv2.resize(frame, (IMAGE_WIDTH, IMAGE_HEIGHT))
            
            # Preprocesamiento
            frame = cv2.convertScaleAbs(frame, alpha=ALPHA_CONTRAST, beta=BETA_BRIGHTNESS)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (7, 7), 0)
            _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

            #ROI 
            thresh[:, :ROI_X_MIN] = 0
            thresh[:, ROI_X_MAX:] = 0

            #Scanlines
            errors = []
            image_center = IMAGE_WIDTH // 2
            debug_img = frame.copy()

            for y in SCANLINES_Y:
                row = thresh[y, :]
                white_px = np.where(row == 255)[0]
                if len(white_px) > 0:
                    p_left, p_right = white_px[0], white_px[-1]
                    line_center = (p_left + p_right) / 2.0
                    errors.append(line_center - image_center)
                    cv2.circle(debug_img, (int(line_center), y), 5, (0, 0, 255), -1)

            #Publicar Error
            if errors:
                avg_error = sum(errors) / len(errors)
                self.err_pub.publish(Float32(data=float(avg_error)))
            
            #Publicar visualización
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8'))
            self.thresh_pub.publish(self.bridge.cv2_to_imgmsg(thresh, encoding='mono8'))

        except Exception as e:
            self.get_logger().error(f'Error en Detector: {e}')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LineDetector())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
