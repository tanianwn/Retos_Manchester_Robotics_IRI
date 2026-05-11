import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np

class TrafficLightNode(Node):

    def __init__(self):
        super().__init__('traffic_light_node')

        self.bridge = CvBridge()

        # 1. CONFIGURACIÓN DEL DETECTOR DE BLOBS
        params = cv.SimpleBlobDetector_Params()
        params.filterByColor = True
        params.blobColor = 255
        params.filterByArea = True
        params.minArea = 210 #190 
        params.maxArea = 50000
        params.filterByCircularity = True
        params.minCircularity = 0.72
        params.filterByConvexity = False
        params.filterByInertia = False
        self.detector = cv.SimpleBlobDetector_create(params)

        # 2. SUSCRIPTOR
        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.camera_callback,
            10
        )

        # 3. PUBLICADORES
        self.state_pub = self.create_publisher(String, '/semaforo_estado', 10)
        self.image_pub = self.create_publisher(Image, '/image_processing/result', 10)

        self.get_logger().info('Nodo de Semáforo iniciado (Detección por Blobs)')

    def camera_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error de conversión: {e}')
            return

        cv_img = cv.convertScaleAbs(cv_img, alpha=1.0, beta=-10)

        # --- PROCESAMIENTO ---
        blurred = cv.GaussianBlur(cv_img, (5, 5), 0)
        hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

        # Máscaras (HSV del DEBUG)
        # Máscaras (HSV del DEBUG)
        
        # El rojo en OpenCV está en los extremos (0-8 y 170-180)
        mask_red = cv.bitwise_or(
            cv.inRange(hsv, np.array([0, 50, 120]), np.array([8, 255, 255])),
            cv.inRange(hsv, np.array([170, 50, 120]), np.array([180, 255, 255]))
        )
        
        # El amarillo en OpenCV suele estar entre 15 y 35
        # Bajamos un poco la Saturación (S) y el Valor (V) mínimos para que no sea tan estricto
        mask_yellow = cv.inRange(
            hsv, np.array([10, 100, 120]), np.array([35, 255, 255])
        )
        
        # El verde suele estar entre 40 y 90 (este lo tenías bien)
        mask_green = cv.inRange(
            hsv, np.array([40, 40, 120]), np.array([90, 255, 255])
        )

        # detección de zona blanca (centro brillante)
        mask_white = cv.inRange(hsv, np.array([0, 0, 240]), np.array([180, 40, 255]))

        kernel_big = np.ones((15, 15), np.uint8)
        white_dilated = cv.dilate(mask_white, kernel_big, iterations=2)

        ring = cv.subtract(white_dilated, mask_white)

        mask_red_ring = cv.bitwise_and(mask_red, ring)
        mask_yellow_ring = cv.bitwise_and(mask_yellow, ring)
        mask_green_ring = cv.bitwise_and(mask_green, ring)

        mask_red = cv.bitwise_or(mask_red, mask_red_ring)
        mask_yellow = cv.bitwise_or(mask_yellow, mask_yellow_ring)
        mask_green = cv.bitwise_or(mask_green, mask_green_ring)

        # Limpieza
        kernel = np.ones((3, 3), np.uint8)
        mask_red = cv.morphologyEx(mask_red, cv.MORPH_OPEN, kernel)
        mask_yellow = cv.morphologyEx(mask_yellow, cv.MORPH_OPEN, kernel)
        mask_green = cv.morphologyEx(mask_green, cv.MORPH_OPEN, kernel)

        # Detección
        kp_red = self.detector.detect(mask_red)
        kp_yellow = self.detector.detect(mask_yellow)
        kp_green = self.detector.detect(mask_green)

        # --- LÓGICA DE ESTADO ---
        letra_estado = "N"
        color_debug = (255, 255, 255)
        tamano_maximo = 0
        kp_ganador = []

        for kp in kp_red:
            if kp.size > tamano_maximo:
                tamano_maximo = kp.size
                letra_estado = "R"
                color_debug = (0, 0, 255)
                kp_ganador = [kp]

        for kp in kp_yellow:
            if kp.size > tamano_maximo:
                tamano_maximo = kp.size
                letra_estado = "A"
                color_debug = (0, 255, 255)
                kp_ganador = [kp]

        for kp in kp_green:
            if kp.size > tamano_maximo:
                tamano_maximo = kp.size
                letra_estado = "V"
                color_debug = (0, 255, 0)
                kp_ganador = [kp]

        # --- PUBLICACIÓN ---
        msg_letra = String()
        msg_letra.data = letra_estado
        self.state_pub.publish(msg_letra)

        result_img = cv_img.copy()
        if kp_ganador:
            result_img = cv.drawKeypoints(result_img, kp_ganador, np.array([]), color_debug, 
                                         cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        cv.putText(result_img, f"Estado: {letra_estado}", (20, 50), 
                   cv.FONT_HERSHEY_SIMPLEX, 1, color_debug, 2)
        
        img_msg = self.bridge.cv2_to_imgmsg(result_img, encoding='bgr8')
        self.image_pub.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
