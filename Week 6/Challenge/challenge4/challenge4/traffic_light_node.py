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

        # CONFIGURACIÓN DEL DETECTOR DE BLOBS
        params = cv.SimpleBlobDetector_Params()
        params.filterByColor = True
        params.blobColor = 255
        params.filterByArea = True
        params.minArea = 50  
        params.maxArea = 50000
        params.filterByCircularity = True
        params.minCircularity = 0.72
        params.filterByConvexity = False
        params.filterByInertia = False
        self.detector = cv.SimpleBlobDetector_create(params)

        # SUSCRIPTOR Y PUBLICADORES
        self.subscription = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10)
        self.state_pub = self.create_publisher(String, '/semaforo_estado', 10)
        self.image_pub = self.create_publisher(Image, '/image_processing/result', 10)

        self.get_logger().info('Nodo de Semáforo: Distancia basada en el TAMAÑO DEL COLOR')

    def camera_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            return

        cv_img = cv.convertScaleAbs(cv_img, alpha=1.0, beta=-10)

        # PROCESAMIENTO
        blurred = cv.GaussianBlur(cv_img, (5, 5), 0)
        hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

        mask_red = cv.bitwise_or(
            cv.inRange(hsv, np.array([0, 50, 150]), np.array([8, 255, 255])),
            cv.inRange(hsv, np.array([170, 50, 120]), np.array([180, 255, 255]))
        )
        #mask_yellow = cv.inRange(hsv, np.array([17, 60, 180]), np.array([35, 160, 255]))
        #este es el buenomask_yellow = cv.inRange(hsv, np.array([10, 30, 80]), np.array([35, 255, 255]))
        mask_yellow = cv.inRange(hsv, np.array([10, 0, 80]), np.array([35, 255, 255]))
        mask_green = cv.inRange(hsv, np.array([40, 40, 120]), np.array([90, 255, 255]))
        
        # detección de zona blanca 
        #mask_white_original = cv.inRange(hsv, np.array([0, 0, 240]), np.array([180, 40, 255]))
        mask_white_original = cv.inRange(hsv, np.array([0, 0, 180]), np.array([180, 50, 255]))
        kernel_big = np.ones((15, 15), np.uint8)
        white_dilated = cv.dilate(mask_white_original, kernel_big, iterations=2)
        ring = cv.subtract(white_dilated, mask_white_original)

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
        
        # Detección de los blobs de colores
        kp_red = self.detector.detect(mask_red)
        kp_yellow = self.detector.detect(mask_yellow)
        kp_green = self.detector.detect(mask_green)

        #Estados
        letra_detectada = "N"
        color_debug = (255, 255, 255)
        tamano_maximo = 0
        kp_ganador = []

        #Buscamos qué color está encendido y medimos qué tan grande es
        for kp in kp_red:
            if kp.size > tamano_maximo:
                tamano_maximo = kp.size
                letra_detectada = "R"
                color_debug = (0, 0, 255)
                kp_ganador = [kp]

        for kp in kp_yellow:
            if kp.size > tamano_maximo:
                tamano_maximo = kp.size
                letra_detectada = "A"
                color_debug = (0, 255, 255)
                kp_ganador = [kp]

        for kp in kp_green:
            if kp.size > tamano_maximo:
                tamano_maximo = kp.size
                letra_detectada = "V"
                color_debug = (0, 255, 0)
                kp_ganador = [kp]

        
        #Distancia de 35 cm
        UMBRAL_TAMANO_LUZ_35CM = 30.0  

        letra_final = "N" 
        
        # Si el tamaño del blob es mayor a nuestro umbral, significa que ya llegamos a 35cm
        if tamano_maximo > UMBRAL_TAMANO_LUZ_35CM:
            letra_final = letra_detectada

        #Publicacion
        msg_letra = String()
        msg_letra.data = letra_final
        self.state_pub.publish(msg_letra)

        #DIbujo para pantalla
        result_img = cv_img.copy()
        
        if kp_ganador:
            result_img = cv.drawKeypoints(result_img, kp_ganador, np.array([]), color_debug, 
                                         cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        # Textos en pantalla para calibrar
        cv.putText(result_img, f"Pub: {letra_final} | Color: {letra_detectada}", (10, 30), 
                   cv.FONT_HERSHEY_SIMPLEX, 0.7, color_debug, 2)
        cv.putText(result_img, f"Tamano Luz: {tamano_maximo:.1f}", (10, 60), 
                   cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
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
