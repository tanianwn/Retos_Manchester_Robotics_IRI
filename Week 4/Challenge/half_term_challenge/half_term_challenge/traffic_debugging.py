import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np

class TrafficLightPrecisionNode(Node):
    def __init__(self):
        super().__init__('traffic_light_precision_node')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10)
        self.state_pub = self.create_publisher(String, '/semaforo_estado', 10)
        self.debug_pub = self.create_publisher(Image, '/image_processing/debug_view', 10)

        self.get_logger().info('Nodo Master: Filtro de exclusión Rojo-Amarillo activado.')

    def procesar_mascara(self, mask, umbral_circularidad, es_amarillo=False):
        # Dilatación inicial para dar cuerpo a los puntos pequeños (evita que el ruido los borre)
        kernel_init = np.ones((5,5) if es_amarillo else (3,3), np.uint8)
        mask = cv.dilate(mask, kernel_init, iterations=1)
        
        mask_final = np.zeros_like(mask)
        cnts, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        for c in cnts:
            area = cv.contourArea(c)
            if area < 10: continue
            
            perimetro = cv.arcLength(c, True)
            if perimetro == 0: continue
            circularidad = 4 * np.pi * (area / (perimetro * perimetro))
            
            # Filtro de forma
            if circularidad > umbral_circularidad:
                (x, y), radius = cv.minEnclosingCircle(c)
                radio_dibujo = max(int(radius * 1.2), 12)
                cv.circle(mask_final, (int(x), int(y)), radio_dibujo, 255, -1)
        
        return mask_final

    def camera_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        # Pre-procesamiento: un poco más oscuro para separar el "blanco" del color
        frame_proc = cv.convertScaleAbs(frame, alpha=1.0, beta=-40)
        hsv = cv.cvtColor(cv.GaussianBlur(frame_proc, (5, 5), 0), cv.COLOR_BGR2HSV)
        
        # ROJO
        m_r1 = cv.inRange(hsv, np.array([0, 150, 130]), np.array([10, 255, 255]))
        m_r2 = cv.inRange(hsv, np.array([160, 150, 130]), np.array([180, 255, 255]))
        mask_red = cv.bitwise_or(m_r1, m_r2)
        
        # AMARILLO
        mask_yellow = cv.inRange(hsv, np.array([26, 30, 170]), np.array([38, 255, 255]))
        
        # VERDE
        mask_green = cv.inRange(hsv, np.array([40, 60, 140]), np.array([95, 255, 255]))

        # Procesamiento
        m_red_s = self.procesar_mascara(mask_red, umbral_circularidad=0.6)
        m_yellow_s = self.procesar_mascara(mask_yellow, umbral_circularidad=0.15, es_amarillo=True)
        m_green_s = self.procesar_mascara(mask_green, umbral_circularidad=0.4)

        def get_blob_info(mask):
            cnts, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            if not cnts: return 0, None
            c = max(cnts, key=cv.contourArea)
            area = cv.contourArea(c)
            M = cv.moments(c)
            if M["m00"] > 0:
                return area, (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))
            return 0, None

        area_r, pos_r = get_blob_info(m_red_s)
        area_a, pos_a = get_blob_info(m_yellow_s)
        area_v, pos_v = get_blob_info(m_green_s)

        # --- LÓGICA DE DECISIÓN ---
        letra, color_bgr, pos_final = "N", (255, 255, 255), None
        UMBRAL_MIN = 50

        # Si el rojo es dominante, ignoramos cualquier "manchita" amarilla (cross-talk)
        if area_r > UMBRAL_MIN and area_r > (area_a * 1.5):
            letra, color_bgr, pos_final = "R", (0, 0, 255), pos_r
        elif area_a > UMBRAL_MIN:
            letra, color_bgr, pos_final = "A", (0, 255, 255), pos_a
        elif area_v > UMBRAL_MIN:
            letra, color_bgr, pos_final = "V", (0, 255, 0), pos_v

        self.state_pub.publish(String(data=letra))

        # --- DEBUG ---
        h, w = 240, 320
        def prep(img, txt, col):
            res = cv.resize(img if len(img.shape)==3 else cv.cvtColor(img, cv.COLOR_GRAY2BGR), (w, h))
            cv.putText(res, txt, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
            return res

        res_panel = frame.copy()
        if pos_final:
            cv.circle(res_panel, pos_final, 20, color_bgr, 3)
            cv.putText(res_panel, f"LUZ: {letra}", (pos_final[0]+25, pos_final[1]), 2, 0.8, color_bgr, 2)

        fila1 = np.hstack([prep(frame, "Original", (255,255,255)), prep(m_red_s, "Rojo", (0,0,255)), prep(m_yellow_s, "Amarillo (Limpio)", (0,255,255))])
        fila2 = np.hstack([prep(m_green_s, "Verde", (0,255,0)), prep(cv.cvtColor(hsv, cv.COLOR_HSV2BGR), "HSV", (255,255,255)), prep(res_panel, f"Veredicto: {letra}", color_bgr)])
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(np.vstack([fila1, fila2]), "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TrafficLightPrecisionNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
