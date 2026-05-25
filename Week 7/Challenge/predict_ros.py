import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  # <-- Usando el mensaje estándar String
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        #Cargar modelo
        ruta_modelo = '/home/inaki/Documents/act2/resultados_yolov8_robot/runs/detect/yolov8_puzzlebot_signs/weights/best.pt'
        self.model = YOLO(ruta_modelo)
        self.get_logger().info("Modelo YOLOv8 cargado")
        
        #Inicializar matriz de la imagen a 640x480
        self.img = np.ndarray((480, 640, 3), dtype=np.uint8)
        self.valid_img = False
        self.bridge = CvBridge()

        #Suscriptor al tópico de la cámara
        self.sub = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10)
        
        #Publicadores 
        self.yolo_pub = self.create_publisher(String, '/yolo/detecciones', 10)
        self.yolo_img_pub = self.create_publisher(Image, '/yolo/inference_result', 10)

        #Timer para la inferencia
        timer_period = 1/20
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def camera_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.valid_img = True
        except Exception as e:
            self.get_logger().info(f'Error al procesar la imagen: {e}')

    def timer_callback(self):
        if not self.valid_img:
            return

        #Ejecutar inferencia
        results = self.model(self.img, conf=0.5, verbose=False)
        
        lista_detecciones = []

        for r in results:
            boxes = r.boxes
            for box in boxes:
                #Extraer coordenadas y clase
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = int(box.cls)
                nombre_clase = self.model.names[c]
                
                #Guardar el nombre y las coordenadas [xmin, ymin, xmax, ymax]
                xmin, ymin, xmax, ymax = int(b[0]), int(b[1]), int(b[2]), int(b[3])
                lista_detecciones.append(f"{nombre_clase} [{xmin}, {ymin}, {xmax}, {ymax}]")
        
        #Publicar los datos de los objetos como String
        msg_obj = String()
        msg_obj.data = " | ".join(lista_detecciones) if lista_detecciones else "Ninguno"
        self.yolo_pub.publish(msg_obj)
        
        #Dibujar y publicar la imagen anotada
        frame = results[0].plot()
        msg_img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        
        msg_img.header.frame_id = 'inference' 
        msg_img.header.stamp = self.get_clock().now().to_msg()
        
        self.yolo_img_pub.publish(msg_img)

def main(args=None):
    rclpy.init(args=args)
    nodo = YoloDetectorNode()
    
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()