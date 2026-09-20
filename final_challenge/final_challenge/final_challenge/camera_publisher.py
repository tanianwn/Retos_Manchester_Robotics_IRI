#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np

def gstreamer_pipeline(
    sensor_id=0,
    sensor_mode=3,
    capture_width=1640,
    capture_height=1232,
    display_width=640,
    display_height=480,
    framerate=20,
    flip_method=2,
):
    return (
        f"nvarguscamerasrc sensor_id={sensor_id} sensor-mode={sensor_mode} ! "
        f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=(string)BGR ! appsink"
    )

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('csi_camera_publisher')
        
        self.bridge = CvBridge()
        
        # Publicadores
        self.publisher_raw = self.create_publisher(Image, '/video_source/raw', 10)
        self.publisher_compressed = self.create_publisher(CompressedImage, '/video_source/compressed', 10)
        
        # Configuración de FPS para el topic comprimido
        self.target_compressed_fps = 5.0
        self.compressed_interval_ns = (1.0 / self.target_compressed_fps) * 1e9 # Convertido a nanosegundos
        self.last_compressed_time = self.get_clock().now().nanoseconds
        
        pipeline = gstreamer_pipeline()
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara CSI.")
            return
            
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info("Cámara publicando: RAW (~20 FPS) | COMPRESSED (5 FPS)")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            now = self.get_clock().now()
            timestamp = now.to_msg()
            current_time_ns = now.nanoseconds

            # 1. Publicar imagen RAW (Siempre se publica al framerate de captura)
            msg_raw = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg_raw.header.stamp = timestamp
            msg_raw.header.frame_id = "camera_frame"
            self.publisher_raw.publish(msg_raw)

            # 2. Publicar imagen COMPRIMIDA (Limitada a 5 FPS)
            # Evaluamos si ha pasado el tiempo necesario desde el último frame publicado
            if (current_time_ns - self.last_compressed_time) >= self.compressed_interval_ns:
                
                # Actualizamos el temporizador antes de procesar
                self.last_compressed_time = current_time_ns
                
                # Comprimimos la imagen a JPEG con 50% de calidad
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
                _, encoded_image = cv2.imencode('.jpg', frame, encode_param)
                
                msg_comp = CompressedImage()
                msg_comp.header.stamp = timestamp
                msg_comp.header.frame_id = "camera_frame"
                msg_comp.format = "jpeg"
                msg_comp.data = np.array(encoded_image).tobytes()
                self.publisher_compressed.publish(msg_comp)
        else:
            self.get_logger().warning("Fallo al capturar frame.")

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
