#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def gstreamer_pipeline(
    sensor_id=0,
    sensor_mode=3,       # Modo 3 = 1640x1232 
    capture_width=1640,
    capture_height=1232,
    display_width=640,  
    display_height=480,
    framerate=30,
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
        
        self.publisher_ = self.create_publisher(Image, '/video_source/raw', 10)
        self.bridge = CvBridge()
        
        pipeline = gstreamer_pipeline()
        self.get_logger().info(f"Intentando abrir la cámara con GStreamer:\n{pipeline}")
        
        # Abrir la cámara usando GStreamer en OpenCV
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara CSI. ¿Hay otro nodo usándola?")
            return
            
        # Temporizador para leer y publicar a ~30 FPS
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info("¡Cámara iniciada! Publicando en /video_source/raw sin zoom...")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convertir la imagen de OpenCV (BGR) 
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            
            self.publisher_.publish(msg)
        else:
            self.get_logger().warning("Fallo al capturar frame de la cámara.")

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