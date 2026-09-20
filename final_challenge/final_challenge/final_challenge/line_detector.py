#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

# Configuración de Imagen y ROI Ordinaria
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
SCANLINES_Y = [390, 410, 430, 450, 475]
ROI_X_MIN = 190
ROI_X_MAX = 450
ALPHA_CONTRAST = 0.4
BETA_BRIGHTNESS = -30
MIN_LINE_WIDTH = 40


class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector')

        self.bridge = CvBridge()

        # Suscriptor de cámara
        self.image_sub = self.create_subscription(
            Image,
            '/video_source/raw',
            self.image_callback,
            10
        )

        # Publicadores
        self.err_pub = self.create_publisher(
            Float32,
            '/puzzlebot/line_error',
            10
        )

        self.thresh_pub = self.create_publisher(
            Image,
            '/puzzlebot/camera/thresh',
            10
        )

        self.debug_pub = self.create_publisher(
            Image,
            '/puzzlebot/camera/debug',
            10
        )

        self.get_logger().info(
            'Nodo Line Detector (Solo Seguimiento de Carril) iniciado.'
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            if (
                frame.shape[1] != IMAGE_WIDTH
                or frame.shape[0] != IMAGE_HEIGHT
            ):
                frame = cv2.resize(
                    frame,
                    (IMAGE_WIDTH, IMAGE_HEIGHT)
                )

            # 1. Preprocesamiento
            frame = cv2.convertScaleAbs(
                frame,
                alpha=ALPHA_CONTRAST,
                beta=BETA_BRIGHTNESS
            )

            gray = cv2.cvtColor(
                frame,
                cv2.COLOR_BGR2GRAY
            )

            blur = cv2.GaussianBlur(
                gray,
                (9, 9),
                0
            )

            _, thresh = cv2.threshold(
                blur,
                0,
                255,
                cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
            )

            kernel = np.ones((7, 7), np.uint8)
            thresh = cv2.erode(
                thresh,
                kernel,
                iterations=4
            )

            # 2. ROI Lateral para control de carril
            thresh_control = thresh.copy()
            thresh_control[:, :ROI_X_MIN] = 0
            thresh_control[:, ROI_X_MAX:] = 0

            image_center = IMAGE_WIDTH // 2
            full_debug = frame.copy()

            # Capa visual (Gris oscuro céntrico)
            overlay = full_debug.copy()

            cv2.rectangle(
                overlay,
                (ROI_X_MIN, 0),
                (ROI_X_MAX, IMAGE_HEIGHT),
                (0, 40, 0),
                -1
            )

            cv2.addWeighted(
                overlay,
                0.4,
                full_debug,
                0.6,
                0,
                full_debug
            )

            # Dibujar rectángulo de escaneo
            cv2.rectangle(
                full_debug,
                (ROI_X_MIN, SCANLINES_Y[0] - 10),
                (ROI_X_MAX, SCANLINES_Y[-1] + 10),
                (255, 0, 255),
                2
            )

            errors = []

            # 3. Escaneo por filas (Scanlines)
            for y in SCANLINES_Y:
                row_pixels = thresh_control[y, :]
                line_blocks = []

                x = ROI_X_MIN

                while x < ROI_X_MAX:
                    if row_pixels[x] == 255:
                        start_x = x
                        end_x = start_x

                        while (
                            end_x < ROI_X_MAX
                            and row_pixels[end_x] == 255
                        ):
                            end_x += 1

                        end_x -= 1

                        width = end_x - start_x + 1

                        if width >= MIN_LINE_WIDTH:
                            line_blocks.append(
                                (start_x, end_x)
                            )

                        x = end_x + 1

                    else:
                        x += 1

                if len(line_blocks) > 0:
                    best_block = min(
                        line_blocks,
                        key=lambda b: abs(
                            ((b[0] + b[1]) / 2.0)
                            - image_center
                        )
                    )

                    pLeft_x, pRight_x = best_block

                    line_center_x = (
                        pLeft_x + pRight_x
                    ) / 2.0

                    error = (
                        line_center_x
                        - image_center
                    )

                    errors.append(error)

                    for b in line_blocks:
                        cv2.circle(
                            full_debug,
                            (int(b[0]), y),
                            4,
                            (0, 255, 0),
                            -1
                        )

                        cv2.circle(
                            full_debug,
                            (int(b[1]), y),
                            4,
                            (0, 255, 0),
                            -1
                        )

                    cv2.circle(
                        full_debug,
                        (int(line_center_x), y),
                        5,
                        (0, 0, 255),
                        -1
                    )

                cv2.line(
                    full_debug,
                    (ROI_X_MIN, y),
                    (ROI_X_MAX, y),
                    (0, 255, 255),
                    1
                )

            # Eje central de referencia
            cv2.line(
                full_debug,
                (image_center, 0),
                (image_center, IMAGE_HEIGHT),
                (255, 255, 255),
                1
            )

            # 4. Publicar Error y Texto
            avg_error = 0.0
            mode = 'LOST'

            if errors:
                avg_error = sum(errors) / len(errors)
                mode = 'TRACK'

                self.err_pub.publish(
                    Float32(
                        data=float(avg_error)
                    )
                )

            cv2.putText(
                full_debug,
                f"M: {mode} E: {avg_error:.0f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

            # 5. Publicar visualización
            timestamp = self.get_clock().now().to_msg()

            thresh_msg = self.bridge.cv2_to_imgmsg(
                thresh_control,
                encoding='mono8'
            )

            thresh_msg.header.stamp = timestamp
            self.thresh_pub.publish(thresh_msg)

            debug_msg = self.bridge.cv2_to_imgmsg(
                full_debug,
                encoding='bgr8'
            )

            debug_msg.header.stamp = timestamp
            self.debug_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(
                f'Error en Detector de Línea: {e}'
            )


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LineDetector())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
