#!/usr/bin/env python3
"""Détecteur de plantes - version de départ SANS YOLO : couleur HSV + boîtes.
Pipeline : caméra -> detect() -> boîtes dessinées -> fenêtre + compteurs.
Étape suivante : remplacer detect() par un vrai modèle YOLO entraîné.
Lancer : python3 color_detector.py   (Gazebo doit tourner)
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

# Plages HSV (OpenCV : H 0-179, S 0-255, V 0-255).
# Ce sont des valeurs de départ : à calibrer avec le clic souris (voir plus bas).
HEALTHY_LOW, HEALTHY_HIGH = (35, 60, 40), (85, 255, 255)      # vert
DISEASED_LOW, DISEASED_HIGH = (21, 110, 100), (34, 255, 255)  # jaune vif
MIN_AREA = 150                                                # ignorer le bruit (pixels)


def detect(bgr):
    """Retourne une liste de (etat, x, y, w, h) pour chaque plante trouvée."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)
    results = []
    for etat, low, high in (("saine", HEALTHY_LOW, HEALTHY_HIGH),
                            ("malade", DISEASED_LOW, DISEASED_HIGH)):
        mask = cv2.inRange(hsv, np.array(low), np.array(high))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv2.contourArea(c) >= MIN_AREA:
                x, y, w, h = cv2.boundingRect(c)
                results.append((etat, x, y, w, h))
    return results


class PlantDetector(Node):
    def __init__(self):
        super().__init__('plant_detector')
        self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)
        self.bridge = CvBridge()
        self.last_hsv = None
        cv2.namedWindow('Detections')
        cv2.setMouseCallback('Detections', self.on_mouse)
        self.get_logger().info('Detecteur demarre. Clique sur la fenetre pour lire les valeurs HSV.')

    def on_mouse(self, event, x, y, flags, param):
        # Clic gauche : affiche la couleur HSV du pixel (pour calibrer les plages)
        if event == cv2.EVENT_LBUTTONDOWN and self.last_hsv is not None:
            h, s, v = self.last_hsv[y, x]
            self.get_logger().info(f'Pixel ({x},{y}) -> HSV = ({h}, {s}, {v})')

    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge : {e}')
            return

        self.last_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        dets = detect(frame)

        n_saine = sum(1 for d in dets if d[0] == 'saine')
        n_malade = sum(1 for d in dets if d[0] == 'malade')
        for etat, x, y, w, h in dets:
            color = (0, 200, 0) if etat == 'saine' else (0, 0, 255)   # BGR
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.putText(frame, etat, (x, max(y - 5, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        cv2.putText(frame, f'saines: {n_saine}  malades: {n_malade}', (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow('Detections', frame)
        cv2.waitKey(1)

        self.get_logger().info(f'saines={n_saine} malades={n_malade}',
                               throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = PlantDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
