#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import cv2
import mediapipe as mp
from std_msgs.msg import String
import json

class HandLandmarksPublisher(Node):
    def __init__(self):
        super().__init__('hand_landmarks_publisher')

        self.publisher_ = self.create_publisher(String, 'hand_landmarks', 10)

        self.cap = cv2.VideoCapture(2) # 0 para camera notebook, 2 para camera usb
        if not self.cap.isOpened():
            self.get_logger().error('Não foi possível abrir a câmera.')
            return

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

    def draw_hand(self, frame, hand_landmarks, hand_label):
        """Desenha cada mão com cores diferentes, separando dedos internos e externos"""
        h, w, _ = frame.shape

        # Define cor base por mão
        if hand_label == "Left":
            color_internos = (0, 255, 0)   # verde -> polegar + indicador
            color_externos = (0, 128, 255) # laranja -> médio + anelar + mínimo
        else:
            color_internos = (255, 0, 0)   # azul -> polegar + indicador
            color_externos = (255, 0, 255) # rosa -> médio + anelar + mínimo

        # Conexões dos dedos
        internos = [(4,3),(3,2),(2,1)] + [(8,7),(7,6),(6,5)]  # polegar + indicador
        externos = [
            (12,11),(11,10),(10,9),   # médio
            (16,15),(15,14),(14,13),  # anelar
            (20,19),(19,18),(18,17)   # mínimo
        ]

        # Desenha internos
        for start, end in internos:
            p1, p2 = hand_landmarks.landmark[start], hand_landmarks.landmark[end]
            cv2.line(frame, (int(p1.x*w), int(p1.y*h)), (int(p2.x*w), int(p2.y*h)), color_internos, 2)
            cv2.circle(frame, (int(p1.x*w), int(p1.y*h)), 4, color_internos, -1)
            cv2.circle(frame, (int(p2.x*w), int(p2.y*h)), 4, color_internos, -1)

        # Desenha externos
        for start, end in externos:
            p1, p2 = hand_landmarks.landmark[start], hand_landmarks.landmark[end]
            cv2.line(frame, (int(p1.x*w), int(p1.y*h)), (int(p2.x*w), int(p2.y*h)), color_externos, 2)
            cv2.circle(frame, (int(p1.x*w), int(p1.y*h)), 4, color_externos, -1)
            cv2.circle(frame, (int(p2.x*w), int(p2.y*h)), 4, color_externos, -1)

    def run(self):
        self.get_logger().info('Capturando vídeo. Pressione Q para sair.')
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn('Falha ao ler frame da câmera.')
                break

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(frame_rgb)

            if results.multi_hand_landmarks and results.multi_handedness:
                for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                    hand_label = handedness.classification[0].label  # "Left" ou "Right"

                    # Publica landmarks em JSON
                    landmarks = [{"x": lm.x, "y": lm.y, "z": lm.z} for lm in hand_landmarks.landmark]
                    msg = String()
                    msg.data = json.dumps({"hand": hand_label, "landmarks": landmarks})
                    self.publisher_.publish(msg)

                    # Desenha com cores
                    self.draw_hand(frame, hand_landmarks, hand_label)

            cv2.imshow("Hand Landmarks Publisher", frame)
            if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q')):
                self.get_logger().info("Encerrando pela tecla Q.")
                break

            rclpy.spin_once(self, timeout_sec=0)

        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = HandLandmarksPublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrompido pelo usuário')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
