#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nó ROS2 que captura vídeo da câmera, detecta landmarks da mão com MediaPipe,
desenha no OpenCV com cores diferentes para cada dedo e cada mão,
e publica o estado dos dedos médio, anelar e mínimo em tópicos separados:
- "mao_esquerda"
- "mao_direita"
"""

import rclpy
from rclpy.node import Node
import cv2
import mediapipe as mp
from std_msgs.msg import String


class HandGestureNode(Node):
    def __init__(self):
        super().__init__('hand_gesture_node')

        # Publishers para cada mão
        self.pub_left = self.create_publisher(String, 'mao_esquerda', 10)
        self.pub_right = self.create_publisher(String, 'mao_direita', 10)

        # Câmera
        self.cap = cv2.VideoCapture(2) # 0 camera do note, 2 camera
        if not self.cap.isOpened():
            self.get_logger().error('Não foi possível abrir a câmera.')
            return

        # MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.hand_connections = self.mp_hands.HAND_CONNECTIONS

    def is_flexed(self, tip, pip):
        """
        Retorna True se o dedo estiver dobrado.
        Ajustado para câmera de cima (dorso da mão).
        """
        return tip.y < pip.y

    def process_hand(self, frame, hand_landmarks, handedness):
        """
        Desenha landmarks e verifica estado dos dedos médio, anelar e mínimo.
        Publica o resultado no tópico correspondente.
        """
        h, w, _ = frame.shape

        # Cor base dependendo da mão
        if handedness == "Left":
            base_color = (255, 0, 0)   # azul
        else:
            base_color = (0, 0, 255)   # vermelho

        # --- Desenho das landmarks com cores por dedo ---
        for idx, lm in enumerate(hand_landmarks.landmark):
            cx, cy = int(lm.x * w), int(lm.y * h)

            if idx in [9, 10, 11, 12]:
                color = (0, 255, 0)         # médio verde
            elif idx in [13, 14, 15, 16]:
                color = (0, 255, 255)       # anelar amarelo
            elif idx in [17, 18, 19, 20]:
                color = (255, 0, 255)       # mínimo magenta
            else:
                color = base_color          # outros pontos na cor da mão

            cv2.circle(frame, (cx, cy), 5, color, -1)

        # --- Desenho das conexões ---
        for connection in self.hand_connections:
            start_idx, end_idx = connection
            start_lm = hand_landmarks.landmark[start_idx]
            end_lm = hand_landmarks.landmark[end_idx]
            x0, y0 = int(start_lm.x * w), int(start_lm.y * h)
            x1, y1 = int(end_lm.x * w), int(end_lm.y * h)
            cv2.line(frame, (x0, y0), (x1, y1), base_color, 2)

        # --- Detecta flexão dos dedos ---
        médio_flex = self.is_flexed(hand_landmarks.landmark[12], hand_landmarks.landmark[10])
        anelar_flex = self.is_flexed(hand_landmarks.landmark[16], hand_landmarks.landmark[14])
        mínimo_flex = self.is_flexed(hand_landmarks.landmark[20], hand_landmarks.landmark[18])

        estado = {
            "Médio": "Dobrado" if médio_flex else "Estendido",
            "Anelar": "Dobrado" if anelar_flex else "Estendido",
            "Mínimo": "Dobrado" if mínimo_flex else "Estendido",
        }

        # --- Publica no tópico correspondente ---
        msg = String()
        msg.data = f"Mão {handedness} | " + " | ".join([f"{d}:{s}" for d, s in estado.items()])
        if handedness == "Left":
            self.pub_left.publish(msg)
        else:
            self.pub_right.publish(msg)

        # Debug no terminal
        self.get_logger().info(msg.data)

    def run(self):
        self.get_logger().info('Iniciando vídeo. Pressione Q para sair.')
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn('Falha ao ler frame da câmera.')
                break

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(frame_rgb)

            if results.multi_hand_landmarks and results.multi_handedness:
                for hand_landmarks, handedness in zip(results.multi_hand_landmarks,
                                                     results.multi_handedness):
                    label = handedness.classification[0].label
                    # Corrige inversão (MediaPipe espelha)
                    if label == "Left":
                        label = "Right"
                    else:
                        label = "Left"

                    self.process_hand(frame, hand_landmarks, label)

            cv2.imshow('Hand Gesture Detector', frame)
            if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q')):
                break

            rclpy.spin_once(self, timeout_sec=0)

        self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = HandGestureNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrompido pelo usuário')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
