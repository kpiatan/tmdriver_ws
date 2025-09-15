#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
import json
import math


def distance(a, b):
    return math.sqrt((a["x"] - b["x"])**2 +
                     (a["y"] - b["y"])**2 +
                     (a["z"] - b["z"])**2)


class HandStatePublisher(Node):
    def __init__(self):
        super().__init__('hand_state_publisher')

        # Inscreve no tópico de landmarks
        self.sub = self.create_subscription(
            String,
            'hand_landmarks',
            self.callback,
            10
        )

        # Publicadores para cada mão
        self.pub_left = self.create_publisher(Int32MultiArray, 'mao_esquerda', 10)
        self.pub_right = self.create_publisher(Int32MultiArray, 'mao_direita', 10)

    def is_flexed(self, landmarks, tip_id, pip_id):
        # True = dobrado, False = estendido
        return landmarks[tip_id]["y"] < landmarks[pip_id]["y"]

    def callback(self, msg):
        data = json.loads(msg.data)
        hand_label = data["hand"]
        landmarks = data["landmarks"]

        # Corrige inversão esquerda/direita (dependendo da câmera)
        if hand_label == "Left":
            hand_label = "Right"
        else:
            hand_label = "Left"

        # Estados dos dedos
        medio_flex = self.is_flexed(landmarks, 12, 10)
        anelar_flex = self.is_flexed(landmarks, 16, 14)
        minimo_flex = self.is_flexed(landmarks, 20, 18)

        # Detecta emergência (3 dedos estendidos e afastados)
        todos_estendidos = (not medio_flex) and (not anelar_flex) and (not minimo_flex)
        dist_ma = distance(landmarks[12], landmarks[16])
        dist_am = distance(landmarks[16], landmarks[20])
        dist_mm = distance(landmarks[12], landmarks[20])
        afastados = (dist_ma > 0.04) and (dist_am > 0.04) and (dist_mm > 0.04)
        emergencia = todos_estendidos and afastados

        # Converte dedos para 0/1 (1 = estendido, 0 = dobrado)
        out = Int32MultiArray()
        out.data = [
            int(not medio_flex),
            int(not anelar_flex),
            int(not minimo_flex),
            int(emergencia)
        ]

        # Log no terminal em texto
        dedos_texto = ['Médio', 'Anelar', 'Mínimo']
        estados = ['Dobrado', 'Estendido']
        info = ', '.join(f"{dedo}: {estados[int(not flex)]}" for dedo, flex in zip(dedos_texto, [medio_flex, anelar_flex, minimo_flex]))
        emergencia_texto = 'SIM' if emergencia else 'NÃO'
        self.get_logger().info(f"Mão {hand_label} | {info} | Emergência: {emergencia_texto}")

        # Publica no tópico certo
        if hand_label == "Left":
            self.pub_left.publish(out)
        else:
            self.pub_right.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = HandStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrompido pelo usuário")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
