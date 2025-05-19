#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nó ROS2 que captura vídeo da câmera, detecta landmarks da mão com MediaPipe,
e publica as landmarks e conexões no formato visualization_msgs/MarkerArray para visualizar no RViz.
"""
import rclpy
from rclpy.node import Node

import cv2
import mediapipe as mp
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

class HandLandmarkerRVizNode(Node):
    def __init__(self):
        super().__init__('hand_landmarker_rviz_node')
        self.publisher_ = self.create_publisher(MarkerArray, 'hand_landmarks_markers', 10)
        
        self.cap = cv2.VideoCapture(0)
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
        self.mp_drawing = mp.solutions.drawing_utils
        self.hand_connections = self.mp_hands.HAND_CONNECTIONS

    def create_sphere_marker(self, x, y, z, ns, id_, r=0.0, g=1.0, b=0.0):
        marker = Marker()
        marker.header.frame_id = "camera_frame"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = id_
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = int(1e8)
        return marker

    def create_line_marker(self, points, ns, id_):
        marker = Marker()
        marker.header.frame_id = "camera_frame"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = id_
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.005  # largura da linha
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # vermelho
        marker.points = points
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = int(1e8)
        return marker

    def publish_landmarks_and_connections(self, hand_landmarks, hand_id=0):
        marker_array = MarkerArray()

        ns = f"hand_{hand_id}"
        landmarks_points = []

        # Cria marcadores para cada landmark (esferas)
        for i, lm in enumerate(hand_landmarks.landmark):
            # Convertendo coordenadas para o sistema do RViz
            x = lm.x - 0.5
            y = 0.5 - lm.y
            z = -lm.z
            landmarks_points.append((x, y, z))

            sphere_marker = self.create_sphere_marker(x, y, z, ns, i)
            marker_array.markers.append(sphere_marker)

        # Cria marcadores para as conexões (linhas)
        # Cada conexão é uma tupla (start_idx, end_idx)
        for idx, connection in enumerate(self.hand_connections):
            start_idx, end_idx = connection
            p_start = landmarks_points[start_idx]
            p_end = landmarks_points[end_idx]

            from geometry_msgs.msg import Point
            line_points = [Point(x=p_start[0], y=p_start[1], z=p_start[2]),
                           Point(x=p_end[0], y=p_end[1], z=p_end[2])]

            line_marker = self.create_line_marker(line_points, ns, idx + 1000)
            marker_array.markers.append(line_marker)

        self.publisher_.publish(marker_array)

    def run(self):
        self.get_logger().info('Iniciando vídeo. Pressione Q para sair.')
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn('Falha ao ler frame da câmera.')
                break

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_rgb.flags.writeable = False
            results = self.hands.process(frame_rgb)
            frame_rgb.flags.writeable = True

            frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

            if results.multi_hand_landmarks:
                for i, hand_landmarks in enumerate(results.multi_hand_landmarks):
                    self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                    self.publish_landmarks_and_connections(hand_landmarks, hand_id=i)

            cv2.imshow('Hand Landmarker', frame)
            if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q')):
                self.get_logger().info('Encerrando pela tecla Q.')
                break

            rclpy.spin_once(self, timeout_sec=0)

        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = HandLandmarkerRVizNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrompido pelo usuário')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
