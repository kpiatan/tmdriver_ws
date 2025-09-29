#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from tm_msgs.srv import SetPositions
import time

class DualRobotController(Node):
    def __init__(self):
        super().__init__('dual_robot_controller')

        # Clientes para os dois robôs
        self.client_left = self.create_client(SetPositions, 'set_positions')
        self.client_right = self.create_client(SetPositions, 'set_positions2')

        while not self.client_left.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando serviço set_positions (esquerda)...')
        while not self.client_right.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando serviço set_positions2 (direita)...')

        # Subscrições
        self.sub_left = self.create_subscription(Int32MultiArray, 'mao_esquerda', self.callback_left, 10)
        self.sub_right = self.create_subscription(Int32MultiArray, 'mao_direita', self.callback_right, 10)

        # Controle de gestos para não spammar comandos
        self.last_gesture_left = None
        self.last_time_left = 0
        self.last_gesture_right = None
        self.last_time_right = 0
        self.gesture_hold_time = 2.0  # tempo mínimo de gesto (segundos)

        # Dicionário de gestos -> posições (separados para cada robô)
        self.gesture_map_left = {
            (0, 0, 0, 0): [4.1888, 0.3491, -2.1817, 1.5708, -1.3963, 0.0],
            (1, 1, 1, 0): [4.1888, 0.0, -2.1817, 1.5708, -1.3963, 0.0],
            (1, 1, 1, 1): [4.1888, 0.6981, -2.1817, 1.5708, -1.3963, 0.0],  # emergência esquerda
        }
        self.gesture_map_right = {
            (0, 0, 0, 0): [-1.0472, -0.3491, 2.1817, -1.5708, 1.3963, 0.0],
            (1, 1, 1, 0): [-1.0472, 0.0, 2.1817, -1.5708, 1.3963, 0.0],
            (1, 1, 1, 1): [-1.0472, -0.6981, 2.1817, -1.5708, 1.3963, 0.0],  # emergência direita
        }

        self.velocity = 0.3

    # --- CALLBACKS ---
    def callback_left(self, msg):
        gesture = tuple(msg.data)
        now = time.time()

        if self.last_gesture_left != gesture:
            self.last_gesture_left = gesture
            self.last_time_left = now
            self.get_logger().info(f"[esquerda] Gesto detectado: {gesture}")
            return

        if now - self.last_time_left >= self.gesture_hold_time:
            self.get_logger().info(f"[esquerda] Gesto mantido {gesture}, enviando comando...")
            self.execute_motion_async(self.client_left, gesture, "esquerda", self.gesture_map_left)
            self.last_time_left = now  # atualiza timer

    def callback_right(self, msg):
        gesture = tuple(msg.data)
        now = time.time()

        if self.last_gesture_right != gesture:
            self.last_gesture_right = gesture
            self.last_time_right = now
            self.get_logger().info(f"[direita] Gesto detectado: {gesture}")
            return

        if now - self.last_time_right >= self.gesture_hold_time:
            self.get_logger().info(f"[direita] Gesto mantido {gesture}, enviando comando...")
            self.execute_motion_async(self.client_right, gesture, "direita", self.gesture_map_right)
            self.last_time_right = now  # atualiza timer

    # --- EXECUÇÃO ASSÍNCRONA DE MOVIMENTO ---
    def execute_motion_async(self, client, gesture, side, gesture_map):
        if gesture not in gesture_map:
            self.get_logger().warn(f"Gesto {gesture} não mapeado para {side}.")
            return

        positions = gesture_map[gesture]
        request = SetPositions.Request()
        request.motion_type = SetPositions.Request.PTP_J
        request.positions = positions
        request.velocity = self.velocity
        request.acc_time = 0.2
        request.blend_percentage = 10
        request.fine_goal = False

        future = client.call_async(request)
        future.add_done_callback(lambda f: self.motion_done_callback(f, side, gesture))

    def motion_done_callback(self, future, side, gesture):
        try:
            result = future.result()
            if result.ok:
                self.get_logger().info(f"[{side}] Comando enviado com sucesso para {gesture}")
            else:
                self.get_logger().error(f"[{side}] Falha ao enviar comando para {gesture}")
        except Exception as e:
            self.get_logger().error(f"[{side}] Erro ao enviar comando para {gesture}: {e}")

# --- MAIN ---
def main():
    rclpy.init()
    node = DualRobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
