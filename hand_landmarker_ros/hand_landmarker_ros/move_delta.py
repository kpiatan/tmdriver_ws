#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from tm_msgs.srv import SetPositions
from tm_msgs.msg import FeedbackState  # msg que contém joint_pos
import time
import copy

class MoveDelta(Node):
    def __init__(self):
        super().__init__('move_delta')

        # Clientes de movimento
        self.client_left = self.create_client(SetPositions, 'set_positions')
        self.client_right = self.create_client(SetPositions, 'set_positions2')

        while not self.client_left.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando serviço set_positions (esquerda)...')
        while not self.client_right.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando serviço set_positions2 (direita)...')

        # Subscribers dos gestos
        self.sub_left = self.create_subscription(Int32MultiArray, 'mao_esquerda', self.callback_left, 10)
        self.sub_right = self.create_subscription(Int32MultiArray, 'mao_direita', self.callback_right, 10)

        # Subscribers de feedback de posicao
        self.sub_feedback_left = self.create_subscription(FeedbackState, '/feedback_states', self.feedback_left_cb, 10)
        self.sub_feedback_right = self.create_subscription(FeedbackState, '/feedback_states2', self.feedback_right_cb, 10)

        self.last_feedback_left = None
        self.last_feedback_right = None

        # Controle de frequência de comandos ao robô
        self.last_time_left = 0
        self.last_time_right = 0
        self.step_interval = 0.8  # segundos entre passos
        self.increment = 0.04     # radianos por passo

        self.velocity = 0.1

        # Limites individuais de aproximacao no centro da bancada 
        self.limit_left = -0.2763647
        self.limit_right = 0.2421141

    # CALLBACKS DE FEEDBACK 
    def feedback_left_cb(self, msg):
        self.last_feedback_left = msg

    def feedback_right_cb(self, msg):
        self.last_feedback_right = msg

    # CALLBACKS DE GESTOS 
    def callback_left(self, msg):
        gesture = tuple(msg.data)
        now = time.time()
        if gesture in [(1,1,1,1), (1,1,1,0)] and self.last_feedback_left is not None:
            if now - self.last_time_left >= self.step_interval:
                self.last_time_left = now
                delta = self.increment if gesture == (1,1,1,1) else -self.increment
                self.increment_joint(self.client_left, self.last_feedback_left.joint_pos, delta, "esquerda", gesture)

    def callback_right(self, msg):
        gesture = tuple(msg.data)
        now = time.time()
        if gesture in [(1,1,1,1), (1,1,1,0)] and self.last_feedback_right is not None:
            if now - self.last_time_right >= self.step_interval:
                self.last_time_right = now
                delta = -self.increment if gesture == (1,1,1,1) else +self.increment
                self.increment_joint(self.client_right, self.last_feedback_right.joint_pos, delta, "direita", gesture)

    # FUNÇÃO PARA ENVIAR MOVIMENTO 
    def increment_joint(self, client, current_positions, delta, side, gesture):
        new_positions = list(copy.deepcopy(current_positions))
        if len(new_positions) > 1:  # junta 2 existe
            new_val = new_positions[1] + delta

            # aplica limites SOMENTE no gesto (1,1,1,0)
            if gesture == (1,1,1,0):
                if side == "esquerda" and new_val < self.limit_left:
                    self.get_logger().warn(
                        f"[{side}] Limite atingido ({new_val:.6f} rad). Não movendo mais para dentro."
                    )
                    return
                if side == "direita" and new_val > self.limit_right:
                    self.get_logger().warn(
                        f"[{side}] Limite atingido ({new_val:.6f} rad). Não movendo mais para dentro."
                    )
                    return

            new_positions[1] = new_val
        else:
            self.get_logger().error(f"[{side}] Feedback inválido (sem junta 2).")
            return

        req = SetPositions.Request()
        req.motion_type = SetPositions.Request.PTP_J
        req.positions = new_positions
        req.velocity = self.velocity
        req.acc_time = 0.2
        req.blend_percentage = 0
        req.fine_goal = False

        self.get_logger().info(f"[{side}] Gesto {gesture} → movendo junta 2 para {new_positions[1]:.6f} rad")

        future = client.call_async(req)
        future.add_done_callback(lambda f: self.motion_done_callback(f, side, gesture))

    def motion_done_callback(self, future, side, gesture):
        try:
            result = future.result()
            if result.ok:
                self.get_logger().info(f"[{side}] Movimento incremental ({gesture}) enviado com sucesso")
            else:
                self.get_logger().error(f"[{side}] Falha ao enviar movimento incremental ({gesture})")
        except Exception as e:
            self.get_logger().error(f"[{side}] Erro ao enviar movimento incremental ({gesture}): {e}")

def main():
    rclpy.init()
    node = MoveDelta()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrompido pelo usuário")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
