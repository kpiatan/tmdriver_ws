#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tm_msgs.srv import SetPositions
import sys

class SetPositionsClient(Node):
    def __init__(self):
        super().__init__('set_positions_client')
        self.client = self.create_client(SetPositions, 'set_positions2')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando serviço set_positions...')
        
    def send_request(self, positions, velocity):
        request = SetPositions.Request()
        request.motion_type = SetPositions.Request.PTP_J
        request.positions = positions
        request.velocity = velocity
        request.acc_time = 0.2
        request.blend_percentage = 10
        request.fine_goal = False

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().ok:
            self.get_logger().info("Comando enviado com sucesso!")
        else:
            self.get_logger().error("Falha ao enviar comando.")

def main():
    rclpy.init()
    client = SetPositionsClient()

    if len(sys.argv) < 8:
        print("Uso: set_positions.py <p1> <p2> <p3> <p4> <p5> <p6> <velocidade>")
        return

    positions = [float(sys.argv[i]) for i in range(1, 7)]
    velocity = float(sys.argv[7])
    
    client.send_request(positions, velocity)

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
