import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import math
import subprocess

ROBOT_DISTANCE = 0.5  # 50 cm

class SimpleIKController(Node):
    def __init__(self):
        super().__init__('simple_ik_controller')
        self.declare_parameter('frequency', 10)
        freq = self.get_parameter('frequency').get_parameter_value().integer_value
        self.create_subscription(PoseArray, '/hand_direction', self.callback, 10)
        self.timer = self.create_timer(1.0/freq, self.loop)
        self.last_poses = []

    def finger_to_joints(self, p_base, p_tip):
        dx = p_tip[0] - p_base[0]
        dy = p_tip[1] - p_base[1]
        dz = p_tip[2] - p_base[2]
        yaw = math.atan2(dy, dx)
        pitch = math.atan2(dz, math.sqrt(dx**2 + dy**2))
        return [yaw, pitch, 0.5, 0.0, 0.0, 0.0]

    def send_command(self, robot, joints):
        cmd = [
            'ros2', 'run', 'demo', f'set_positions_{robot}.py',
            str(joints[0]), str(joints[1]), str(joints[2]),
            str(joints[3]), str(joints[4]), str(joints[5]), '1'
        ]
        subprocess.Popen(cmd)

    def loop(self):
        if len(self.last_poses) == 0:
            return

        for i, pose in enumerate(self.last_poses):
            p_base = [pose.position.x, pose.position.y, pose.position.z]
            p_tip = [pose.orientation.x, pose.orientation.y, pose.orientation.z]
            if i == 1:  # robô da direita deslocado 50 cm
                p_base[0] -= ROBOT_DISTANCE
                p_tip[0] -= ROBOT_DISTANCE

            joints = self.finger_to_joints(p_base, p_tip)
            robot = 'left' if i == 0 else 'right'
            self.send_command(robot, joints)

    def callback(self, msg):
        self.last_poses = msg.poses

def main(args=None):
    rclpy.init(args=args)
    node = SimpleIKController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
