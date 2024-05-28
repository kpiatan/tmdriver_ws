# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rclpy
import time
from rclpy.node import Node

from sensor_msgs.msg import JointState


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_statesLR', 10)
        timer_period = 0.05  # seconds
        self.i = 0
        self.subscriptionleft = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback_left,
            10)
        self.subscriptionleft  # prevent unused variable warning
        self.subscriptionright = self.create_subscription(
            JointState,
            'joint_states2',
            self.listener_callback_right,
            10)
        self.subscriptionright  # prevent unused variable warning
        #time.sleep(10)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def listener_callback_left(self, msg):
        global mensagem_left
        mensagem_left = msg
        # self.get_logger().info('I heard: "%s"' % msg.position)

    def listener_callback_right(self, msg):
        global mensagem_right
        mensagem_right = msg
        # self.get_logger().info('I heard: "%s"' % msg.position)

    # def timer_callback(self):
    #     global mensagem_left, mensagem_right
    #     msg = JointState()
    #     msg.header.stamp = self.get_clock().now().to_msg()
    #     msg.name.append('leftjoint_1')
    #     msg.name.append('leftjoint_2')
    #     msg.name.append('leftjoint_3')
    #     msg.name.append('leftjoint_4')
    #     msg.name.append('leftjoint_5')
    #     msg.name.append('leftjoint_6')
    #     msg.name.append('rightjoint_1')
    #     msg.name.append('rightjoint_2')
    #     msg.name.append('rightjoint_3')
    #     msg.name.append('rightjoint_4')
    #     msg.name.append('rightjoint_5')
    #     msg.name.append('rightjoint_6')
    #     msg.position.append(mensagem_left.position[0])
    #     msg.position.append(mensagem_left.position[1])
    #     msg.position.append(mensagem_left.position[2])
    #     msg.position.append(mensagem_left.position[3])
    #     msg.position.append(mensagem_left.position[4])
    #     msg.position.append(mensagem_left.position[5])
    #     msg.position.append(mensagem_right.position[0])
    #     msg.position.append(mensagem_right.position[1])
    #     msg.position.append(mensagem_right.position[2])
    #     msg.position.append(mensagem_right.position[3])
    #     msg.position.append(mensagem_right.position[4])
    #     msg.position.append(mensagem_right.position[5])
    #     msg.velocity.append(mensagem_left.velocity[0])
    #     msg.velocity.append(mensagem_left.velocity[1])
    #     msg.velocity.append(mensagem_left.velocity[2])
    #     msg.velocity.append(mensagem_left.velocity[3])
    #     msg.velocity.append(mensagem_left.velocity[4])
    #     msg.velocity.append(mensagem_left.velocity[5])
    #     msg.velocity.append(mensagem_right.velocity[0])
    #     msg.velocity.append(mensagem_right.velocity[1])
    #     msg.velocity.append(mensagem_right.velocity[2])
    #     msg.velocity.append(mensagem_right.velocity[3])
    #     msg.velocity.append(mensagem_right.velocity[4])
    #     msg.velocity.append(mensagem_right.velocity[5])
    #     msg.effort.append(mensagem_left.effort[0])
    #     msg.effort.append(mensagem_left.effort[1])
    #     msg.effort.append(mensagem_left.effort[2])
    #     msg.effort.append(mensagem_left.effort[3])
    #     msg.effort.append(mensagem_left.effort[4])
    #     msg.effort.append(mensagem_left.effort[5])
    #     msg.effort.append(mensagem_right.effort[0])
    #     msg.effort.append(mensagem_right.effort[1])
    #     msg.effort.append(mensagem_right.effort[2])
    #     msg.effort.append(mensagem_right.effort[3])
    #     msg.effort.append(mensagem_right.effort[4])
    #     msg.effort.append(mensagem_right.effort[5])
    #     self.publisher_.publish(msg)
    #     self.i += 1

    def timer_callback(self):
        global mensagem_left, mensagem_right
        
        # Check if both mensagem_left and mensagem_right have been populated
        if not mensagem_left.position or not mensagem_right.position:
            self.get_logger().warn("Waiting for joint state messages...")
            return
        
        # # Check if the positions have the expected number of joints
        # if len(mensagem_left.position) < 6 or len(mensagem_right.position) < 6:
        #     self.get_logger().warn("Received joint states do not have enough data.")
        #     return
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        joint_names = [f'leftjoint_{i+1}' for i in range(6)] + [f'rightjoint_{i+1}' for i in range(6)]
        msg.name.extend(joint_names)
        msg.position.extend(mensagem_left.position[:6] + mensagem_right.position[:6])
        msg.velocity.extend(mensagem_left.velocity[:6] + mensagem_right.velocity[:6])
        msg.effort.extend(mensagem_left.effort[:6] + mensagem_right.effort[:6])
        self.publisher_.publish(msg)
        self.i += 1

mensagem_left = JointState()
mensagem_right = JointState()

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
