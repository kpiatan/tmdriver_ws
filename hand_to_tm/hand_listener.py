import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, Pose

class HandListener(Node):
    def __init__(self):
        super().__init__('hand_listener')
        self.sub = self.create_subscription(MarkerArray, '/hand_landmarks_markers', self.callback, 10)
        self.pub = self.create_publisher(PoseArray, '/hand_direction', 10)

    def callback(self, msg):
        # Assumindo que:
        # Marker IDs -> 5 = indicador base, 8 = indicador ponta
        hands = {}
        for marker in msg.markers:
            if marker.ns.startswith('hand_'):  # hand_0 ou hand_1
                hand_id = int(marker.ns.split('_')[1])
                if hand_id not in hands:
                    hands[hand_id] = {}
                if marker.id in [5, 8]:
                    hands[hand_id][marker.id] = marker.pose.position
        
        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "camera_link"

        for _, pts in hands.items():
            if 5 in pts and 8 in pts:
                pose = Pose()
                pose.position.x = pts[5].x
                pose.position.y = pts[5].y
                pose.position.z = pts[5].z
                pose.orientation.x = pts[8].x
                pose.orientation.y = pts[8].y
                pose.orientation.z = pts[8].z
                pose.orientation.w = 0.0
                pose_array.poses.append(pose)

        if pose_array.poses:
            self.pub.publish(pose_array)

def main(args=None):
    rclpy.init(args=args)
    node = HandListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
