from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node para publicar landmarks da mão
        Node(
            package='hand_landmarker_ros',
            executable='hand_landmarks_publisher',
            name='hand_landmarks_publisher',
            output='screen'
        ),

        # Node para publicar estado da mão
        Node(
            package='hand_landmarker_ros',
            executable='hand_state_publisher',  
            name='hand_state_publisher',
            output='screen'
        )
    ])
