from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # pacote que publica os landmarks das mãos com mediapipe
    hand_landmarks_node = Node(
        package='hand_landmarker_ros',
        executable='hand_landmarks_publisher',
        name='hand_landmarks_publisher',
        output='screen'
    )

    # pacote que publica o estado dos dedos das maos
    hand_state_node = Node(
        package='hand_landmarker_ros',
        executable='hand_state_publisher',
        name='hand_state_publisher',
        output='screen'
    )

    # TM Driver nodes
    # Robô da esquerda
    tm_driver_left = Node(
        package='tm_driver',
        executable='tm_driver',
        name='tm_driver_left',
        output='screen',
        arguments=['robot_ip:=169.254.78.132'],
        remappings=[
            ('/tmr_arm_controller/follow_joint_trajectory/_action/feedback', '/left_tmr_arm_controller/follow_joint_trajectory/_action/feedback'),
            ('/tmr_arm_controller/follow_joint_trajectory/_action/status', '/left_tmr_arm_controller/follow_joint_trajectory/_action/status'),
            ('/tmr_arm_controller/follow_joint_trajectory/_action/cancel_goal', '/left_tmr_arm_controller/follow_joint_trajectory/_action/cancel_goal'),
            ('/tmr_arm_controller/follow_joint_trajectory/_action/get_result', '/left_tmr_arm_controller/follow_joint_trajectory/_action/get_result'),
            ('/tmr_arm_controller/follow_joint_trajectory/_action/send_goal', '/left_tmr_arm_controller/follow_joint_trajectory/_action/send_goal'),
        ]
    )

    # Robô da direita
    tm_driver_right = Node(
        package='tm_driver',
        executable='tm_driver',
        name='tm_driver_right',
        output='screen',
        arguments=['robot_ip:=169.254.78.131'],
        remappings=[
                ('/ask_item', '/ask_item2'),
                ('/ask_sta', '/ask_sta2'),
                ('/connect_tmsct', '/connect_tmsct2'),
                ('/connect_tmsvr', '/connect_tmsvr2'),
                ('/send_script', '/send_script2'),
                ('/set_event', '/set_event2'),
                ('/set_io', '/set_io2'),
                ('/set_positions', '/set_positions2'),
                ('/tm_driver_node/describe_parameters', '/tm_driver_node2/describe_parameters2'),
                ('/tm_driver_node/get_parameter_types', '/tm_driver_node2/get_parameter_types'),
                ('/tm_driver_node/get_parameters', '/tm_driver_node2/get_parameters'),
                ('/tm_driver_node/list_parameters', '/tm_driver_node2/list_parameters'),
                ('/tm_driver_node/set_parameters', '/tm_driver_node2/set_parameters'),
                ('/tm_driver_node/set_parameters_atomically', '/tm_driver_node2/set_parameters_atomically'),
                ('/write_item', '/write_item2'),
                ('/feedback_states', '/feedback_states2'),
                ('/joint_states', '/joint_states2'),
                ('/sct_response', '/sct_response2'),
                ('/sta_response', '/sta_response2'),
                ('/svr_response', '/svr_response2'),
                ('/tool_pose', '/tool_pose2'),
                ('/tmr_arm_controller/follow_joint_trajectory/_action/feedback', '/right_tmr_arm_controller/follow_joint_trajectory/_action/feedback'),
                ('/tmr_arm_controller/follow_joint_trajectory/_action/status', '/right_tmr_arm_controller/follow_joint_trajectory/_action/status'),
                ('/tmr_arm_controller/follow_joint_trajectory/_action/cancel_goal', '/right_tmr_arm_controller/follow_joint_trajectory/_action/cancel_goal'),
                ('/tmr_arm_controller/follow_joint_trajectory/_action/get_result', '/right_tmr_arm_controller/follow_joint_trajectory/_action/get_result'),
                ('/tmr_arm_controller/follow_joint_trajectory/_action/send_goal', '/right_tmr_arm_controller/follow_joint_trajectory/_action/send_goal'),

            ]
    )

    # Retorna todos os nodes no LaunchDescription
    return LaunchDescription([
        hand_landmarks_node,
        hand_state_node,
        tm_driver_left,
        tm_driver_right,

    ])
