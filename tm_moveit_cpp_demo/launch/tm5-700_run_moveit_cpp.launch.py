import os
import sys
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    args = []
    length = len(sys.argv)
    if (len(sys.argv) >= 5):
        i = 4
        while i < len(sys.argv):
            args.append(sys.argv[i])
            i = i + 1

    args1 = [args[0]]
    args2 = [args[1]]

    # moveit_cpp.yaml is passed by filename for now since it's node specific
    moveit_cpp_yaml_file_name = get_package_share_directory('tm_moveit_cpp_demo') + "/config/moveit_cpp.yaml"

    #moveit_cpp_yaml_file_name2 = get_package_share_directory('tm_moveit_cpp_demo') + "/config/moveit_cpp2.yaml"

    # Component yaml files are grouped in separate namespaces
    # Use URDF file: tm5-700-nominal.urdf to do moveit demo
    # robot_description_config = load_file('tm_description', 'urdf/tm5-700-nominal.urdf')
    # robot_description = {'robot_description' : robot_description_config}
    # Use Xacro file: tm5-700.urdf.xacro to do moveit demo
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("tm_description"),
            "xacro",
            "tm5-700.urdf.xacro",
        )
    )
                                                       
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file('tm_moveit_config_tm5-700', 'config/tm5-700.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}
    #robot_description_semantic_config2 = load_file('tm_moveit_config_tm5-700', 'config/tm5-700.srdf')
    #robot_description_semantic2 = {'robot_description_semantic' : robot_description_semantic_config2}

    kinematics_yaml = load_yaml('tm_moveit_config_tm5-700', 'config/kinematics.yaml')
    robot_description_kinematics = { 'robot_description_kinematics' : kinematics_yaml }
    #kinematics_yaml2 = load_yaml('tm_moveit_config_tm5-700', 'config/kinematics.yaml')
    #robot_description_kinematics2 = { 'robot_description_kinematics' : kinematics_yaml2 }

    controllers_yaml = load_yaml('tm_moveit_cpp_demo', 'config/controllers.yaml')
    moveit_controllers = { 'moveit_simple_controller_manager' : controllers_yaml,
                           'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}
    #controllers_yaml2 = load_yaml('tm_moveit_cpp_demo', 'config/controllers.yaml')
    #moveit_controllers2 = { 'moveit_simple_controller_manager' : controllers_yaml2,
    #                       'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}
    
    ompl_planning_pipeline_config = { 'ompl' : {
        'planning_plugin' : 'ompl_interface/OMPLPlanner',
        'request_adapters' : """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""" ,
        'start_state_max_bounds_error' : 0.1 } }
    ompl_planning_yaml = load_yaml('tm_moveit_config_tm5-700', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    # ompl_planning_pipeline_config2 = { 'ompl' : {
    #     'planning_plugin' : 'ompl_interface/OMPLPlanner',
    #     'request_adapters' : """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""" ,
    #     'start_state_max_bounds_error' : 0.1 } }
    # ompl_planning_yaml2 = load_yaml('tm_moveit_config_tm5-700', 'config/ompl_planning2.yaml')
    # ompl_planning_pipeline_config2['ompl'].update(ompl_planning_yaml2)

    #MoveItCpp demo executable
    run_moveit_cpp_node = Node(
        package='tm_moveit_cpp_demo',
        # TODO(henningkayser): add debug argument
        # prefix='xterm -e gdb --args',
        executable='run_moveit_cpp',
        namespace='',
        output='screen',
        parameters=[
            moveit_cpp_yaml_file_name,
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            moveit_controllers]
        )
    
    # run_moveit_cpp_node2 = Node(
    #     package='tm_moveit_cpp_demo',
    #     # TODO(henningkayser): add debug argument
    #     # prefix='xterm -e gdb --args',
    #     executable='run_moveit_cpp2',
    #     namespace='',
    #     output='screen',
    #     parameters=[
    #         moveit_cpp_yaml_file_name,
    #         robot_description,
    #         robot_description_semantic2,
    #         kinematics_yaml2,
    #         ompl_planning_pipeline_config2,
    #         moveit_controllers2],
    #     remappings=[
    #         ('/moveit_cpp/planning_scene_monitor','/moveit_cpp/planning_scene_monitor2'),
    #         ('/moveit_cpp/publish_planning_scene','/moveit_cpp/publish_planning_scene2'),
    #         ('/moveit_cpp/monitored_planning_scene', '/moveit_cpp/monitored_planning_scene2'),
    #         ('/tmr_arm_controller/follow_joint_trajectory/_action/feedback', '/tmr_arm_controller/follow_joint_trajectory2/_action/feedback'),
    #         ('/tmr_arm_controller/follow_joint_trajectory/_action/status', '/tmr_arm_controller/follow_joint_trajectory2/_action/status'),
    #         ('/tmr_arm_controller/follow_joint_trajectory/_action/cancel_goal', '/tmr_arm_controller/follow_joint_trajectory2/_action/cancel_goal'),
    #         ('/tmr_arm_controller/follow_joint_trajectory/_action/get_result', '/tmr_arm_controller/follow_joint_trajectory2/_action/get_result'),
    #         ('/tmr_arm_controller/follow_joint_trajectory/_action/send_goal', '/tmr_arm_controller/follow_joint_trajectory2/_action/send_goal'),
    #       ]
            
    #     )


    # RViz
    rviz_config_file = get_package_share_directory('tm_moveit_cpp_demo') + "/launch/run_moveit_cpp.rviz"
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic]
        )

    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base']
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # robot_state_publisher2 = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher2',
    #     output='both',
    #     parameters=[robot_description],
    #     remappings=[
    #             ('/joint_states', '/joint_states2'),
    #             ('/robot_description', '/robot_description'),
    #     ]
    # )

    # joint driver
    tm_driver_node = Node(
        package='tm_driver',
        executable='tm_driver',
        namespace='',
        output='screen',
        arguments=args1
    )

    tm_driver_node2 = Node(
        package='tm_driver',
        executable='tm_driver',
        namespace='',
        output='screen',
        arguments=args2,
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
                ('/tmr_arm_controller/follow_joint_trajectory/_action/feedback', '/tmr_arm_controller/follow_joint_trajectory2/_action/feedback'),
                ('/tmr_arm_controller/follow_joint_trajectory/_action/status', '/tmr_arm_controller/follow_joint_trajectory2/_action/status'),
                ('/tmr_arm_controller/follow_joint_trajectory/_action/cancel_goal', '/tmr_arm_controller/follow_joint_trajectory2/_action/cancel_goal'),
                ('/tmr_arm_controller/follow_joint_trajectory/_action/get_result', '/tmr_arm_controller/follow_joint_trajectory2/_action/get_result'),
                ('/tmr_arm_controller/follow_joint_trajectory/_action/send_goal', '/tmr_arm_controller/follow_joint_trajectory2/_action/send_goal'),

            ]
    )


    return LaunchDescription([ tm_driver_node, tm_driver_node2, static_tf, robot_state_publisher, rviz_node, run_moveit_cpp_node])
    #return LaunchDescription([ tm_driver_node, tm_driver_node2, static_tf, robot_state_publisher, robot_state_publisher2, rviz_node, run_moveit_cpp_node, run_moveit_cpp_node2])
    
