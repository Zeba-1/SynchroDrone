import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    NB_DRONES = int(os.getenv('NB_DRONES', '1'))

    # Setup project paths'''
    pkg_project_crazyswarm2 = get_package_share_directory('crazyflie')
    pkg_multiranger_bringup = get_package_share_directory('crazyflie_ros2_multiranger_bringup')
    crazyflies_yaml = os.path.join(
        pkg_multiranger_bringup,
        'config',
        'crazyflie_real_crazyswarm2.yaml')

    # Start up a crazyflie server through the Crazyswarm2 project
    crazyflie_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_project_crazyswarm2, 'launch'), '/launch.py']),
        launch_arguments={'crazyflies_yaml_file': crazyflies_yaml, 'backend': 'cflib', 'mocap': 'False', 'rviz': 'False'}.items()
    )

    # Start a velocity multiplexer node for the crazyflie
    crazyflie_vel_mux = Node(
            package='crazyflie',
            executable='vel_mux.py',
            name='vel_mux',
            output='screen',
            parameters=[{'hover_height': 0.3},
                        {'incoming_twist_topic': '/cmd_vel'},
                        {'robot_prefix': 'crazyflie_real'},]
        )

    # start a simple mapper node
    simple_mapper = Node(
        package='crazyflie_ros2_multiranger_simple_mapper',
        executable='simple_mapper_multiranger',
        name='simple_mapper',
        output='screen',
        parameters=[
            {'robot_prefix': 'crazyflie_real'},
            {'use_sim_time': False}
        ]
    )

    drone_name = []
    for i in range(NB_DRONES):
        drone_name.append('crazyflie' + str(i))

    # start a path finder node with a delay of 5 seconds
    wall_following = Node(
        package='crazyflie_ros2_path_finder_controller',
        executable='path_finder_controller',
        name='path_finder_controller',
        output='screen',
        parameters=[
            {'robot_prefix': drone_name},
            {'use_sim_time': False},
            {'delay': 5.0},
            {'max_turn_rate': 0.7},
            {'max_forward_speed': 0.5},
            {'wall_following_direction': 'right'}
        ]
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('crazyflie_ros2_multiranger_bringup'),
        'config',
        'real_mapping.rviz')

    rviz = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{
                "use_sim_time": False
            }]
            )

    return LaunchDescription([
        crazyflie_real,
        crazyflie_vel_mux,
        simple_mapper,
        wall_following,
        rviz
        ])