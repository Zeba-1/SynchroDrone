import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    NB_DRONES = int(os.getenv('NB_DRONES', '1'))

    # Setup project paths
    pkg_project_crazyflie_gazebo = get_package_share_directory('ros_gz_crazyflie_bringup')

    # Setup to launch a crazyflie gazebo simulation from the ros_gz_crazyflie project
    crazyflie_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_crazyflie_gazebo, 'launch', 'crazyflie_simulation.launch.py')) ,
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

    return LaunchDescription([
        crazyflie_simulation,
        wall_following
        ])