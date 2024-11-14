import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess

import os

def generate_launch_description():
    # use_sim_time を True に設定
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    share_dir = get_package_share_directory('sllidar_ros2')
    rviz_config_file = os.path.join(
            get_package_share_directory('sllidar_ros2'),
            'rviz',
            'cartographer.rviz')

    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', 
                                                    default=os.path.join(share_dir, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='rplidar_cartographer.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    simulator_package = 'arcanain_simulator'
    file_path = os.path.expanduser('~/ros2_ws/src/arcanain_simulator/urdf/mobile_robot.urdf.xml')
    with open(file_path, 'r') as file:
        robot_description = file.read()

    
    return LaunchDescription([

        ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'play', 
                    os.path.expanduser('~/ドキュメント/ros2_bag_file/lidar_test_run_1110_1731221219'),
                    '--rate', '0.5',
                    '--remap', '/filtered/scan:=/scan',
                    '/tf:=/ignore_tf', 
                    '/tf_static:=/ignore_tf_static'
                ],
                output='screen'
            ),

        Node(package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
            ),

        Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='both',
                parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
            ),

        Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='both',
                parameters=[{'use_sim_time': use_sim_time}]
            ),

        Node(
                package='gnss_preprocessing',
                executable='gnss_preprocessing',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            ),

        Node(
                package=simulator_package,
                executable='odrive_gps_odom_pub',
                output="screen",
                parameters=[{'use_sim_time': use_sim_time}]
            ),


        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'resolution': 0.05, 'use_sim_time': use_sim_time}],
            ),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir, '-configuration_basename', configuration_basename],
            ),
    ])
