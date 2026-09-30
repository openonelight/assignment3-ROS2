from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('hik_camera')
    return LaunchDescription([
        DeclareLaunchArgument('serial_number', description='Exact USB serial from MVS'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'camera.yaml')),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(package='hik_camera', executable='camera_node', name='hik_camera', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'serial_number': ParameterValue(LaunchConfiguration('serial_number'), value_type=str)}]),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(share, 'rviz', 'camera.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
