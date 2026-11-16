"""测试专用：只启动原图服务，定位使用 运动学仿真的 TF。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'yaml_filename': LaunchConfiguration('map'), 'use_sim_time': False}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
             parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': ['map_server'], 'bond_timeout': 30.0}]),
    ])
