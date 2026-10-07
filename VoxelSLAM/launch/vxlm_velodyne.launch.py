import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('voxel_slam')
    rviz_config_file = os.path.join(pkg_dir, 'rviz_cfg', 'back.rviz')
    config_file = os.path.join(pkg_dir, 'config', 'velodyne.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz'
        ),
        
        Node(
            package='voxel_slam',
            executable='voxelslam',
            name='voxelslam',
            output='screen',
            parameters=[
                config_file,
                {'finish': False}
            ]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(LaunchConfiguration('rviz'))
        )
    ])
