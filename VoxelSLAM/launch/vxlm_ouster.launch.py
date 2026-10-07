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
    config_file = os.path.join(pkg_dir, 'config', 'ouster.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz'
        ),
        DeclareLaunchArgument(
            'lid_topic',
            default_value='/os1_cloud_node/points',
            description='Lidar topic'
        ),
        DeclareLaunchArgument(
            'bag_path',
            default_value='',
            description='Path to the rosbag2 directory to process'
        ),
        DeclareLaunchArgument(
            'bagname',
            default_value='long',
            description='Name of the bag, used for saving outputs'
        ),

        Node(
            package='voxel_slam',
            executable='voxelslam',
            name='voxelslam',
            output='screen',
            parameters=[
                config_file,
                {
                    'finish': False,
                    'General.lid_topic': LaunchConfiguration('lid_topic'),
                    'General.bag_path': LaunchConfiguration('bag_path'),
                    'General.bagname': LaunchConfiguration('bagname'),
                }
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
